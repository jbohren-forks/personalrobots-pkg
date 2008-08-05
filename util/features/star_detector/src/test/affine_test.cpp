#include "detector.h"
#include "transform_utils.h"
#include "keypoint_utils.h"
#include "cv_utils.h"
#include <highgui.h>
#include <algorithm>
#include <cstdio>
#include <iostream>

char src_wndname[] = "Source image";
char out_wndname[] = "Warped image";
IplImage *loaded = NULL, *warped = NULL, *loaded_color = NULL, *warped_color = NULL;
CvMat *map_matrix = NULL, *inv_matrix = NULL;
int angle = 18;
int search_radius = 0;
int W, H;
std::vector<Keypoint> src_keypts;
std::vector<Keypoint> out_keypts;
std::vector<Keypoint> warped_keypts;
bool show_warped_pts = false;
bool show_detected_pts = true;
static const int POINTS_TO_TAKE = 800;

void update() {
    cvCvtColor(warped, warped_color, CV_GRAY2BGR);

    if (show_detected_pts) {
        DrawPoints(warped_color, out_keypts);
    }

    if (show_warped_pts) {
        DrawPoints(warped_color, warped_keypts, CV_RGB(0,0,255));
    }

    cvShowImage(out_wndname, warped_color);
}

void on_radius_trackbar(int r)
{
    printf("%d correspondences found with search radius %d\n",
           CountCorrespondences(warped_keypts, out_keypts, r), r);
}

// define a trackbar callback
void on_trackbar(int h)
{
    // Calculate new rotation matrix
    double angle = h*10;
    CvSize warped_size = FullImageRotation(W, H, angle, map_matrix);
    
    // Make images large enough to hold entire warped image
    cvReleaseImage(&warped);
    cvReleaseImage(&warped_color);
    warped = cvCreateImage(warped_size, IPL_DEPTH_8U, 1);
    warped_color = cvCreateImage(warped_size, IPL_DEPTH_8U, 3);

    // Apply the warp
    cvWarpAffine(loaded, warped, map_matrix);

    // Calculate inverse map matrix
    InvertRotation(map_matrix, inv_matrix);
    
    // Detect and threshold keypoints in the warped image
    StarDetector detector(warped_size, 7, 0);
    out_keypts = detector.DetectPoints(warped);
    out_keypts.erase(std::remove_if(out_keypts.begin(), out_keypts.end(),
                                    OutsideSource(W, H, inv_matrix)),
                     out_keypts.end());
    std::partial_sort(out_keypts.begin(), out_keypts.begin() + POINTS_TO_TAKE,
                      out_keypts.end());
    out_keypts.erase(out_keypts.begin() + POINTS_TO_TAKE, out_keypts.end());

    // Make list of points in the warped image corresponding to keypoints
    // from the source image
    typedef std::vector<Keypoint>::const_iterator iter;
    warped_keypts.clear();
    for (iter i = src_keypts.begin(); i != src_keypts.end(); ++i) {
        CvPoint pt = MapPoint(cvPoint(i->x, i->y), map_matrix);
        int r = i->scale + i->scale / 2;
        if (pt.x > r && pt.x < warped_size.width - r &&
            pt.y > r && pt.y < warped_size.height - r) {
            warped_keypts.push_back(Keypoint(pt.x, pt.y, i->scale, i->response));
        }
    }

    on_radius_trackbar(search_radius);
    
    update();
}

int main( int argc, char** argv )
{
    assert(argc > 1);
    
    loaded = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    assert(loaded);
    W = loaded->width;
    H = loaded->height;
    
    loaded_color = cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 3);
    map_matrix = cvCreateMat(2, 3, CV_32FC1);
    inv_matrix = cvCreateMat(2, 3, CV_32FC1);

    StarDetector detector(cvSize(W,H), 7, 0);
    
    cvNamedWindow(src_wndname);
    cvNamedWindow(out_wndname);
    cvMoveWindow(src_wndname, 1200, 5000);
    cvMoveWindow(out_wndname, 300, 5000);
    cvCreateTrackbar("Angle", out_wndname, &angle, 35, on_trackbar);
    cvCreateTrackbar("Search radius", out_wndname, &search_radius, 5, on_radius_trackbar);

    OnMouseData src_data(src_keypts);
    OnMouseData out_data(out_keypts);
    cvSetMouseCallback(src_wndname, on_mouse_find_keypts, &src_data);
    cvSetMouseCallback(out_wndname, on_mouse_find_keypts, &out_data);

    src_keypts = detector.DetectPoints(loaded);
    std::partial_sort(src_keypts.begin(), src_keypts.begin()+ POINTS_TO_TAKE,
                      src_keypts.end());
    src_keypts.erase(src_keypts.begin() + POINTS_TO_TAKE, src_keypts.end());
    cvCvtColor(loaded, loaded_color, CV_GRAY2BGR);
    DrawPoints(loaded_color, src_keypts);
    cvShowImage(src_wndname, loaded_color);
    
    on_trackbar(angle);

    float* data;
    for(;;){
        int c = cvWaitKey(0);
        switch( (char) c ) {
            case '\x1b':
                printf("Exiting...\n");
                goto exit_main;
            case 'w':
                //printf("Toggling warped points detected from source image.\n");
                show_warped_pts = !show_warped_pts;
                update();
                break;
            case 'd':
                //printf("Toggling points detected in warped image.\n");
                show_detected_pts = !show_detected_pts;
                update();
                break;
            case 't':
                WriteTransform("out.xfm", map_matrix);
                printf("Wrote map_matrix to out.xfm\n");
                break;
            case 'r':
                ReadTransform("out.xfm", map_matrix);
                data = map_matrix->data.fl;
                printf("%f %f %f\n", data[0], data[1], data[2]);
                data += map_matrix->step / sizeof(float);
                printf("%f %f %f\n", data[0], data[1], data[2]);
                break;
            case 'k':
                WriteKeypoints("out.key", warped_keypts);
                printf("Wrote warped keypoints to out.key\n");
                break;
            case 'l':
                warped_keypts = ReadKeypoints("out.key");
                update();
                printf("Loaded out.key as warped keypoints\n");
                break;
        }
    }

exit_main:
    
    cvReleaseImage(&loaded);
    cvReleaseImage(&warped);
    cvReleaseImage(&loaded_color);
    cvReleaseImage(&warped_color);
    cvReleaseMat(&map_matrix);
    cvReleaseMat(&inv_matrix);
    cvDestroyWindow(out_wndname);
    cvDestroyWindow(src_wndname);
    
    return 0;
}
