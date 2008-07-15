#include "censure.h"
#include <highgui.h>
#include <algorithm>
#include <stdio.h>
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
static const float LINE_THRESH = 10;
static const int POINTS_TO_TAKE = 800;

// Returns number of correspondences.
int CountCorrespondences(int r)
{
    typedef std::vector<Keypoint>::const_iterator iter;

    int correspondences = 0;
    
    for (iter i = warped_keypts.begin(); i != warped_keypts.end(); ++i) {
        for (iter j = out_keypts.begin(); j != out_keypts.end(); ++j) {
            if (abs(i->x - j->x) + abs(i->y - j->y) + abs(i->scale - j->scale) <= r) {
                ++correspondences;
                break;
            }
        }
    }

    return correspondences;
}

// Functor to reject points that exceed the line response threshold.
struct LineThreshold
{
    float line_threshold;
    
    LineThreshold(float thresh = LINE_THRESH)
        : line_threshold(thresh)
    {};
    
    inline bool operator() (Keypoint const& pt) {
        return pt.line_response >= line_threshold;
    }
};

void DrawPoints(IplImage* dst, std::vector<Keypoint> const& pts,
                CvScalar color = CV_RGB(0,255,0))
{
    typedef std::vector<Keypoint>::const_iterator iter;

    for (iter i = pts.begin(); i != pts.end(); ++i) {
        int r = i->scale + i->scale / 2;

        cvCircle(dst, cvPoint(i->x, i->y), r, color, 1);
    }
}

CvPoint MapPoint(CvPoint pt, CvMat* map_matrix)
{
    float* row1 = map_matrix->data.fl;
    float* row2 = row1 + map_matrix->step / sizeof(float);

    return cvPoint(pt.x*row1[0] + pt.y*row1[1] + row1[2] + 0.5,
                   pt.x*row2[0] + pt.y*row2[1] + row2[2] + 0.5);
}

// Functor to reject points that project back to somewhere outside
// or on the border of the source image.
struct OutsideSource
{
    inline bool operator() (Keypoint const& pt) {
        CvPoint back = MapPoint(cvPoint(pt.x, pt.y), inv_matrix);
        int offset = 3*(pt.scale + 1);

        return back.x < offset || back.x >= W - offset ||
            back.y < offset || back.y >= H - offset;
    }
};
    
CvSize WarpedSize(int W, int H, CvMat* map_matrix)
{
    CvPoint tl = MapPoint(cvPoint(0,0), map_matrix);
    CvPoint tr = MapPoint(cvPoint(W-1,0), map_matrix);
    CvPoint bl = MapPoint(cvPoint(0,H-1), map_matrix);
    CvPoint br = MapPoint(cvPoint(W-1,H-1), map_matrix);

    int min_x = MIN(tl.x, MIN(tr.x, MIN(bl.x, br.x)));
    int min_y = MIN(tl.y, MIN(tr.y, MIN(bl.y, br.y)));
    int max_x = MAX(tl.x, MAX(tr.x, MAX(bl.x, br.x)));
    int max_y = MAX(tl.y, MAX(tr.y, MAX(bl.y, br.y)));

    //printf("min_x = %d, min_y = %d, max_x = %d, max_y = %d\n", min_x, min_y, max_x, max_y);
    return cvSize(max_x - min_x + 1, max_y - min_y + 1);
}

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
    printf("%d correspondences found with search radius %d\n", CountCorrespondences(r), r);
}

// define a trackbar callback
void on_trackbar(int h)
{
    // Calculate new rotation matrix
    h *= 10;
    cv2DRotationMatrix(cvPoint2D32f(W/2, H/2), h, 1, map_matrix);

    // Make images large enough to hold entire warped image
    CvSize warped_size = WarpedSize(W, H, map_matrix);
    cvReleaseImage(&warped);
    cvReleaseImage(&warped_color);
    warped = cvCreateImage(warped_size, IPL_DEPTH_8U, 1);
    warped_color = cvCreateImage(warped_size, IPL_DEPTH_8U, 3);

    // Adjust the shift to map to the new warped image center
    float* map_data = map_matrix->data.fl;
    map_data[2] += warped_size.width/2 - W/2;
    (map_data + map_matrix->step / sizeof(float))[2] += warped_size.height/2 - H/2;
    cvWarpAffine(loaded, warped, map_matrix);

    // Calculate inverse map matrix
    float* inv_data = inv_matrix->data.fl;
    float alpha = map_data[0], beta = map_data[1], dx = map_data[2];
    float dy = (map_data + map_matrix->step / sizeof(float))[2];
    inv_data[0] = alpha;
    inv_data[1] = -beta;
    inv_data[2] = -alpha*dx + beta*dy;
    inv_data += inv_matrix->step / sizeof(float);
    inv_data[0] = beta;
    inv_data[1] = alpha;
    inv_data[2] = -beta*dx - alpha*dy;
    
    // Detect and threshold keypoints in the warped image
    CensureDetector detector(warped_size);
    out_keypts = detector.DetectPoints(warped);
    out_keypts.erase(std::remove_if(out_keypts.begin(), out_keypts.end(),
                                    LineThreshold()),
                     out_keypts.end());
    out_keypts.erase(std::remove_if(out_keypts.begin(), out_keypts.end(),
                                    OutsideSource()),
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

void on_mouse(int event, int x, int y, int flags, void* param)
{
    typedef std::vector<Keypoint>::const_iterator iter;
    
    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            std::vector<Keypoint> *keypts = (std::vector<Keypoint>*) param;
            
            for (iter i = keypts->begin(); i != keypts->end(); ++i) {
                int r = i->scale + i->scale / 2;

                if (abs(i->x - x) + abs(i->y - y) <= r) {
                    printf("Keypoint: (%d, %d, %d), response = %f, line response = %f\n",
                           i->x, i->y, i->scale, i->response, i->line_response);
                    break;
                }
            }

            break;
    }
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

    CensureDetector detector(cvSize(W,H));
    
    cvNamedWindow(src_wndname);
    cvNamedWindow(out_wndname);
    cvMoveWindow(src_wndname, 1200, 5000);
    cvMoveWindow(out_wndname, 300, 5000);
    cvCreateTrackbar("Angle", out_wndname, &angle, 35, on_trackbar);
    cvCreateTrackbar("Search radius", out_wndname, &search_radius, 5, on_radius_trackbar);
    cvSetMouseCallback(src_wndname, on_mouse, &src_keypts);
    cvSetMouseCallback(out_wndname, on_mouse, &out_keypts);

    src_keypts = detector.DetectPoints(loaded);
    src_keypts.erase(std::remove_if(src_keypts.begin(), src_keypts.end(), LineThreshold()),
    src_keypts.end());
    std::partial_sort(src_keypts.begin(), src_keypts.begin()+ POINTS_TO_TAKE,
                      src_keypts.end());
    src_keypts.erase(src_keypts.begin() + POINTS_TO_TAKE, src_keypts.end());
    cvCvtColor(loaded, loaded_color, CV_GRAY2BGR);
    DrawPoints(loaded_color, src_keypts);
    cvShowImage(src_wndname, loaded_color);
    
    on_trackbar(angle);

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
