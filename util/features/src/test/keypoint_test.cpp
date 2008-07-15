#include "censure.h"
//#include "cv_wrapper.h"
//#include "nonmax_suppress.h"
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>

char wndname[] = "Keypoints";
IplImage *source = NULL, *disp = NULL;
int thresh = 6;
int line_thresh = 10;
CensureDetector *detector = NULL;
std::vector<Keypoint> keypts;

void DrawPoints(IplImage* src, IplImage* dst, std::vector<Keypoint> const& pts,
                float threshold, float line_threshold)
{
    typedef std::vector<Keypoint>::const_iterator iter;
    
    cvCvtColor(src, dst, CV_GRAY2BGR);

    int num_good = 0, num_rejected = 0;
    for (iter i = pts.begin(); i != pts.end(); ++i) {
        int r = i->scale + i->scale / 2;

        if (abs(i->response) > threshold) {
            if (i->line_response < line_threshold) {
                cvCircle(dst, cvPoint(i->x, i->y), r, cvScalar(0,255,0), 1);
                ++num_good;
            } else {
                cvCircle(dst, cvPoint(i->x, i->y), r, cvScalar(0,0,255), 1);
                ++num_rejected;
            }
        }
    }

    printf("%d good keypoints, %d rejected by line suppression\n",
           num_good, num_rejected);
}

inline float get_threshold(int h) { return 5*h; }
inline float get_line_threshold(int h) { return h; }

// define a trackbar callback
void on_threshold_trackbar(int h)
{
    float threshold = get_threshold(h);
    float line_threshold = get_line_threshold(line_thresh);
    printf("threshold = %f, line threshold = %f\n", threshold, line_threshold);
    
    DrawPoints(source, disp, keypts, threshold, line_threshold);
    
    cvShowImage(wndname, disp);
}

void on_line_trackbar(int h)
{
    float threshold = get_threshold(thresh);
    float line_threshold = get_line_threshold(h);
    printf("threshold = %f, line threshold = %f\n", threshold, line_threshold);
    
    DrawPoints(source, disp, keypts, threshold, line_threshold);
    
    cvShowImage(wndname, disp);
}

void on_mouse(int event, int x, int y, int flags, void* param)
{
    typedef std::vector<Keypoint>::const_iterator iter;
    
    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            float threshold = get_threshold(thresh);
            float line_threshold = get_line_threshold(line_thresh);
            
            for (iter i = keypts.begin(); i != keypts.end(); ++i) {
                int r = i->scale + i->scale / 2;

                if (abs(i->response) > threshold &&
                    abs(i->x - x) + abs(i->y - y) <= r) {
                    printf("Keypoint: (%d, %d, %d)\n", i->x, i->y, i->scale);
                    //detector->LineSuppress(*i, line_threshold, true);
                    break;
                }
            }

            break;
    }
}

int main( int argc, char** argv )
{
    assert(argc > 1);
    
    source = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    assert(source);
    
    int W = source->width, H = source->height;
    disp = cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 3);
    
    detector = new CensureDetector(cvSize(W,H), 7);
    keypts = detector->DetectPoints(source);
    printf("%d keypoints found\n", keypts.size());
    
    cvNamedWindow(wndname, 1);
    cvCreateTrackbar("Threshold", wndname, &thresh, 20, on_threshold_trackbar);
    cvCreateTrackbar("Line threshold", wndname, &line_thresh, 20, on_line_trackbar);
    cvSetMouseCallback(wndname, on_mouse, NULL);
    
    on_threshold_trackbar(thresh);
    
    cvWaitKey(0);
    cvReleaseImage(&source);
    cvReleaseImage(&disp);
    delete detector;
    cvDestroyWindow(wndname);
    
    return 0;
}
