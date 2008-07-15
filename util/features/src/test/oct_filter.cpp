#include "censure.h"
#include <highgui.h>
#include <stdio.h>
#include <iostream>

char wndname[] = "Pseudo-octagonal filter";
IplImage *source = NULL, *scaled = NULL;
int filt_scale = 1;
CensureDetector *detector = NULL;

// define a trackbar callback
void on_trackbar(int h)
{
    h = MAX(h, 1) - 1;

    IplImage* filtered = detector->m_responses[h];
    double min_val, max_val;
    cvMinMaxLoc(filtered, &min_val, &max_val);
    //printf("Min: %f, Max: %f\n", min_val, max_val);
    double scale = 1.0 / (max_val - min_val);
    cvConvertScale(filtered, scaled, scale, -min_val*scale);
    //cvMinMaxLoc(scaled, &min_val, &max_val);
    //printf("Min: %f, Max: %f\n", min_val, max_val);

    cvShowImage(wndname, scaled);
}

int main( int argc, char** argv )
{
    assert(argc > 1);
    
    source = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    assert(source);
    
    int W = source->width, H = source->height;
    scaled = cvCreateImage(cvSize(W,H), IPL_DEPTH_32F, 1);
    
    detector = new CensureDetector(cvSize(W,H));
    std::vector<Keypoint> keypts = detector->DetectPoints(source);
    printf("%d maxima found\n", keypts.size());
    
    cvNamedWindow(wndname, 1);
    cvCreateTrackbar("Scale", wndname, &filt_scale, 7, on_trackbar);
    on_trackbar(1);
    
    cvWaitKey(0);
    cvReleaseImage(&source);
    cvReleaseImage(&scaled);
    delete detector;
    cvDestroyWindow(wndname);
    
    return 0;
}
