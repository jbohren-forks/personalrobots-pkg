#include "detector.h"
#include "transform_utils.h"
#include "keypoint_utils.h"
#include "timer.h"
#include <highgui.h>
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <cstdio>

int main( int argc, char** argv )
{
    if (argc == 1) {
        std::cerr << "Usage: ./censure_detect -i image.pgm -o image.key "
                  << "[-t warp.xfm -i2 warped.pgm -o2 warped.key] [options]\n"
                  << "  number of points to take: -p 800\n"
                  << "  number of scales:         -s 7\n"
                  << "  strength threshold:       -thres 30\n"
                  << "  line threshold:           -line 10\n";
        return 0;
    }
    
    char *image_file = NULL, *warped_file = NULL, *transform_file = NULL;
    int num_points = 800;
    std::string key_file = "source.key", warped_key_file = "warped.key";
    int scales = 7;
    float thres = WgDetector::DEFAULT_THRESHOLD;
    float line_thres = WgDetector::DEFAULT_LINE_THRESHOLD;
    
    int arg = 0;
    while (++arg < argc) {
        if (! strcmp(argv[arg], "-i"))
            image_file = argv[++arg];
        if (! strcmp(argv[arg], "-i2"))
            warped_file = argv[++arg];
        if (! strcmp(argv[arg], "-t"))
            transform_file = argv[++arg];
        if (! strcmp(argv[arg], "-o"))
            key_file = argv[++arg];
        if (! strcmp(argv[arg], "-o2"))
            warped_key_file = argv[++arg];
        if (! strcmp(argv[arg], "-p"))
            num_points = atoi(argv[++arg]);
        if (! strcmp(argv[arg], "-s"))
            scales = atoi(argv[++arg]);
        if (! strcmp(argv[arg], "-thres"))
            thres = atof(argv[++arg]);
        if (! strcmp(argv[arg], "-line"))
            line_thres = atof(argv[++arg]);
    }

    assert(image_file);
    assert(!warped_file || transform_file);

    // Load the source image
    IplImage* source = cvLoadImage(image_file, CV_LOAD_IMAGE_GRAYSCALE);
    assert(source);
    int W = source->width;
    int H = source->height;

    // Load the warped image
    IplImage* warped = NULL;
    CvMat *transform = NULL, *inv = NULL;
    if (warped_file) {
        warped = cvLoadImage(warped_file, CV_LOAD_IMAGE_GRAYSCALE);
        
        // Get inverse transform (warped -> source)
        transform = cvCreateMat(3, 3, CV_32FC1);
        ReadTransform(transform_file, transform);
        inv = cvCreateMat(3, 3, CV_32FC1);
        cvInvert(transform, inv);
    }
    
    // Find keypoints in source image
    WgDetector detector(cvSize(W, H), scales, thres, line_thres);
    std::vector<Keypoint> keypts;
    {
        Timer t("Censure detector");
        keypts = detector.DetectPoints(source);
    }

    if (warped_file) {
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    OutsideSource(warped->width, warped->height, transform)),
                     keypts.end());
    }
    if ((int)keypts.size() < num_points) {
        num_points = keypts.size();
        printf("WARNING: Only taking %d points!\n", num_points);
        std::sort(keypts.begin(), keypts.end());
    } else {
        std::partial_sort(keypts.begin(), keypts.begin()+ num_points, keypts.end());
    }
    
    if (warped_file) {
        // Find keypoints in warped image
        WgDetector warp_detector(cvSize(warped->width, warped->height),
                                      scales, thres, line_thres);
        std::vector<Keypoint> warp_keypts;
        {
            Timer t("Censure detector (warped)");
            warp_keypts = warp_detector.DetectPoints(warped);
        }
        /*
        warp_keypts.erase(std::remove_if(warp_keypts.begin(), warp_keypts.end(),
                                         LineThreshold()),
                          warp_keypts.end());
        */
        warp_keypts.erase(std::remove_if(warp_keypts.begin(), warp_keypts.end(),
                                         OutsideSource(W, H, inv)),
                          warp_keypts.end());
        if ((int)warp_keypts.size() < num_points) {
            num_points = warp_keypts.size();
            printf("WARNING: Only taking %d points!\n", num_points);
            std::sort(warp_keypts.begin(), warp_keypts.end());
        } else {
            std::partial_sort(warp_keypts.begin(), warp_keypts.begin() + num_points,
                              warp_keypts.end());
            warp_keypts.erase(warp_keypts.begin() + num_points, warp_keypts.end());
        }

        WriteKeypoints(warped_key_file, warp_keypts);
    }

    keypts.erase(keypts.begin() + num_points, keypts.end());
    WriteKeypoints(key_file, keypts);
    
    cvReleaseMat(&inv);
    cvReleaseMat(&transform);
    cvReleaseImage(&warped);
    cvReleaseImage(&source);
    
    return 0;
}
