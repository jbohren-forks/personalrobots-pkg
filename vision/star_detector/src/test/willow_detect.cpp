#include "star_detector/detector.h"
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
        std::cerr << "Usage: ./willow_detect -i image.pgm -o image.key "
                  << "[-t warp.xfm -i2 warped.pgm -o2 warped.key] [options]\n"
                  << "  number of points to take: -p 800\n"
                  << "  number of scales:         -s 7\n"
                  << "  response threshold:       -thres 30\n"
                  << "  line threshold:           -line 10\n"
                  << "  line threshold 2:         -line2 8\n"
                  << "  scale upper bound 1:      -ub1 6\n"
                  << "  scale lower bound 1:      -lb1 2\n"
                  << "  scale upper bound 2:      -ub2 9\n"
                  << "  scale lower bound 2:      -lb2 2.8\n";
        return 0;
    }
    
    char *image_file = NULL, *warped_file = NULL, *transform_file = NULL;
    int num_points = 800;
    std::string key_file = "source.key", warped_key_file = "warped.key";
    int scales = 7;
    float thres = StarDetector::DEFAULT_RESPONSE_THRESHOLD;
    float line_thres = StarDetector::DEFAULT_LINE_THRESHOLD_PROJECTED;
    float line_thres2 = StarDetector::DEFAULT_LINE_THRESHOLD_BINARIZED;
    float scale_ub1 = -1, scale_lb1 = -1, scale_ub2 = -1, scale_lb2 = -1;
    
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
        if (! strcmp(argv[arg], "-line2"))
            line_thres2 = atof(argv[++arg]);
        if (! strcmp(argv[arg], "-ub1"))
            scale_ub1 = atof(argv[++arg]);
        if (! strcmp(argv[arg], "-lb1"))
            scale_lb1 = atof(argv[++arg]);
        if (! strcmp(argv[arg], "-ub2"))
            scale_ub2 = atof(argv[++arg]);
        if (! strcmp(argv[arg], "-lb2"))
            scale_lb2 = atof(argv[++arg]);
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
    std::vector<Keypoint> keypts;
    {
      StarDetector detector(cvSize(W, H), scales, thres, line_thres, line_thres2);
      Timer t("Willow detector");
      detector.DetectPoints(source, std::back_inserter(keypts));
    }

    if (scale_ub1 > 0)
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    ScaleUpperBound(scale_ub1)),
                     keypts.end());
    if (scale_lb1 > 0)
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    ScaleLowerBound(scale_lb1)),
                     keypts.end());

    if (warped_file) {
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    OutsideSource(warped->width, warped->height, transform)),
                     keypts.end());
    }
    num_points = std::min(num_points, (int)keypts.size());
    
    if (warped_file) {
        // Find keypoints in warped image
        std::vector<Keypoint> warp_keypts;
        {
          StarDetector warp_detector(cvSize(warped->width, warped->height),
                                     scales, thres, line_thres, line_thres2);
          Timer t("Willow detector (warped)");
          warp_detector.DetectPoints(warped, std::back_inserter(warp_keypts));
        }
        if (scale_ub2 > 0)
            warp_keypts.erase(std::remove_if(warp_keypts.begin(), warp_keypts.end(),
                                             ScaleUpperBound(scale_ub2)),
                              warp_keypts.end());
        if (scale_lb2 > 0)
            warp_keypts.erase(std::remove_if(warp_keypts.begin(), warp_keypts.end(),
                                             ScaleLowerBound(scale_lb2)),
                              warp_keypts.end());
        warp_keypts.erase(std::remove_if(warp_keypts.begin(), warp_keypts.end(),
                                         OutsideSource(W, H, inv)),
                          warp_keypts.end());
        num_points = std::min(num_points, (int)warp_keypts.size());
        KeepBestPoints(warp_keypts, num_points);

        WriteKeypoints(warped_key_file, warp_keypts);
    }

    KeepBestPoints(keypts, num_points);
    WriteKeypoints(key_file, keypts);
    
    cvReleaseMat(&inv);
    cvReleaseMat(&transform);
    cvReleaseImage(&warped);
    cvReleaseImage(&source);
    
    return 0;
}
