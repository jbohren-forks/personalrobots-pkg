#include "keypoint_utils.h"
#include "transform_utils.h"
#include "timer.h"
#include <lowe_key.h> // SIFT interface
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <cstdio>

int main( int argc, char** argv )
{
    if (argc == 1) {
        std::cerr << "Usage: ./sift_detect -i image.pgm -o image.key "
                  << "[-t warp.xfm -i2 warped.pgm -o2 warped.key] [options]\n"
                  << "  number of points to take: -p 800\n"
                  << "  scale upper bound 1:      -ub1 6\n"
                  << "  scale lower bound 1:      -lb1 2\n"
                  << "  scale upper bound 2:      -ub2 9\n"
                  << "  scale lower bound 2:      -lb2 2.8\n";
        return 0;
    }
    
    char *image_file = NULL, *warped_file = NULL, *transform_file = NULL;
    int num_points = 800;
    std::string key_file = "source.key", warped_key_file = "warped.key";
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
    Image im = ReadPGMFile(image_file);
    int W = im->cols;
    int H = im->rows;
    
    // Find keypoints in source image
    LoweKeypoint lowe_keypts = NULL;
    {
        Timer t("SIFT detector");
        lowe_keypts = GetKeypoints(im);
    }
    std::vector<KeypointFl> keypts;
    for (LoweKeypoint pt = lowe_keypts; pt != NULL; pt = pt->next) {
        keypts.push_back(KeypointFl(pt->col, pt->row, pt->scale, pt->strength));
    }
    if (scale_ub1 > 0)
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    ScaleUpperBound(scale_ub1)),
                     keypts.end());
    if (scale_lb1 > 0)
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    ScaleLowerBound(scale_lb1)),
                     keypts.end());

    Image warped = NULL;
    CvMat *transform = NULL, *inv = NULL;
    if (warped_file) {
        warped = ReadPGMFile(warped_file);

        // Get inverse transform (warped -> source)
        transform = cvCreateMat(3, 3, CV_32FC1);
        ReadTransform(transform_file, transform);
        inv = cvCreateMat(3, 3, CV_32FC1);
        cvInvert(transform, inv);
        
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    OutsideSource(warped->cols, warped->rows, transform)),
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
        LoweKeypoint lowe_warp_keypts = NULL;
        {
            Timer t("SIFT detector (warped)");
            lowe_warp_keypts = GetKeypoints(warped);
        }
        std::vector<KeypointFl> warp_keypts;
        for (LoweKeypoint pt = lowe_warp_keypts; pt != NULL; pt = pt->next) {
            warp_keypts.push_back(KeypointFl(pt->col, pt->row, pt->scale, pt->strength));
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
        if ((int)warp_keypts.size() < num_points) {
            num_points = warp_keypts.size();
            printf("WARNING: Only taking %d points!\n", num_points);
            std::sort(warp_keypts.begin(), warp_keypts.end());
        } else {
            std::partial_sort(warp_keypts.begin(), warp_keypts.begin() + num_points,
                              warp_keypts.end());
            warp_keypts.erase(warp_keypts.begin() + num_points, warp_keypts.end());
        }

        WriteKeypointsFl(warped_key_file, warp_keypts);
    }

    keypts.erase(keypts.begin() + num_points, keypts.end());
    WriteKeypointsFl(key_file, keypts);
    
    cvReleaseMat(&inv);
    cvReleaseMat(&transform);
    delete warped;
    delete im;
    
    return 0;
}
