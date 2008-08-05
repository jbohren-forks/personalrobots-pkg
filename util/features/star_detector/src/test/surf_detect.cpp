#include "surf_points.h"
#include "keypoint_utils.h"
#include "transform_utils.h"
#include "timer.h"
#include <imload.h> // SURF image loading
#include <image.h>  // SURF image class
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <iostream>

using namespace surf;

int main( int argc, char** argv )
{
    if (argc == 1) {
        std::cerr << "Usage: ./surf_detect -i image.pgm -o image.key "
                  << "[-t warp.xfm -i2 warped.pgm -o2 warped.key] [options]\n"
                  << "  number of points to take: -p 800\n"
                  << "  use image doubling:       -d\n"
                  << "  subsampling step:         -ss 2\n";
        return 0;
    }
    
    char *image_file = NULL, *warped_file = NULL, *transform_file = NULL;
    int num_points = 800;
    std::string key_file = "source.key", warped_key_file = "warped.key";
    bool doubled_image = false;
    int subsampling = 2;
    
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
        if (! strcmp(argv[arg], "-d"))
            doubled_image = true;
        if (! strcmp(argv[arg], "-ss"))
            subsampling = atoi(argv[++arg]);
    }

    assert(image_file);
    assert(!warped_file || transform_file);

    // Load the source image
    ImLoad ImageLoader;
    Image* im = ImageLoader.readImage(image_file);
    int W = im->getWidth();
    int H = im->getHeight();

    Image* warped = NULL;
    CvMat *transform = NULL, *inv = NULL;
    if (warped_file) {
        warped = ImageLoader.readImage(warped_file);

        // Get inverse transform (warped -> source)
        transform = cvCreateMat(3, 3, CV_32FC1);
        ReadTransform(transform_file, transform);
        inv = cvCreateMat(3, 3, CV_32FC1);
        cvInvert(transform, inv);
    }
    
    // Find keypoints in source image
    std::vector<KeypointFl> keypts;
    {
        Timer t("SURF detector");
        keypts = DetectSurfPoints(im, doubled_image, subsampling);
    }
    if (warped_file) {
        keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                                    OutsideSource(warped->getWidth(), warped->getHeight(), transform)),
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
        std::vector<KeypointFl> warp_keypts;
        {
            Timer t("SURF detector (warped)");
            warp_keypts = DetectSurfPoints(warped, doubled_image, subsampling);
        }
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
