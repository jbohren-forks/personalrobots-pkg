#include "transform_utils.h"
#include <highgui.h>
#include <string>
#include <cstdlib>
#include <cassert>
#include <iostream>

enum Mode { None, Rotate, Transform };

int main( int argc, char** argv )
{
    if (argc == 1) {
        std::cerr << "Usage: ./transform_image -i source.pgm -o warped.pgm [-t warp.xfm] [options]\n"
                  << "  Rotation angle:           -a 60\n"
                  << "  Use homography from file: -h warp.xfm\n";
        return 0;
    }
    
    char *image_file = NULL, *transform_file = NULL, *homography_file = NULL;
    std::string out_file = "warped.pgm";
    float angle = 0; //degrees
    Mode mode = None;
    
    int arg = 0;
    while (++arg < argc) {
        if (! strcmp(argv[arg], "-i"))
            image_file = argv[++arg];
        if (! strcmp(argv[arg], "-o"))
            out_file = argv[++arg];
        if (! strcmp(argv[arg], "-t"))
            transform_file = argv[++arg];
        if (! strcmp(argv[arg], "-a")) {
            angle = atof(argv[++arg]);
            mode = Rotate;
        }
        if (! strcmp(argv[arg], "-h")) {
            homography_file = argv[++arg];
            mode = Transform;
        }
    }
    assert(image_file);
    assert(mode != None);

    IplImage* loaded = cvLoadImage(image_file, CV_LOAD_IMAGE_GRAYSCALE);
    assert(loaded);
    int W = loaded->width;
    int H = loaded->height;

    CvMat* transform = NULL;
    IplImage* warped = NULL;

    if (mode == Rotate) {
        transform = cvCreateMat(2, 3, CV_32FC1);
        CvSize warped_size = FullImageRotation(W, H, angle, transform);
        warped = cvCreateImage(warped_size, IPL_DEPTH_8U, 1);
        cvWarpAffine(loaded, warped, transform);
    }
    else if (mode == Transform) {
        transform = cvCreateMat(3, 3, CV_32FC1);
        ReadTransform(homography_file, transform);
        warped = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
        cvWarpPerspective(loaded, warped, transform);
    }
    
    cvSaveImage(out_file.c_str(), warped);

    if (transform_file)
        WriteTransform(transform_file, transform);
    
    return 0;
}
