#include "keypoint_utils.h"
#include "transform_utils.h"
#include <iostream>
#include <cstdio>
#include <cassert>
#include <cstdlib>

int main( int argc, char** argv )
{
    if (argc < 4) {
        std::cerr << "Usage: ./repeatability -k1 img1.key -k2 img2.key "
                  << "-t img1to2.xfm [options]\n"
                  << "  scale factor:    -sf 1.5\n"
                  << "  do rounding:     -r\n"
                  << "  pure scaling:    -s\n"
                  << "  normalize size:  -n 15\n";
        return 0;
    }

    char *key1_file = NULL, *key2_file = NULL, *transform_file = NULL;
    float scale_factor = 10;
    bool round = false, normalize = true, plain = false, pure_scaling = false;
    
    int arg = 0;
    while (++arg < argc) {
        if (! strcmp(argv[arg], "-k1"))
            key1_file = argv[++arg];
        if (! strcmp(argv[arg], "-k2"))
            key2_file = argv[++arg];
        if (! strcmp(argv[arg], "-t"))
            transform_file = argv[++arg];
        if (! strcmp(argv[arg], "-sf")) {
            scale_factor = atof(argv[++arg]);
            normalize = false;
        }
        if (! strcmp(argv[arg], "-r"))
            round = true;
        if (! strcmp(argv[arg], "-s"))
            pure_scaling = true;
        if (! strcmp(argv[arg], "-n")) {
            scale_factor = atof(argv[++arg]);
            normalize = true;
        }
        if (! strcmp(argv[arg], "-plain"))
            plain = true;
    }

    assert(key1_file);
    assert(key2_file);
    assert(transform_file);
    
    std::vector<KeypointFl> pts1 = ReadKeypointsFl(key1_file);
    std::vector<KeypointFl> pts2 = ReadKeypointsFl(key2_file);
    unsigned num_pts = pts1.size();
    assert(num_pts == pts2.size());
    
    CvMat* transform = cvCreateMat(3, 3, CV_32FC1);
    ReadTransform(transform_file, transform);
    float pure_scale_factor = transform->data.fl[0];
    
    typedef std::vector<KeypointFl>::iterator iter;
    for (iter i = pts1.begin(); i != pts1.end(); ++i) {
        CvPoint2D32f pt = MapPoint(cvPoint2D32f(i->x, i->y), transform);
        if (round) {
            i->x = (int)(pt.x + 0.5);
            i->y = (int)(pt.y + 0.5);
        } else {
            i->x = pt.x;
            i->y = pt.y;
        }
        if (pure_scaling) {
            if (round)
                i->scale = (int)(i->scale*pure_scale_factor + 0.5);
            else
                i->scale *= pure_scale_factor;
        }
    }

    for (int ot = 5; ot <= 40; ot += 5) {
        int corr = OverlapCorrespondences(pts1, pts2, (float)ot / 100, scale_factor, normalize);
        if (plain)
            printf("%d %d %f\n", ot, corr, (float)corr*100/num_pts);
        else 
            printf("Overlap threshold = %d%%: %d correspondences (%f%%)\n",
                   ot, corr, (float)corr*100/num_pts);
    }
    
    return 0;
}
