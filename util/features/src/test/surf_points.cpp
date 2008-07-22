#include "surf_points.h"

// Surflib includes
#include <surflib.h>

using namespace surf;

std::vector<KeypointFl> DetectSurfPoints(Image* im, bool doubled_image,
                                         int subsampling_factor)
{
    // Default params
    static const int octaves = 4;
    static const double thres = 4.0;
    static const int init_lobe = 3;

    // Create the integral image
    Image iimage(im, doubled_image);

    // Extract interest points
    std::vector<Ipoint> ipts;
    ipts.reserve(1000);

    FastHessian fh(&iimage, ipts, thres, doubled_image, init_lobe*3,
                   subsampling_factor, octaves);
    fh.getInterestPoints();

    // Copy keypoints into our own format
    std::vector<KeypointFl> keypts;
    keypts.reserve(ipts.size());
    
    typedef std::vector<Ipoint>::const_iterator iter;
    for (iter i = ipts.begin(); i != ipts.end(); ++i) {
        keypts.push_back(KeypointFl(i->x, i->y, i->scale, i->strength));
    }
    
    return keypts;
}
