#ifndef FEATURES_TEST_SURF_POINTS_H
#define FEATURES_TEST_SURF_POINTS_H

#include <string>
#include <vector>
#include "keypoint_utils.h"

namespace surf {
    class Image;
}

std::vector<KeypointFl> DetectSurfPoints(surf::Image* im, bool doubled_image = false,
                                         int subsampling_factor = 2);

#endif
