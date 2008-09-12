#include <vector>
#include <keypoint.h>
#include <cv.h>

namespace features {

std::vector<Keypoint> starKeypoints(IplImage *image, int scales = 7, float threshold = 0);

std::vector<Keypoint> siftKeypoints(IplImage *image);

} // namespace features
