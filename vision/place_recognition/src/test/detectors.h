#include <vector>
#include <star_detector/keypoint.h>
#include <cv.h>

namespace features {

std::vector<Keypoint> starKeypoints(IplImage *image, int scales = 7, float threshold = 0);

std::vector<Keypoint> siftKeypoints(IplImage *image);

std::vector<Keypoint> fastKeypoints(IplImage *image, int threshold, int barrier);

} // namespace features
