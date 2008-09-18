#include "star_detector/detector.h"
#include "timer.h"
#include <cv.h>
#include <highgui.h>

int main( int argc, char** argv )
{
  assert(argc > 1);
  
  IplImage* source = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  StarDetector detector(cvSize(source->width, source->height));
  std::vector<Keypoint> keypts;
  
  detector.DetectPoints(source, std::back_inserter(keypts));
  {
    Timer t("Willow detector (20x)");
    for (int i = 0; i < 20; ++i) {
      keypts.clear();
      detector.DetectPoints(source, std::back_inserter(keypts));
      //detector.InterpolateScales(keypts.begin(), keypts.end());
    }
  }
  
  return 0;
}
