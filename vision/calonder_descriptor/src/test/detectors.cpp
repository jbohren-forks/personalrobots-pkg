#include "detectors.h"
#include <detector.h> // Star detector
#include <lowe_key.h> // SIFT interface

namespace features {

std::vector<Keypoint> starKeypoints(IplImage *image, int scales, float threshold)
{
  StarDetector detector( cvSize(image->width, image->height), scales, threshold );
  detector.interpolate(false);
  return detector.DetectPoints(image);
}

std::vector<Keypoint> siftKeypoints(IplImage *image)
{
  // Allocate Lowe image without his bloody memory pools
  ImageSt im;
  im.rows = image->height;
  im.cols = image->width;
  float *data = new float[im.rows*im.cols];
  im.pixels = new float*[im.rows];
  float *row = data;
  for (int i = 0; i < im.rows; ++i) {
    im.pixels[i] = row;
    row += im.cols;
  }

  // Copy into Lowe image format
  for (int y = 0; y < im.rows; ++y) {
    for (int x = 0; x < im.cols; ++x) {
      im.pixels[y][x] = ((float) CV_IMAGE_ELEM(image, uchar, y, x)) / 255.0;
    }
  }

  // Detect keypoints
  LoweKeypoint lowe_keypts = NULL;
  lowe_keypts = GetKeypoints(&im);

  // Copy linked list into vector
  std::vector<Keypoint> keypts;
  for (LoweKeypoint pt = lowe_keypts; pt != NULL; pt = pt->next) {
    keypts.push_back(Keypoint( (int)(pt->col + 0.5), (int)(pt->row + 0.5),
                               pt->scale, pt->strength ));
  }

  // Clean up
  delete[] im.pixels[0];
  delete[] im.pixels;
  
  return keypts;
}

} // namespace features
