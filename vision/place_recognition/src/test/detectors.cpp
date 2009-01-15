#include "detectors.h"
#include <star_detector/detector.h> // Star detector
#include <lowe_key.h>               // SIFT interface
#include "fast.h"                   // FAST detector

namespace features {

std::vector<Keypoint> starKeypoints(IplImage *image, int scales, float threshold)
{
  StarDetector detector( cvSize(image->width, image->height), scales, threshold );
  std::vector<Keypoint> pts;
  detector.DetectPoints(image, std::back_inserter(pts));
  return pts;
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

std::vector<Keypoint> fastKeypoints(IplImage *image, int threshold, int barrier)
{
  int num_corners = 0, num_nonmax = 0;
  unsigned char* imdata = (unsigned char*)image->imageData;
  xy* corners = fast_corner_detect_9(imdata, image->width, image->height, threshold, &num_corners);
  xy* nm = fast_nonmax(imdata, image->width, image->height, corners, num_corners, barrier, &num_nonmax);

  // Copy keypoints over
  std::vector<Keypoint> keypts;
  keypts.reserve(num_nonmax);
  for (int i = 0; i < num_nonmax; ++i)
    keypts.push_back( Keypoint(nm[i].x, nm[i].y, 3, 0) );

  free(corners);
  free(nm);

  return keypts;
}

} // namespace features
