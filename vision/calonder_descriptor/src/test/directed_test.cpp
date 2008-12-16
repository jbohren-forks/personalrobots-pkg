#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
#include "fast.h"
#include <cvwimage.h>
#include <highgui.h>
#include <vector>
#include <ctime>
#include <cstdio>
#include <cstdlib>

using namespace features;

xy* fast(const IplImage* image, int threshold, int barrier, int* num_nonmax)
{
  int num_corners = 0;
  unsigned char *imdata = (unsigned char*)image->imageData;
  xy* corners = fast_corner_detect_9(imdata, image->width, image->height, threshold, &num_corners);
  xy* nm = fast_nonmax(imdata, image->width, image->height, corners, num_corners, barrier, num_nonmax);
  free(corners);
  return nm;
}

int main(int argc, char** argv)
{
  static const char im_name[] = "/u/konolige/vslam/data/indoor1/left-1000.ppm";
  static const char tree_name[] = "cpp.tree";
  cv::WImageBuffer1_b im( cvLoadImage(im_name, CV_LOAD_IMAGE_GRAYSCALE) );
  int num_corners = 0;
  xy* kp = fast(im.Ipl(), 150, 40, &num_corners);
  printf("%d keypoints\n", num_corners);

  std::vector<BaseKeypoint> base_set;
  base_set.reserve(num_corners);
  for (int i = 0; i < num_corners; ++i)
    base_set.push_back( BaseKeypoint(kp[i].x, kp[i].y, im.Ipl()) );
  
  RTreeClassifier cl(true);
  //Rng rng( std::time(0) );
  Rng rng( 0 );
  /*
  PatchGenerator make_patch(NULL, rng);
  make_patch.addWhiteNoise(true);
  static const double TWENTY_DEG = 0.349066;
  make_patch.setThetaBounds(-TWENTY_DEG, TWENTY_DEG);
  make_patch.setPhiBounds(-TWENTY_DEG, TWENTY_DEG);
  make_patch.setLambdaBounds(0.85, 1.15);
  */

  size_t sig_size = num_corners;
  cl.train(base_set, rng, /*make_patch,*/ 25, 10, 1000, num_corners); // Only 20 views?
  cl.write(tree_name);

  BruteForceMatcher<CvPoint> matcher(cl.classes());
  float* sig_buffer = NULL;
  posix_memalign((void**)&sig_buffer, 16, sig_size * sizeof(float) * base_set.size());
  float* sig = sig_buffer;
  BOOST_FOREACH( BaseKeypoint &pt, base_set ) {
    cv::WImageView1_b patch = extractPatch(im.Ipl(), pt);
    cl.getFloatSignature(patch.Ipl(), sig);
    
    float sum = 0;
    for (int i = 0; i < num_corners; ++i) {
      float elem = sig[i];
      printf("%.3f, ", elem);
      sum += elem;
    }
    printf("sum = %f\n", sum);
    matcher.addSignature(sig, cvPoint(pt.x, pt.y));
    sig += sig_size;
  }

  sig = (float*) malloc(sig_size * sizeof(float));
  BOOST_FOREACH( BaseKeypoint &pt, base_set ) {
    cv::WImageView1_b patch = extractPatch(im.Ipl(), pt);
    cl.getFloatSignature(patch.Ipl(), sig);
    float distance = 0;
    int match = matcher.findMatch(sig, &distance);
    printf("match = %d, distance = %f\n", match, distance);
  }
  free(sig);

  free(sig_buffer);
  free(kp);
  return 0;
}
