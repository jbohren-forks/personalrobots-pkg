#include "detectors.h"
#include <calonder_descriptor/rtree_classifier.h>
#include <Eigen/Core>
#include <cvwimage.h>
#include <highgui.h>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdio>

using namespace features;

/*
static const char image_format[] = "/u/mihelich/images/holidays/jpg/1%03u00.jpg";
static const char key_format[] = "/u/mihelich/images/holidays/jpg/1%03u00.fast";
static const unsigned int NUM_IMAGES = 500;
static const unsigned int MAX_PER_IMAGE = 2500;
*/
static const char image_format[] = "/wg/stor2/prdata/stereo_video_wg_indoor1/data/left-%04u.bmp";
static const unsigned int NUM_IMAGES = 4961;
static const unsigned int MAX_PER_IMAGE = 300;

static const char classifier_file[] = "/u/prdata/calonder_trees/current.rtc";
static const char sig_file[] = "signatures.dat";
static const char obj_file[] = "objects.dat";
static const unsigned int TARGET_FEATURES = 1000000;

struct FilterBorder
{
  FilterBorder(int width, int height)
    : w(width), h(height)
  {}

  inline bool operator() (Keypoint const& pt) {
    return pt.x < 16 || pt.x >= w - 16 ||
           pt.y < 16 || pt.y >= h - 16;
  }

  int w, h;
};

int main(int argc, char** argv)
{
  std::ofstream sig_out(sig_file, std::ofstream::binary);
  std::ofstream obj_out(obj_file, std::ofstream::binary);
  RTreeClassifier classifier;
  classifier.read(classifier_file);
  unsigned int dimension = classifier.classes();

  std::vector<Keypoint> pts;
  unsigned int num_pts = 0;
  float* buffer = (float*) Eigen::ei_aligned_malloc(dimension * sizeof(float));
  for (unsigned int i = 0; i < NUM_IMAGES; ++i) {
    std::string image_name = (boost::format(image_format) % i).str();
    printf("Processing %s... ", image_name.c_str());
    cv::WImageBuffer1_b image( cvLoadImage(image_name.c_str(), CV_LOAD_IMAGE_GRAYSCALE) );

    // Find keypoints
    pts = fastKeypoints(image.Ipl(), 12, 9);
    //pts = starKeypoints(image.Ipl(), 7, 20.0);

    // Filter keypoints too close to border
    pts.erase( std::remove_if(pts.begin(), pts.end(),
                              FilterBorder(image.Width(), image.Height())),
               pts.end());

    // Restrict number of keypoints from one image
    KeepBestPoints(pts, MAX_PER_IMAGE);
    
    // Write out keypoints
    //WriteKeypoints( (boost::format(key_format) % i).str(), pts);
    
    // Compute descriptors and write to file
    BOOST_FOREACH( Keypoint pt, pts ) {
      cv::WImageView1_b view = extractPatch(image.Ipl(), pt);
      classifier.getSignature(view.Ipl(), buffer);
      sig_out.write((char*)buffer, dimension * sizeof(float));
      obj_out.write((char*)&i, sizeof(unsigned int));
    }

    num_pts += pts.size();
    printf("%u keypoints (%u total)\n", pts.size(), num_pts);
    if (num_pts > TARGET_FEATURES)
      break;
  }
  Eigen::ei_aligned_free(buffer);

  printf("%u features\n", num_pts);

  return 0;
}
