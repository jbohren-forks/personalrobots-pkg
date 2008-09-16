// calonder_descriptor
#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
// star_detector
#include "detector.h"
#include "timer.h"
#include <cvwimage.h> // Google C++ wrappers
#include <highgui.h>
#include <boost/foreach.hpp>
#include <cassert>
#include <cstdio>

using namespace features;

// Usage: ./match_benchmark land30.trees img1.pgm img2.pgm
int main( int argc, char** argv )
{
  static const int NUM_PTS = 500;
  // A threshold of 0 is safest (but slowest), as the signatures are
  // effectively dense vectors. Increasing the threshold makes the
  // signatures sparser, increasing the speed of matching at some cost
  // in the recognition rate. Reasonable thresholds are in [0, 0.01].
  static const float SIG_THRESHOLD = 0.005;

  assert(argc > 3);

  // Load random forests classifier and input images
  RTreeClassifier classifier;
  classifier.read(argv[1]);
  classifier.setThreshold(SIG_THRESHOLD);
  cv::WImageBuffer1_b reference( cvLoadImage(argv[2], CV_LOAD_IMAGE_GRAYSCALE) );
  cv::WImageBuffer1_b query( cvLoadImage(argv[3], CV_LOAD_IMAGE_GRAYSCALE) );
  
  // Detect points in reference image
  StarDetector ref_detector( cvSize(reference.Width(), reference.Height()) );
  ref_detector.interpolate(false);
  std::vector<Keypoint> ref_keypts = ref_detector.DetectPoints(reference.Ipl());
  std::sort(ref_keypts.begin(), ref_keypts.end());
  if ((int)ref_keypts.size() > NUM_PTS)
    ref_keypts.erase(ref_keypts.begin() + NUM_PTS, ref_keypts.end());
  
  // Extract patches and add their signatures to matcher database
  BruteForceMatcher< SparseSignature, CvPoint > matcher;
  //BruteForceMatcher< DenseSignature, CvPoint > matcher;

  {
    Timer ref_timer("Generating reference signatures");
    BOOST_FOREACH( Keypoint &pt, ref_keypts ) {
      cv::WImageView1_b view = extractPatch(reference.Ipl(), pt);
      SparseSignature sig = classifier.getSparseSignature(view.Ipl());
      //DenseSignature sig = classifier.getDenseSignature(view.Ipl());
      matcher.addSignature(sig, cvPoint(pt.x, pt.y));
    }
  }

  // Detect points in query image
  StarDetector query_detector( cvSize(query.Width(), query.Height()) );
  query_detector.interpolate(false);
  std::vector<Keypoint> query_keypts = query_detector.DetectPoints(query.Ipl());
  std::sort(query_keypts.begin(), query_keypts.end());
  if ((int)query_keypts.size() > NUM_PTS)
    query_keypts.erase(query_keypts.begin() + NUM_PTS, query_keypts.end());

  // Find matches
  float distance;
  int index = 0;
  //CvRect rect = cvRect(32, 32, 224, 224);
  std::vector< SparseSignature > query_sigs;
  query_sigs.reserve(query_keypts.size());
  {
    Timer query_timer("Generating query signatures");
    BOOST_FOREACH( Keypoint &pt, query_keypts ) {
      cv::WImageView1_b view = extractPatch(query.Ipl(), pt);
      SparseSignature sig = classifier.getSparseSignature(view.Ipl());
      //DenseSignature sig = classifier.getDenseSignature(view.Ipl());
      query_sigs.push_back(sig);
    }
  }
  int match;
  {
    Timer match_timer("Matching");
    BOOST_FOREACH( SparseSignature &sig, query_sigs ) {
      match = matcher.findMatch(sig, &distance);
      //match = matcher.findMatchInWindow(sig, rect, &distance);
      //printf("%i -> %i, d = %f\n", index, match, distance);
      //++index;

      /* Do something with match */
    }
  }

  return 0;
}
