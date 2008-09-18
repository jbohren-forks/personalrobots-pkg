// calonder_descriptor
#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
// star_detector
#include "star_detector/detector.h"
#include "timer.h"
#include <cvwimage.h> // Google C++ wrappers
#include <highgui.h>
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <cstdio>

using namespace features;
using namespace boost::accumulators;

void print_signature_stats(std::vector< SparseSignature > const& sigs)
{
  accumulator_set<int, stats<tag::variance> > acc;
  BOOST_FOREACH( SparseSignature const& sig, sigs ) {
    acc( sig.nnz() );
  }

  printf("\tMean length = %.1f, stddev = %.1f\n", mean(acc), sqrt(variance(acc)) );
}

// Usage: ./match_benchmark land30.trees img1.pgm img2.pgm
int main( int argc, char** argv )
{
  static const unsigned NUM_PTS = 500;
  // A threshold of 0 is safest (but slowest), as the signatures are
  // effectively dense vectors. Increasing the threshold makes the
  // signatures sparser, increasing the speed of matching at some cost
  // in the recognition rate. Reasonable thresholds are in [0, 0.01].
  //static const float SIG_THRESHOLD = 0.005;

  assert(argc > 4);

  // Load random forests classifier and input images
  RTreeClassifier classifier;
  classifier.read(argv[1]);
  classifier.setThreshold(atof(argv[4]));
  cv::WImageBuffer1_b reference( cvLoadImage(argv[2], CV_LOAD_IMAGE_GRAYSCALE) );
  cv::WImageBuffer1_b query( cvLoadImage(argv[3], CV_LOAD_IMAGE_GRAYSCALE) );
  
  // Detect points in reference image
  StarDetector ref_detector( cvSize(reference.Width(), reference.Height()) );
  std::vector<Keypoint> ref_keypts;
  ref_detector.DetectPoints(reference.Ipl(), std::back_inserter(ref_keypts));
  KeepBestPoints(ref_keypts, NUM_PTS);
  if (ref_keypts.size() < NUM_PTS) {
    printf("WARNING: Only %u reference keypoints\n", ref_keypts.size());
  }
  
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
  print_signature_stats(matcher.signatures());
  
  // Detect points in query image
  StarDetector query_detector( cvSize(query.Width(), query.Height()) );
  std::vector<Keypoint> query_keypts;
  query_detector.DetectPoints(query.Ipl(), std::back_inserter(query_keypts));
  KeepBestPoints(query_keypts, NUM_PTS);
  if (query_keypts.size() < NUM_PTS) {
    printf("WARNING: Only %u query keypoints\n", query_keypts.size());
  }

  // Find matches
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
  print_signature_stats(query_sigs);
  
  int match;
  float distance;
  //int index = 0;
  //CvRect rect = cvRect(32, 32, 224, 224);
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
