// calonder_descriptor
#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
// star_detector
#include "star_detector/detector.h"
#include "timer.h"
#include <cvwimage.h> // Google C++ wrappers
#include <highgui.h>
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <cstdio>

using namespace features;
using namespace boost::accumulators;

/*
void print_signature_stats(std::vector< float* > const& sigs)
{
  accumulator_set<int, stats<tag::variance> > acc;
  float max_value = 0;
  BOOST_FOREACH( const float* sig, sigs ) {
    acc( sig.nnz() );
    BOOST_FOREACH( float val, sig ) {
      if (val > max_value) max_value = val;
    }
  }

  printf("\t%u signatures, mean length = %.1f, stddev = %.1f, max value = %f\n",
         sigs.size(), mean(acc), sqrt(variance(acc)), max_value);
}
*/

// Usage: ./match_benchmark land30.trees img1.pgm img2.pgm
int main( int argc, char** argv )
{
  static const unsigned NUM_PTS = 1000;
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
  // Partition into bright and dark features
  typedef std::vector<Keypoint>::iterator KeyptIterator;
  KeyptIterator ref_dark_begin = std::partition(ref_keypts.begin(), ref_keypts.end(),
                                                DarkOrBright());
  
  // Extract patches and add their signatures to matcher database
  BruteForceMatcher<CvPoint> bright_matcher, dark_matcher;

  int sig_size = classifier.classes();
  float* sig_buffer = NULL;
  posix_memalign(reinterpret_cast<void**>(&sig_buffer), 16, sig_size * sizeof(float) * ref_keypts.size());
  float* sig = sig_buffer;
  {
    Timer ref_timer("Generating reference signatures");
    
    BOOST_FOREACH( Keypoint &pt, boost::make_iterator_range(ref_keypts.begin(),
                                                            ref_dark_begin) ) {
      cv::WImageView1_b view = extractPatch(reference.Ipl(), pt);
      classifier.getSignature(view.Ipl(), sig);
      bright_matcher.addSignature(sig, cvPoint(pt.x, pt.y));
      sig += sig_size;
    }
    BOOST_FOREACH( Keypoint &pt, boost::make_iterator_range(ref_dark_begin,
                                                            ref_keypts.end()) ) {
      cv::WImageView1_b view = extractPatch(reference.Ipl(), pt);
      classifier.getSignature(view.Ipl(), sig);
      dark_matcher.addSignature(sig, cvPoint(pt.x, pt.y));
      sig += sig_size;
    }
  }
  //print_signature_stats(bright_matcher.signatures());
  //print_signature_stats(dark_matcher.signatures());
  
  // Detect points in query image
  StarDetector query_detector( cvSize(query.Width(), query.Height()) );
  std::vector<Keypoint> query_keypts;
  query_detector.DetectPoints(query.Ipl(), std::back_inserter(query_keypts));
  KeepBestPoints(query_keypts, NUM_PTS);
  if (query_keypts.size() < NUM_PTS) {
    printf("WARNING: Only %u query keypoints\n", query_keypts.size());
  }
  // Partition into bright and dark features
  KeyptIterator query_dark_begin = std::partition(query_keypts.begin(), query_keypts.end(),
                                                  DarkOrBright());
  int bright_features = std::distance(query_keypts.begin(), query_dark_begin);

  // Find matches
  std::vector< float* > query_bright_sigs, query_dark_sigs;
  query_bright_sigs.reserve(bright_features);
  query_dark_sigs.reserve(query_keypts.size() - bright_features);
  float* query_sig_buffer = NULL;
  posix_memalign(reinterpret_cast<void**>(&query_sig_buffer), 16,
                 sig_size * sizeof(float) * query_keypts.size());
  sig = query_sig_buffer;
  {
    Timer query_timer("Generating query signatures");
    BOOST_FOREACH( Keypoint &pt, boost::make_iterator_range(query_keypts.begin(),
                                                            query_dark_begin) ) {
      cv::WImageView1_b view = extractPatch(query.Ipl(), pt);
      classifier.getSignature(view.Ipl(), sig);
      query_bright_sigs.push_back(sig);
      sig += sig_size;
    }
    BOOST_FOREACH( Keypoint &pt, boost::make_iterator_range(query_dark_begin,
                                                            query_keypts.end()) ) {
      cv::WImageView1_b view = extractPatch(query.Ipl(), pt);
      classifier.getSignature(view.Ipl(), sig);
      query_dark_sigs.push_back(sig);
      sig += sig_size;
    }
  }
  //print_signature_stats(query_bright_sigs);
  //print_signature_stats(query_dark_sigs);
  
  int match;
  float distance;
  //int index = 0;
  //CvRect rect = cvRect(32, 32, 224, 224);
  {
    Timer match_timer("Matching");
    // Match bright features
    BOOST_FOREACH( const float* sig, query_bright_sigs ) {
      match = bright_matcher.findMatch(sig, &distance);
      //match = matcher.findMatchInWindow(sig, rect, &distance);
    }
    // Match dark features
    BOOST_FOREACH( const float* sig, query_dark_sigs ) {
      match = dark_matcher.findMatch(sig, &distance);
    }
  }

  free(sig_buffer);
  free(query_sig_buffer);
  
  return 0;
}
