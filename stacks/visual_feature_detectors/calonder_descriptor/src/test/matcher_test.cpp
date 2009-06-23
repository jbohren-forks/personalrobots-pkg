#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"
#include <detector.h>
#include <cvwimage.h>
#include <highgui.h>
#include <boost/foreach.hpp>
#include <vector>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <cstdio>
#include <ctime>

using namespace features;

int main( int argc, char** argv )
{
  assert(argc > 2);

  int patch_size = RandomizedTree::PATCH_SIZE;
  int num_views = 100;
  int num_keypts = 50;

  RTreeClassifier classifier;
  classifier.read(argv[1]);
  classifier.setThreshold(0.005);
  cv::WImageBuffer1_b source( cvLoadImage(argv[2], CV_LOAD_IMAGE_GRAYSCALE) );
  std::ofstream sig_file;
  sig_file.open(argv[3]);

  // Detect test set
  StarDetector detector(cvSize(source.Width(), source.Height()), 7, 0);
  std::vector<Keypoint> keypts = detector.DetectPoints(source.Ipl());
  std::sort(keypts.begin(), keypts.end());
  assert((int)keypts.size() >= num_keypts);
  keypts.erase(keypts.begin() + num_keypts, keypts.end());

  BruteForceMatcher<SparseSignature, CvPoint> matcher;

  // Extract patches and add their signatures to matcher database
  std::vector< cv::WImageView1_b > patches;
  int offset = patch_size / 2;
  BOOST_FOREACH( Keypoint &pt, keypts ) {
    cv::WImageView1_b view(&source, pt.x - offset, pt.y - offset, patch_size, patch_size);
    patches.push_back(view);
    //DenseSignature sig = classifier.getDenseSignature(view.Ipl());
    SparseSignature sig = classifier.getSparseSignature(view.Ipl());
    matcher.addSignature(sig, cvPoint(pt.x, pt.y));
    
    BOOST_FOREACH( float prob, sig )
      sig_file << prob << ' ';
    sig_file << std::endl;
  }

  IplImage* patch = cvCreateImage(cvSize(patch_size, patch_size), IPL_DEPTH_8U, 1);
  Rng rng( std::time(NULL) );
  PatchGenerator make_patch(source.Ipl(), rng);

  float distance;
  float multiplier = 100.0f / num_views;
  int total_correct = 0;
  int index = 0;
  BOOST_FOREACH( Keypoint pt, keypts ) {
    printf("Test point %i:\n", index);

    int correct = 0;
    for (int i = 0; i < num_views; ++i) {
      make_patch(cvPoint(pt.x, pt.y), patch);
      //DenseSignature sig = classifier.getDenseSignature(patch);
      SparseSignature sig = classifier.getSparseSignature(patch);
      int match = matcher.findMatch(sig, &distance);
      if (match == index)
        ++correct;
      printf("%i ", match);
    }
    
    /*
    ublas::vector<float> sig = classifier.getDenseSignature(patches[index].Ipl());
    int match = matcher.findMatch(sig);
    printf("%i", match);
    */
    
    printf("\nCorrect: %i / %i = %f%%\n\n", correct, num_views, multiplier*correct);
    total_correct += correct;
    ++index;
  }

  int total_views = num_keypts*num_views;
  printf("Total correct: %i / %i = %f%%\n", total_correct, total_views,
         100.0f * total_correct / total_views);

  sig_file.close();
  cvReleaseImage(&patch);
  
  return 0;
}
