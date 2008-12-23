#include "place_recognition/vocabulary_tree.h"
#include "place_recognition/sparse_stereo.h"
#include <iterator>
#include <numeric>
#include <sstream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <star_detector/detector.h>
#include <calonder_descriptor/rtree_classifier.h>
#include <calonder_descriptor/matcher.h>
#include <Cv3DPoseEstimateStereo.h>
#include <cvwimage.h>
#include <highgui.h>

using boost::format;
using namespace features;
using namespace vision;

typedef cv::willow::Keypoint voKeypoint;
typedef cv::willow::Keypoints voKeypoints;

static const char image_format[] = "/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff";
static const char classifier_file[] = "/u/mihelich/ros/ros-pkg/vision/calonder_descriptor/src/test/land50_cs.trees";
static const unsigned int NUM_QUERIES = 78;

int main(int argc, char** argv)
{ 
  srand(0);
  std::vector< std::string > query_files;
  int num_objs = NUM_QUERIES;

  query_files.reserve(NUM_QUERIES);

  // List query files and their corresponding files
  for (unsigned int i = 0; i < NUM_QUERIES; ++i) {
    std::stringstream ss;
    ss << format(image_format) % (i * 30);
    query_files.push_back(ss.str());
  }
  printf("%d database images\n", num_objs);

  // Prepare keypoint detector, classifier
  StarDetector detector(cvSize(640, 480), 7, 10.0);
  std::vector<Keypoint> pts;
  RTreeClassifier classifier(true);
  classifier.read(classifier_file);
  unsigned int dimension = classifier.classes();

  // Compute features and their descriptors for each object (image)
  std::vector<float*> buffers;
  std::vector<size_t> buffer_sizes;
  std::vector<unsigned int> objs;
  // Save features for each image
  std::vector<voKeypoints> image_pts(num_objs);
  std::vector< std::vector<float*> > image_features(num_objs);
  int obj = 0;
  BOOST_FOREACH( std::string file, query_files ) {
    // Load left/right images
    cv::WImageBuffer1_b left( cvLoadImage(file.c_str(),
                                          CV_LOAD_IMAGE_GRAYSCALE) );
    boost::replace_last(file, "left", "right");
    cv::WImageBuffer1_b right( cvLoadImage(file.c_str(),
                                           CV_LOAD_IMAGE_GRAYSCALE) );
    SparseStereoFrame frame(left, right);

    // Find keypoints in left image
    pts.resize(0);
    detector.DetectPoints(left.Ipl(), std::back_inserter(pts));

    // Compute descriptors, disparities
    float* sig_buffer = NULL;
    posix_memalign(reinterpret_cast<void**>(&sig_buffer), 16,
                   dimension * sizeof(float) * pts.size());
    buffers.push_back(sig_buffer);
    buffer_sizes.push_back(pts.size());
    float* sig = sig_buffer;
    BOOST_FOREACH( const Keypoint& pt, pts ) {
      // Signature
      cv::WImageView1_b view = extractPatch(left.Ipl(), pt);
      classifier.getSignature(view.Ipl(), sig);
      objs.push_back(obj);

      // Disparity
      double d = frame.lookupDisparity(pt.x, pt.y);
      if (d > 0) {
        image_pts[obj].push_back( voKeypoint(pt.x, pt.y, d, pt.response, pt.scale, NULL) );
        image_features[obj].push_back(sig);
      }

      sig += dimension;
    }
    ++obj;
  }
  size_t num_features = std::accumulate(buffer_sizes.begin(), buffer_sizes.end(), 0);  
  printf("%u features\n", num_features);

  // Copy into single Eigen matrix
  FeatureMatrix features((int)num_features, (int)dimension);
  size_t current_row = 0;
  for (unsigned int i = 0; i < buffers.size(); ++i) {
    features.block(current_row, 0, buffer_sizes[i], dimension) =
      Eigen::Map<FeatureMatrix>(buffers[i], buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    free(buffers[i]);
  }
  buffers.clear();
  
  // Train vocabulary tree
  VocabularyTree tree;
  tree.build(features, objs, 5, 4);
  tree.save("james4.tree");
  
  // Validation
  int training_correct = 0;
  unsigned int N = NUM_QUERIES;
  unsigned int N_show = 10;
  std::vector<VocabularyTree::Match> matches;
  matches.reserve(N);
  // Confusion matrix
  Eigen::MatrixXi confusion((int)NUM_QUERIES, (int)NUM_QUERIES);
  confusion.setZero();
  
  // Validate on training images
  current_row = 0;
  for (unsigned int i = 0; i < NUM_QUERIES; ++i) {
    FeatureMatrix query = features.block(current_row, 0, buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    printf("Training image %u, %u features\n", i, query.rows());

    // Find top N matches
    matches.resize(0);
    tree.find(query, N, std::back_inserter(matches));
    if (matches[0].id == i) ++training_correct;

    // Set up matcher for signatures in query image
    BruteForceMatcher<float, int> matcher(dimension);
    BOOST_FOREACH( float* sig, image_features[i] )
      matcher.addSignature(sig, 0);
    
    // Geometric check (find number of inliers)
    cv::willow::PoseEstimateStereo pose_estimator;
    pose_estimator.setInlierErrorThreshold(3.0);
    pose_estimator.setNumRansacIterations(15);
    // Camera parameters for james4
    pose_estimator.setCameraParams(432.0, 432.0, .088981, 313.7821, 313.7821, 220.407);
    float distance; // dummy
    for (unsigned int j = 0; j < N_show; ++j) {
      // Find matching keypoint pairs
      std::vector< std::pair<int, int> > match_index_pairs;
      unsigned int match_id = matches[j].id;
      for (int index2 = 0; index2 < (int)image_features[match_id].size(); ++index2) {
        float* sig = image_features[match_id][index2];
        int index1 = matcher.findMatch(sig, &distance);
        if (index1 >= 0)
          match_index_pairs.push_back( std::make_pair(index1, index2) );
      }

      // Run RANSAC, no smoothing
      double rot_[9];
      double shift_[3];
      CvMat rot = cvMat(3, 3, CV_64FC1, rot_);
      CvMat shift = cvMat(3, 1, CV_64FC1, shift_);
      int inliers = pose_estimator.estimate(image_pts[i], image_pts[match_id],
                                            match_index_pairs, rot, shift, false);
      confusion(i, match_id) = inliers;
      printf("\tid = %u, score = %f, inliers = %d\n", match_id,
             matches[j].score, inliers);
    }
  }
  printf("\nTraining set: %u / %u\n", training_correct, num_objs);

  // Write out confusion matrix
  std::ofstream confusion_file("inliers.mat");
  for (unsigned i = 0; i < NUM_QUERIES; ++i) {
    for (unsigned j = 0; j < NUM_QUERIES; ++j) {
      confusion_file << confusion(i, j) << " ";
    }
    confusion_file << std::endl;
  }
    
  return 0;
}
