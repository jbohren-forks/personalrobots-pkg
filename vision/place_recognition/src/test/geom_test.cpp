#include "place_recognition/vocabulary_tree.h"
#include "place_recognition/sparse_stereo.h"
#include "detectors.h"
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
static const char classifier_file[] = "/u/prdata/calonder_trees/current.rtc";
static const unsigned int NUM_QUERIES = 78;
static const unsigned int MAX_FEATURES = 400;

#ifdef USE_BYTE_SIGNATURES
typedef uint8_t sig_t;
#else
typedef float sig_t;
#endif
typedef Promote<sig_t>::type distance_t;

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
  //StarDetector detector(cvSize(640, 480), 7, 10.0);
  std::vector<Keypoint> pts;
  RTreeClassifier classifier(true);
  classifier.read(classifier_file);
  unsigned int dimension = classifier.classes();

  // Compute features and their descriptors for each object (image)
  std::vector<float*> train_buffers;
  std::vector<sig_t*> test_buffers;
  std::vector<size_t> buffer_sizes;
  std::vector<unsigned int> objs;
  // Save features for each image
  std::vector<voKeypoints> image_pts(num_objs);
  std::vector< std::vector<sig_t*> > image_features(num_objs);
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
    //pts = starKeypoints(left.Ipl(), 7, 10.0);
    //pts = siftKeypoints(left.Ipl());
    pts = fastKeypoints(left.Ipl(), 18, 40);
    //pts.resize(0);
    //detector.DetectPoints(left.Ipl(), std::back_inserter(pts));
    //KeepBestPoints(pts, MAX_FEATURES);

    // Compute descriptors, disparities
    float* train_buffer = Eigen::ei_aligned_malloc<float>(dimension * pts.size());
    train_buffers.push_back(train_buffer);
    buffer_sizes.push_back(pts.size());
    float* train_sig = train_buffer;
    sig_t* test_buffer;
    posix_memalign((void**)&test_buffer, 16, dimension * pts.size() * sizeof(sig_t));
    test_buffers.push_back(test_buffer);
    sig_t* test_sig = test_buffer;
    BOOST_FOREACH( const Keypoint& pt, pts ) {
      // Signature
      cv::WImageView1_b view = extractPatch(left.Ipl(), pt);
      classifier.getSignature(view.Ipl(), train_sig);
      classifier.getSignature(view.Ipl(), test_sig);
      objs.push_back(obj);

      // Disparity
      double d = frame.lookupDisparity(pt.x, pt.y);
      if (d > 0) {
        image_pts[obj].push_back( voKeypoint(pt.x, pt.y, d, pt.response, pt.scale, NULL) );
        image_features[obj].push_back(test_sig);
      }

      train_sig += dimension;
      test_sig += dimension;
    }
    ++obj;
  }
  size_t num_features = std::accumulate(buffer_sizes.begin(), buffer_sizes.end(), 0);  
  printf("%u features\n", num_features);

  // Copy into single Eigen matrix
  FeatureMatrix features((int)num_features, (int)dimension);
  size_t current_row = 0;
  for (unsigned int i = 0; i < train_buffers.size(); ++i) {
    features.block(current_row, 0, buffer_sizes[i], dimension) =
      Eigen::Map<FeatureMatrix>(train_buffers[i], buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
  }
  
  // Train vocabulary tree
  VocabularyTree tree;
  //tree.build(features, objs, 5, 4);
  
  //tree.build(features, objs, 5, 4, false);
  //tree.save("james4_empty.tree");
  //tree.load("james4_empty.tree");
  tree.load("/u/mihelich/images/holidays/holidays.tree");
  printf("Done loading tree\n");
#if 1
  printf("Adding images to tree dynamically...\n");
  current_row = 0;
  for (unsigned int i = 0; i < NUM_QUERIES; ++i) {
#ifdef USE_BYTE_SIGNATURES
    sig_t* feature_block = NULL;
    posix_memalign((void**)&feature_block, 16, buffer_sizes[i]*dimension*sizeof(sig_t));
    for (unsigned int j = 0; j < buffer_sizes[i]; ++j) {
      memcpy((void*)(feature_block + j*dimension), (void*)(test_buffers[i] + j*dimension),
             dimension*sizeof(sig_t));
    }
    current_row += buffer_sizes[i];
    tree.insert(feature_block, buffer_sizes[i]);
    free(feature_block);
#else
    FeatureMatrix feature_block = features.block(current_row, 0, buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    tree.insert(feature_block);
#endif
  }
#endif
  
  //tree.save("james4.tree");
  //tree.load("james4.tree");

  // Free training buffers
  for (unsigned int i = 0; i < train_buffers.size(); ++i) {
    free(train_buffers[i]);
  }
  
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
    // Construct query matrix
#ifdef USE_BYTE_SIGNATURES
    sig_t* query = NULL;
    posix_memalign((void**)&query, 16, buffer_sizes[i]*dimension*sizeof(sig_t));
    for (unsigned int j = 0; j < buffer_sizes[i]; ++j) {
      memcpy((void*)(query + j*dimension), (void*)(test_buffers[i] + j*dimension),
             dimension*sizeof(sig_t));
    }
#else
    FeatureMatrix query = features.block(current_row, 0, buffer_sizes[i], dimension);
#endif
    current_row += buffer_sizes[i];
    printf("Training image %u, %u features\n", i, buffer_sizes[i]);

    // Find top N matches
    matches.resize(0);
#ifdef USE_BYTE_SIGNATURES
    tree.find(query, buffer_sizes[i], N, std::back_inserter(matches));
    free(query);
#else
    tree.find(query, N, std::back_inserter(matches));
    //tree.findAndInsert(query, N, std::back_inserter(matches));
#endif
    if (matches[0].id == i) ++training_correct;

    // Set up matcher for signatures in query image
    BruteForceMatcher<sig_t, int> matcher(dimension);
    BOOST_FOREACH( sig_t* sig, image_features[i] )
      matcher.addSignature(sig, 0);
    
    // Geometric check (find number of inliers)
    cv::willow::PoseEstimateStereo pose_estimator;
    pose_estimator.setInlierErrorThreshold(3.0);
    pose_estimator.setNumRansacIterations(15);
    // Camera parameters for james4
    pose_estimator.setCameraParams(432.0, 432.0, .088981, 313.7821, 313.7821, 220.407);
    distance_t distance; // dummy
    for (unsigned int j = 0; j < std::min(N_show, matches.size()); ++j) {
      // Find matching keypoint pairs
      std::vector< std::pair<int, int> > match_index_pairs;
      unsigned int match_id = matches[j].id;
      for (int index2 = 0; index2 < (int)image_features[match_id].size(); ++index2) {
        sig_t* sig = image_features[match_id][index2];
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

  // Free buffers
  for (unsigned int i = 0; i < test_buffers.size(); ++i) {
    free(test_buffers[i]);
  }
    
  return 0;
}
