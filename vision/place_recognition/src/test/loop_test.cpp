#include "place_recognition/vocabulary_tree.h"
#include <numeric>
#include <sstream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <star_detector/detector.h>
#include <calonder_descriptor/rtree_classifier.h>
#include <cvwimage.h>
#include <highgui.h>

using boost::format;
using namespace vision;
using namespace features;

static const char image_format[] = "/u/prdata/videre-bags/james4/im.%06u.left_rectified.tiff";
static const char classifier_file[] = "/u/mihelich/ros/ros-pkg/vision/calonder_descriptor/src/test/land50_cs.trees";
static const unsigned int NUM_QUERIES = 78;

int main(int argc, char** argv)
{
  std::vector< std::string > query_files;
  //std::vector< std::vector<std::string> > corresponding_files;
  //std::vector<int> query_index;
  int num_objs = NUM_QUERIES;
  //int num_queries = 0;

  query_files.reserve(NUM_QUERIES);
  //corresponding_files.resize(NUM_QUERIES);

  // List query files and their corresponding files
  for (unsigned int i = 0; i < NUM_QUERIES; ++i) {
    std::stringstream ss;
    ss << format(image_format) % (i * 30);
    query_files.push_back(ss.str());
  }
  //printf("%d database images\n", num_objs);
  //printf("%d query images\n", num_queries);

  // Compute features and their descriptors for each object (image)
  std::vector<float*> buffers;
  std::vector<size_t> buffer_sizes;
  std::vector<unsigned int> objs;
  int obj = 0;
  StarDetector detector(cvSize(640, 480), 7, 10.0);
  std::vector<Keypoint> pts;
  RTreeClassifier classifier(true);
  classifier.read(classifier_file);
  unsigned int dimension = classifier.classes();
  BOOST_FOREACH( std::string& file, query_files ) {
    cv::WImageBuffer1_b image( cvLoadImage(file.c_str(), CV_LOAD_IMAGE_GRAYSCALE) );
    pts.resize(0);
    detector.DetectPoints(image.Ipl(), std::back_inserter(pts));
    float* sig_buffer = NULL;
    posix_memalign(reinterpret_cast<void**>(&sig_buffer), 16,
                   dimension * sizeof(float) * pts.size());
    buffers.push_back(sig_buffer);
    buffer_sizes.push_back(pts.size());
    float* sig = sig_buffer;
    BOOST_FOREACH( const Keypoint& pt, pts ) {
      cv::WImageView1_b view = extractPatch(image.Ipl(), pt);
      classifier.getFloatSignature(view.Ipl(), sig);
      objs.push_back(obj);
      sig += dimension;
    }
    ++obj;
  }

  // Copy into single Eigen matrix
  size_t num_features = std::accumulate(buffer_sizes.begin(), buffer_sizes.end(), 0);
  FeatureMatrix pfeatures((int)num_features, (int)dimension);
  size_t current_row = 0;
  for (unsigned int i = 0; i < buffers.size(); ++i) {
    pfeatures.block(current_row, 0, buffer_sizes[i], dimension) =
      Eigen::Map<FeatureMatrix>(buffers[i], buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    free(buffers[i]);
  }
  buffers.clear();
  
  printf("%u features\n", num_features);
  
  // Train vocabulary tree
  VocabularyTree tree;
  tree.build(pfeatures, objs, 5, 5);
  tree.save("james4.tree");
  
  // Validation
  int training_correct = 0, query_correct = 0;
  unsigned int N = NUM_QUERIES;
  unsigned int N_show = 5;
  std::vector<VocabularyTree::Match> matches;
  matches.reserve(N);
  // Confusion matrix
  FeatureMatrix confusion((int)NUM_QUERIES, (int)NUM_QUERIES);
  
  // Validate on training images
  current_row = 0;
  for (unsigned int i = 0; i < NUM_QUERIES; ++i) {
    FeatureMatrix query = pfeatures.block(current_row, 0, buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    matches.resize(0);
    tree.find(query, N, std::back_inserter(matches));
    if (matches[0].id == i) ++training_correct;
    printf("Training image %u, %u features\n", i, query.rows());
    for (unsigned int j = 0; j < N_show; ++j)
      printf("\tid = %u, score = %f\n", matches[j].id, matches[j].score);
    for (unsigned int j = 0; j < N; ++j)
      confusion(i, matches[j].id) = 1.0f - matches[j].score * 0.5f;
  }

  // Write out confusion matrix
  std::ofstream confusion_file("conf.mat");
  for (unsigned i = 0; i < NUM_QUERIES; ++i) {
    for (unsigned j = 0; j < NUM_QUERIES; ++j) {
      confusion_file << confusion(i, j) << " ";
    }
    confusion_file << std::endl;
  }

  // Read descriptors for each query image
  /*
  std::vector< std::vector< Descriptor > > qr_features(num_queries);
  obj = 0;
  BOOST_FOREACH(std::string& file, query_files) {
    readSiftGeoDescriptors(file.c_str(), qr_features[obj]);
    ++obj;
  }
  
  // Try query images
  query_num = 0;
  printf("\n--------------\n\n");
  BOOST_FOREACH(std::vector< Descriptor >& query, qr_features) {
    tree.query(query, N, std::back_inserter(matches));
    if (query_index[matches[0].id] == query_num) ++query_correct;
    printf("Query image %u, %u features\n", query_num++, query.size());
    BOOST_FOREACH( const vocab_tree::match& m, matches )
      printf("\tid = %u corresponding to query %u, score = %f\n",
             m.id, query_index[m.id], m.score);
    matches.resize(0);
  }

  printf("\nTraining set: %u / %u\nQuery set: %u / %u\n", training_correct,
         num_objs, query_correct, num_queries);
  */

  printf("\nTraining set: %u / %u\n", training_correct, num_objs);
    
  return 0;
}
