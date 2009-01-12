#include "place_recognition/vocabulary_tree.h"
#include "timer.h"
#include <numeric>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cstdio>
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
  int num_objs = NUM_QUERIES;
  double total_insertion_time = 0, wallclock_insertion_time = 0;
  double total_query_time = 0, wallclock_query_time = 0;
  FILE* out;

  query_files.reserve(NUM_QUERIES);

  // List query files and their corresponding files
  for (unsigned int i = 0; i < NUM_QUERIES; ++i) {
    std::stringstream ss;
    ss << format(image_format) % (i * 30);
    query_files.push_back(ss.str());
  }
  printf("%d database images\n", num_objs);

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
    // Find keypoints
    cv::WImageBuffer1_b image( cvLoadImage(file.c_str(), CV_LOAD_IMAGE_GRAYSCALE) );
    pts.resize(0);
    detector.DetectPoints(image.Ipl(), std::back_inserter(pts));
    //KeepBestPoints(pts, 400);

    // Calculate descriptors
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
  if (argc < 2) {
    tree.build(pfeatures, objs, 5, 5);
    //tree.build(pfeatures, objs, 5, 5, false);
    printf("Saving to james4.tree...\n");
    tree.save("james4.tree");
  }
  else {
    tree.load(argv[1]);
    current_row = 0;
    out = fopen("insertion.out", "w");
    Timer wallclock_time;
    for (int i = 0; i < num_objs; ++i) {
      FeatureMatrix obj_features = pfeatures.block(current_row, 0, buffer_sizes[i], dimension);
      current_row += buffer_sizes[i];
      Timer t;
      tree.insert(obj_features);
      double elapsed = t.elapsed();
      total_insertion_time += elapsed;
      fprintf(out, "%f %u\n", elapsed, buffer_sizes[i]);
    }
    wallclock_insertion_time = wallclock_time.elapsed();
    fclose(out);
  }
  
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
  out = fopen("train_query.out", "w");
  Timer query_timer;
  for (unsigned int i = 0; i < NUM_QUERIES; ++i) {
    FeatureMatrix query = pfeatures.block(current_row, 0, buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    matches.resize(0);
    Timer t;
    tree.find(query, N, std::back_inserter(matches));
    double elapsed = t.elapsed();
    total_query_time += elapsed;
    fprintf(out, "%f %u\n", elapsed, buffer_sizes[i]);
    if (matches[0].id == i) ++training_correct;
    printf("Training image %u, %u features\n", i, query.rows());
    for (unsigned int j = 0; j < N_show; ++j)
      printf("\tid = %u, score = %f\n", matches[j].id, matches[j].score);
    for (unsigned int j = 0; j < N; ++j)
      confusion(i, matches[j].id) = 1.0f - matches[j].score * 0.5f;
  }
  wallclock_query_time = query_timer.elapsed();
  fclose(out);

  // Write out confusion matrix
  std::ofstream confusion_file("conf.mat");
  for (unsigned i = 0; i < NUM_QUERIES; ++i) {
    for (unsigned j = 0; j < NUM_QUERIES; ++j) {
      confusion_file << confusion(i, j) << " ";
    }
    confusion_file << std::endl;
  }

  printf("\nTraining set: %u / %u\n", training_correct, num_objs);
  printf("Wallclock time for insertion: %fs\n", wallclock_insertion_time);
  printf("Inserted %u objects in total time %fs, avg. %fs\n",
         num_objs, total_insertion_time, total_insertion_time / num_objs);
  printf("Wallclock time for queries: %fs\n", wallclock_query_time);
  printf("Queried %u objects in total time %fs, avg. %fs\n",
         num_objs, total_query_time, total_query_time / num_objs);
    
  return 0;
}
