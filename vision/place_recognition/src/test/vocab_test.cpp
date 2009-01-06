#include "io.h"
#include "place_recognition/vocabulary_tree.h"
#include <string>
#include <cstdlib>
#include <boost/foreach.hpp>

using namespace vision;

static const char des_directory[] = "/u/mihelich/images/holidays/siftgeo/";
static const unsigned int NUM_QUERIES = 5;

int main(int argc, char** argv)
{
  std::vector< std::string > all_files, query_files;
  std::vector< std::vector<std::string> > corresponding_files;
  std::vector<int> query_index;
  int num_objs = 0;
  int num_queries = 0;

  dir(des_directory, all_files);
  query_files.reserve(NUM_QUERIES);
  corresponding_files.resize(NUM_QUERIES);

  // List query files and their corresponding files
  char last_digit = ' ';
  BOOST_FOREACH(const std::string& file, all_files) {
    char digit = file[3];
    if (digit != last_digit) {
      if (query_files.size() == NUM_QUERIES)
        break;
      last_digit = digit;
      query_files.push_back(des_directory + file);
      ++num_queries;
    }
    else {
      corresponding_files[query_files.size()-1].push_back(des_directory + file);
      query_index.push_back(num_queries - 1);
      ++num_objs;
    }
  }
  printf("%d database images\n", num_objs);
  printf("%d query images\n", num_queries);

  // Precompute number of features and allocate feature matrix appropriately
  unsigned int num_features = 0;
  BOOST_FOREACH(std::vector<std::string>& files, corresponding_files) {
    BOOST_FOREACH(std::string& file, files)
      num_features += countSiftGeo(file.c_str());
  }
  FeatureMatrix features(num_features, 128);
  
  // Read descriptors for each object (image)
  std::vector<unsigned int> objs;
  std::vector<unsigned int> feature_counts;
  int current_row = 0;
  int obj = 0;
  BOOST_FOREACH(std::vector<std::string>& files, corresponding_files) {
    BOOST_FOREACH(std::string& file, files) {
      unsigned int count = countSiftGeo(file.c_str());
      Eigen::BlockReturnType<FeatureMatrix>::Type block = features.block(current_row, 0, count, 128);
      readSiftGeoDescriptors(file.c_str(), block);
      current_row += count;
      feature_counts.push_back(count);
      objs.insert(objs.end(), count, obj++);
    }
  }
  printf("%u features\n", features.rows());
  
  // Train vocabulary tree
  VocabularyTree tree;
  tree.build(features, objs, 5, 4);
  //tree.build(features, objs, 5, 4, false);
  printf("Saving...\n");
  tree.save("test.tree");
    
  // Validation
  int training_correct = 0, query_correct = 0;
  unsigned int N = 5;
  std::vector< VocabularyTree::Match > matches;
  matches.reserve(N);
  
  // Validate on training images
  current_row = 0;
  for (int i = 0; i < num_objs; ++i) {
    FeatureMatrix query = features.block(current_row, 0, feature_counts[i], 128);
    current_row += feature_counts[i];
    matches.resize(0);
    tree.find(query, N, std::back_inserter(matches));
    if (matches[0].id == i) ++training_correct;
    printf("Training image %u, %u features\n", i, query.rows());
    BOOST_FOREACH( const VocabularyTree::Match& m, matches )
      printf("\tid = %u, score = %f\n", m.id, m.score);
  }
    
  // Try query images
  unsigned int query_num = 0;
  printf("\n--------------\n\n");
  BOOST_FOREACH(std::string& file, query_files) {
    // Read descriptors from file
    unsigned int count = countSiftGeo(file.c_str());
    FeatureMatrix query(count, 128);
    readSiftGeoDescriptors(file.c_str(), query);

    matches.resize(0);
    tree.find(query, N, std::back_inserter(matches));
    if (query_index[matches[0].id] == query_num) ++query_correct;
    printf("Query image %u, %u features\n", query_num++, query.rows());
    BOOST_FOREACH( const VocabularyTree::Match& m, matches )
      printf("\tid = %u corresponding to query %u, score = %f\n",
             m.id, query_index[m.id], m.score);
  }

  printf("\nTraining set: %u / %u\nQuery set: %u / %u\n", training_correct,
         num_objs, query_correct, num_queries);
  
  return 0;
}
