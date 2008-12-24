#ifndef _VOCABULARY_TREE_H_
#define _VOCABULARY_TREE_H_

#include <vector>
#include <map>
#include <iterator>
#include <algorithm>
#include <string>
#include <Eigen/Core>
#include <boost/pool/object_pool.hpp>
#include <boost/noncopyable.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/tail.hpp>
#include <cmath>
#include <cstdio>
#include "place_recognition/kmeans.h" // TODO: only for FeatureMatrix typedef

namespace vision {

// TODO: template on feature dimension / matrix type?
//       good for speed, memory management...
// TODO: const correctness in API
class VocabularyTree : boost::noncopyable
{
public:
  
  struct Match
  {
    unsigned int id;
    float score;

    Match() : id(0), score(0.0f) {}
    Match(unsigned int id, float score) : id(id), score(score) {}

    bool operator<(const Match& other) const
    {
      return score < other.score;
    }
  };
  
  VocabularyTree();

  //! Construct a vocabulary tree using a set of training features. Each row of
  //! features is a training descriptor. objs[i] is the id of the training image
  //! from which features.row(i) was taken.
  //! Nister uses k = 10, levels = 6, but for smaller training sets these numbers
  //! should be reduced.
  //! If keep_training_images is true, the image database is populated with the
  //! training images; otherwise the database starts empty.
  void build( const FeatureMatrix& features,
              const std::vector<unsigned int>& objs, // a feature with ID id is from objs[id]
              unsigned int k,  unsigned int levels,
              bool keep_training_images = true);

  //! Clears inverted files and database vectors
  void clearDatabase();

  //! Find the top N database vectors matching the query
  template< typename OutputIterator >
  void find(const FeatureMatrix& query, unsigned int N, OutputIterator output);

  //! Save an image represented by its feature descriptors for future lookup
  unsigned int insert(const FeatureMatrix& image_features);

  // TODO: combined find + insertion for efficiency? findAndRetain & insertRetainedQuery?
  
  // File I/O
  void save(const std::string& file);
  void load(const std::string& file);
  
private:
  typedef std::map<unsigned int, unsigned int> InvertedFile;
  typedef std::back_insert_iterator< std::vector<unsigned int> > IdOutputIterator;
  
  // TODO: better to keep child centroids together in parent? (locality)
  // TODO: think more on child memory management
  struct Node
  {
    Node() : weight(0.0f) {}
    
    Eigen::VectorXf centroid;
    std::vector<Node*> children;
    InvertedFile inverted_file;
    float weight;
  };

  typedef std::map< Node*, float > ImageVector;

  void clearDatabaseAux(Node* node);
  
  void saveAux(Node* node, FILE* out, std::string indentation = "");

  void loadAux(Node* node, FILE* in, unsigned int indent_level = 0);

  void constructVocabulary(const FeatureMatrix& features,
                           const std::vector<unsigned int>& input);
  
  void constructVocabularyAux(const FeatureMatrix& features,
                              const std::vector<unsigned int>& input,
                              Node* node, unsigned int level);

  Node* findNearestChild(Node* node, const FeatureMatrix::RowXpr& feature);

  void addFeature(Node* node, unsigned int object_id,
                  const FeatureMatrix::RowXpr& feature);
  
  void addFeatureToQuery(Node* node, ImageVector& vec,
                         const FeatureMatrix::RowXpr& feature);

  void addFeatureToDatabaseVector(Node* node, unsigned int object_id,
                                  const FeatureMatrix::RowXpr& feature);

  float tfIdfWeight(size_t N_i);
  
  void recursiveTfIdfWeighting(Node* node, IdOutputIterator output);
  
  void assignWeights();

  void updateFiles(Node* node, InvertedFile& file, InvertedFile& parent_file);
  
  void calculateDatabaseVectorsAux(Node* node, InvertedFile& parent_file);

  void calculateDatabaseVectors();

  void normalizeL1(ImageVector& vec);

  float distanceL1(const ImageVector& v1, const ImageVector& v2);

  // TODO: problematic if multiple pools instantiated? weird I/O issue...
  Node* root_;
  boost::object_pool<Node> pool_; // for fast node allocation
  unsigned int k_; // branching factor
  unsigned int levels_; // max # levels
  unsigned int dim_; // descriptor dimension
  std::vector< ImageVector > db_vectors_; // precomputed database vectors
};


inline VocabularyTree::VocabularyTree()
  : root_(NULL), k_(0), levels_(0), dim_(0)
{}

template< typename OutputIterator >
void VocabularyTree::find(const FeatureMatrix& query, unsigned int N,
                          OutputIterator output)
{
  ImageVector query_vec;

  for (int i = 0; i < query.rows(); ++i)
    addFeatureToQuery(root_, query_vec, query.row(i));
  normalizeL1(query_vec);
  printf("Size of query_vec: %u\n", query_vec.size());

  // accumulate the best N matches
  using namespace boost::accumulators;
  typedef tag::tail<left> bestN_tag;
  accumulator_set<Match, features<bestN_tag> > acc(bestN_tag::cache_size = N);

  // TODO: only compute distances against objects from inverted files
  for (unsigned int i = 0; i < db_vectors_.size(); ++i) {
    float distance = distanceL1(query_vec, db_vectors_[i]);
    acc( Match(i, distance) );
  }

  extractor<bestN_tag> bestN;
  std::copy(bestN(acc).begin(), bestN(acc).end(), output);
}

inline float VocabularyTree::tfIdfWeight(size_t N_i)
{
  return log( (float)db_vectors_.size() / (float)N_i );
}

inline void VocabularyTree::normalizeL1(ImageVector& vec)
{
  float sum = 0.0f;
  for (ImageVector::iterator i = vec.begin(), ie = vec.end();
       i != ie; ++i)
    sum += i->second;

  float inv_sum = 1.0f / sum;
  for (ImageVector::iterator i = vec.begin(), ie = vec.end();
       i != ie; ++i)
    i->second *= inv_sum;
}

// TODO: possible to optimize further?
inline float VocabularyTree::distanceL1(const ImageVector& v1,
                                        const ImageVector& v2)
{
  float distance = 0.0f;
  ImageVector::const_iterator i1 = v1.begin(), i1e = v1.end();
  ImageVector::const_iterator i2 = v2.begin(), i2e = v2.end();

  while (i1 != i1e && i2 != i2e) {
    if (i2->first < i1->first) {
      distance += i2->second;
      ++i2;
    }
    else if (i1->first < i2->first) {
      distance += i1->second;
      ++i1;
    }
    else {
      distance += fabs(i1->second - i2->second);
      ++i1; ++i2;
    }
  }

  while (i1 != i1e) {
    distance += i1->second;
    ++i1;
  }

  while (i2 != i2e) {
    distance += i2->second;
    ++i2;
  }
  
  return distance;
}

} // namespace vision

#endif
