#ifndef FEATURES_RTREE_CLASSIFIER_H
#define FEATURES_RTREE_CLASSIFIER_H

#include "calonder_descriptor/randomized_tree.h"
#include "calonder_descriptor/signature.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>

namespace ublas = boost::numeric::ublas;

namespace features {

class RTreeClassifier
{
public:
  static const int DEFAULT_TREES = 50;
  typedef enum { PHI_GAUSS=1, PHI_BERNOULLI, PHI_DBFRIENDLY } PHI_DISTR_TYPE;   // used in makeRandomMeasMatrix
  
  RTreeClassifier();

  void train(std::vector<BaseKeypoint> const& base_set, Rng &rng,
             int num_trees = RTreeClassifier::DEFAULT_TREES,
             int depth = RandomizedTree::DEFAULT_DEPTH,
             int views = RandomizedTree::DEFAULT_VIEWS);
  void train(std::vector<BaseKeypoint> const& base_set,
             Rng &rng, PatchGenerator &make_patch,
             int num_trees = RTreeClassifier::DEFAULT_TREES,
             int depth = RandomizedTree::DEFAULT_DEPTH,
             int views = RandomizedTree::DEFAULT_VIEWS);

  DenseSignature getDenseSignature(IplImage* patch) const;
  DenseSignature getCompressedSignature(IplImage* patch, bool from_sparse);
  SparseSignature getSparseSignature(IplImage* patch) const;

  int classes() { return classes_; }

  float threshold() { return threshold_; }
  void setThreshold(float thres) { threshold_ = thres; }
  // TODO: make threshold arg independent of the number of classes and trees.
  //float threshold();
  //void setThreshold(float thres);
  
  void makeRandomMeasMatrix(size_t m, PHI_DISTR_TYPE distr_type=PHI_GAUSS);
  void setMeasMatrix(std::string filename, size_t m);
  
  void read(const char* file_name);
  void read(std::istream &is);
  void write(const char* file_name) const;
  void write(std::ostream &os) const;

private:
  int classes_;
  float threshold_;
  //float element_threshold_;
  std::vector<RandomizedTree> trees_;
  ublas::matrix<float> cs_phi_;     // measurement matrix phi for compressive sensing
};

/*
inline float RTreeClassifier::threshold()
{
  return threshold_;
}

inline void RTreeClassifier::setThreshold(float thres)
{
  threshold_ = thres;
  element_threshold_ = (thres / classes_) * trees_.size();
}
*/

} // namespace features

#endif
