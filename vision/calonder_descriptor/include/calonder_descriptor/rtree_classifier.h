#ifndef FEATURES_RTREE_CLASSIFIER_H
#define FEATURES_RTREE_CLASSIFIER_H

#include "calonder_descriptor/randomized_tree.h"
#include "calonder_descriptor/signature.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_sparse.hpp>

namespace ublas = boost::numeric::ublas;

namespace features {

class RTreeClassifier
{
public:
  static const int DEFAULT_TREES = 16;
  
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

  // TODO: think about reducing memory allocations (return-by-reference versions)
  //DenseSignature getDenseSignature(cv::WImageView1_b const& patch) const;
  //SparseSignature getSparseSignature(cv::WImageView1_b const& patch) const;
  DenseSignature getDenseSignature(IplImage* patch) const;
  SparseSignature getSparseSignature(IplImage* patch) const;

  int classes() { return classes_; }

  float threshold() { return threshold_; }
  void setThreshold(float thres) { threshold_ = thres; }
  
  void read(const char* file_name);
  void read(std::istream &is);
  void write(const char* file_name) const;
  void write(std::ostream &os) const;

private:
  int classes_;
  float threshold_;
  std::vector<RandomizedTree> trees_;
};

} // namespace features

#endif
