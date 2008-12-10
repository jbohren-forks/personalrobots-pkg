#ifndef FEATURES_RTREE_CLASSIFIER_H
#define FEATURES_RTREE_CLASSIFIER_H

#include "calonder_descriptor/randomized_tree.h"
#include "calonder_descriptor/basic_math.h"

namespace features {

class RTreeClassifier
{
public:
  static const int DEFAULT_TREES = 50;
    
  RTreeClassifier(bool keep_floats);

  void train(std::vector<BaseKeypoint> const& base_set, Rng &rng,
             int num_trees = RTreeClassifier::DEFAULT_TREES,
             int depth = RandomizedTree::DEFAULT_DEPTH,
             int views = RandomizedTree::DEFAULT_VIEWS,
             size_t reduced_num_dim = RandomizedTree::DEFAULT_REDUCED_NUM_DIM,
             int num_quant_bits=0);
  void train(std::vector<BaseKeypoint> const& base_set,
             Rng &rng, PatchGenerator &make_patch,
             int num_trees = RTreeClassifier::DEFAULT_TREES,
             int depth = RandomizedTree::DEFAULT_DEPTH,
             int views = RandomizedTree::DEFAULT_VIEWS,
             size_t reduced_num_dim = RandomizedTree::DEFAULT_REDUCED_NUM_DIM,
             int num_quant_bits=0);

  // Caller is responsible for calling free() on returned signature
  //float* getSignature(IplImage* patch);
  
  // sig must point to a memory block of at least classes()*sizeof(float|uchar) bytes
  void getSignature(IplImage *patch, ushort *sig);
  void getFloatSignature(IplImage *patch, float *sig);  
  void getSparseSignature(IplImage *patch, float *sig);
    
  inline int classes() { return classes_; }
  inline int original_num_classes() { return original_num_classes_; }
  
  void read(const char* file_name);
  void read(std::istream &is);
  void write(const char* file_name) const;
  void write(std::ostream &os) const;

private:  
  int classes_;
  int original_num_classes_;
  std::vector<RandomizedTree> trees_;
  bool keep_floats_;
};

} // namespace features

#endif
