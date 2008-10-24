#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"
#include <fstream>
#include <boost/foreach.hpp>

namespace features {

RTreeClassifier::RTreeClassifier()
  : classes_(0), threshold_(0)//, element_threshold_(0)
{}

void RTreeClassifier::train(std::vector<BaseKeypoint> const& base_set,
                            Rng &rng, int num_trees, int depth,
                            int views)
{
  PatchGenerator make_patch(NULL, rng);
  train(base_set, rng, make_patch, num_trees, depth, views);
}

// Single-threaded version of train(), with progress output
void RTreeClassifier::train(std::vector<BaseKeypoint> const& base_set,
                            Rng &rng, PatchGenerator &make_patch,
                            int num_trees, int depth, int views)
{
  classes_ = base_set.size();
  trees_.resize(num_trees);
  
  int count = 1;
  printf("Trained 0 / %i trees", num_trees);
  fflush(stdout);
  BOOST_FOREACH( RandomizedTree &tree, trees_ ) {
    tree.train(base_set, rng, make_patch, depth, views);
    printf("\rTrained %i / %i trees", count++, num_trees);
    fflush(stdout);
  }
  printf("\n");
}

// TODO: trivially vectorizable
//DenseSignature RTreeClassifier::getDenseSignature(cv::WImageView1_b const& patch) const
DenseSignature RTreeClassifier::getDenseSignature(IplImage* patch) const
{
  DenseSignature sig = ublas::zero_vector<float>(classes_);

  std::vector<RandomizedTree>::const_iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it) {
    const float* post_array = tree_it->getPosterior(patch);
    // Cram float* into uBLAS-friendly type without copying
    typedef const ublas::array_adaptor<float> PostStorage;
    typedef const ublas::vector<float, PostStorage> PostVec;
    PostVec post(classes_, PostStorage(classes_, const_cast<float*>(post_array)) );
    sig += post;
  }

  // TODO: get rid of this multiply
  sig *= (1.0 / trees_.size());
  //sig /= trees_.size();

  return sig;
}

//SparseSignature RTreeClassifier::getSparseSignature(cv::WImageView1_b const& patch) const
SparseSignature RTreeClassifier::getSparseSignature(IplImage* patch) const
{
  DenseSignature dense_sig = getDenseSignature(patch);
  SparseSignature sparse_sig(classes_, 0);

  for (int i = 0; i < classes_; ++i) {
    float elem = dense_sig[i];
    //if (elem > element_threshold_)
    if (elem > threshold_)
      sparse_sig.insert_element(i, elem);
  }
  
  return sparse_sig;
}

void RTreeClassifier::read(const char* file_name)
{
  std::ifstream file(file_name, std::ifstream::binary);
  read(file);
}

void RTreeClassifier::read(std::istream &is)
{
  int num_trees = 0;
  is.read((char*)(&num_trees), sizeof(num_trees));
  is.read((char*)(&classes_), sizeof(classes_));
  is.read((char*)(&threshold_), sizeof(threshold_));

  trees_.resize(num_trees);
  std::vector<RandomizedTree>::iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it) {
    tree_it->read(is);
  }
}

void RTreeClassifier::write(const char* file_name) const
{
  std::ofstream file(file_name, std::ofstream::binary);
  write(file);
}

void RTreeClassifier::write(std::ostream &os) const
{
  int num_trees = trees_.size();
  os.write((char*)(&num_trees), sizeof(num_trees));
  os.write((char*)(&classes_), sizeof(classes_));
  os.write((char*)(&threshold_), sizeof(threshold_));

  std::vector<RandomizedTree>::const_iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it)
    tree_it->write(os);
}

} // namespace features
