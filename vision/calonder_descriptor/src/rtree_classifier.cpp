#define BOOST_UBLAS_SHALLOW_ARRAY_ADAPTOR
#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/random.hpp>

namespace features {


RTreeClassifier::RTreeClassifier()
  : classes_(0), threshold_(0)
{
   //setReducedDim(DEFAULT_RED);
}

void RTreeClassifier::train(std::vector<BaseKeypoint> const& base_set,
                            Rng &rng, int num_trees, int depth,
                            int views, size_t reduced_num_dim)
{
  PatchGenerator make_patch(NULL, rng);
  train(base_set, rng, make_patch, num_trees, depth, views, reduced_num_dim);
}

// Single-threaded version of train(), with progress output
void RTreeClassifier::train(std::vector<BaseKeypoint> const& base_set,
                            Rng &rng, PatchGenerator &make_patch, int num_trees,
                            int depth, int views, size_t reduced_num_dim)
{
  if (reduced_num_dim > base_set.size()) {
    printf("INVALID PARAMS in RTreeClassifier::train: reduced_num_dim > base_set.size()\n");
    return;
  }
  classes_ = reduced_num_dim; // base_set.size();
  original_num_classes_ = base_set.size();
  trees_.resize(num_trees);
  
  printf("[OK] Training trees: base size=%i, reduced size=%i\n", base_set.size(), reduced_num_dim); 
  
  int count = 1;
  printf("[OK] Trained 0 / %i trees\r", num_trees);
  fflush(stdout);
  BOOST_FOREACH( RandomizedTree &tree, trees_ ) {
    tree.train(base_set, rng, make_patch, depth, views, reduced_num_dim);    
    printf("[OK] Trained %i / %i trees\r", count++, num_trees);
    fflush(stdout);
  }
  printf("\n");
}

// TODO: trivially vectorizable
DenseSignature RTreeClassifier::getSignature(IplImage* patch)
{
  // used inside loop to cram float* into uBLAS-friendly type without copying
  typedef const ublas::shallow_array_adaptor<float> PostStorage;
  typedef const ublas::vector<float, PostStorage> PostVec;

  DenseSignature sig = ublas::zero_vector<float>(classes_);

  std::vector<RandomizedTree>::const_iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it) {
    PostVec post(classes_, PostStorage(classes_, const_cast<float*>(tree_it->getPosterior(patch))) );
    sig += post;
  }

  // TODO: get rid of this multiply
  sig *= (1.0 / trees_.size());

  return sig;
}

SparseSignature RTreeClassifier::getSparseSignature(IplImage* patch) const
{
  printf("ERROR [RTreeClassifier::getSparseSignature()]: Framework does not support sparse signatures any longer.\n");
  return SparseSignature(classes_, 0);
  
  /*
  DenseSignature dense_sig = getDenseSignature(patch);
  SparseSignature sparse_sig(classes_, 0);

  for (int i = 0; i < classes_; ++i) {
    float elem = dense_sig[i];
    //if (elem > element_threshold_)
    if (elem > threshold_)
      sparse_sig.insert_element(i, elem);
  }
  
  return sparse_sig;
  */
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
  is.read((char*)(&original_num_classes_), sizeof(original_num_classes_));  
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
  os.write((char*)(&original_num_classes_), sizeof(original_num_classes_));
  os.write((char*)(&threshold_), sizeof(threshold_));

  std::vector<RandomizedTree>::const_iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it)
    tree_it->write(os);
}


/*DenseSignature RTreeClassifier::getCompressedSignature(IplImage* patch, bool from_sparse)
{ 
   DenseSignature sig(cs_phi_.size1());
   if (cs_phi_.size1()==0 || cs_phi_.size2()==0) {
      printf("Error: Must call RTreeClassifier::setReducedDim() prior to RTreeClassifier::getCompressedSignature().\n");
      return sig;
   }   
   
   if (from_sparse) {
      SparseSignature sparse = getSparseSignature(patch);
      ublas::axpy_prod(cs_phi_, sparse, sig, true);   
   }
   else {
      DenseSignature dense = getDenseSignature(patch);
      ublas::axpy_prod(cs_phi_, dense, sig, true);
   }
   return sig;
}*/


} // namespace features
