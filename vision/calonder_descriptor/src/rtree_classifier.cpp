#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"
#include <fstream>
#include <cstring>
#include <boost/foreach.hpp>

namespace features {


RTreeClassifier::RTreeClassifier(bool keep_floats)
  : classes_(0), keep_floats_(keep_floats)
{
  posteriors_ = NULL;
   //setReducedDim(DEFAULT_RED);
}

void RTreeClassifier::train(std::vector<BaseKeypoint> const& base_set,
                            Rng &rng, int num_trees, int depth,
                            int views, size_t reduced_num_dim,
                            int num_quant_bits)
{
  PatchGenerator make_patch(NULL, rng);
  train(base_set, rng, make_patch, num_trees, depth, views, reduced_num_dim, num_quant_bits);
}

// Single-threaded version of train(), with progress output
void RTreeClassifier::train(std::vector<BaseKeypoint> const& base_set,
                            Rng &rng, PatchGenerator &make_patch, int num_trees,
                            int depth, int views, size_t reduced_num_dim, 
                            int num_quant_bits)
{
  if (reduced_num_dim > base_set.size()) {
    printf("INVALID PARAMS in RTreeClassifier::train: reduced_num_dim > base_set.size()\n");
    return;
  }
  if (!keep_floats_) {
    printf("WARNING: Cannot release FLOAT posteriors when training, otherwise saving will\n"
           "         not be possible afterwards. Setting keep_floats_ <- TRUE.\n"
           "         Releasing float posteriors is possible when you load the classifier\n"
           "         from the disc.\n");
    keep_floats_ = true;
  }
  
  classes_ = reduced_num_dim; // base_set.size();
  original_num_classes_ = base_set.size();
  trees_.resize(num_trees);  
  
  printf("[OK] Training trees: base size=%i, reduced size=%i\n", base_set.size(), reduced_num_dim); 
  
  int count = 1;
  printf("[OK] Trained 0 / %i trees", num_trees);
  fflush(stdout);
  BOOST_FOREACH( RandomizedTree &tree, trees_ ) {
    tree.setKeepFloatPosteriors(keep_floats_);
    tree.train(base_set, rng, make_patch, depth, views, reduced_num_dim, num_quant_bits);
    printf("\r[OK] Trained %i / %i trees", count++, num_trees);
    fflush(stdout);
  }
  printf("\n\n");  
}

// TODO: vectorize
void RTreeClassifier::getSignature(IplImage* patch, float *sig)
{ 
  // Need pointer to 32x32 patch data
  uchar buffer[RandomizedTree::PATCH_SIZE * RandomizedTree::PATCH_SIZE];
  uchar* patch_data;
  if (patch->widthStep != RandomizedTree::PATCH_SIZE) {
    //printf("[INFO] patch is padded, data will be copied (%i/%i).\n", 
    //       patch->widthStep, RandomizedTree::PATCH_SIZE);
    uchar* data = getData(patch);
    patch_data = buffer;
    for (int i = 0; i < RandomizedTree::PATCH_SIZE; ++i) {
      memcpy((void*)patch_data, (void*)data, RandomizedTree::PATCH_SIZE);
      data += patch->widthStep;
      patch_data += RandomizedTree::PATCH_SIZE;
    }
    patch_data = buffer;
  } 
  else {
    patch_data = getData(patch);
  }
    
  memset((void*)sig, 0, classes_ * sizeof(float));
  std::vector<RandomizedTree>::iterator tree_it;
 
  // get posteriors
  float **posteriors = new float*[trees_.size()];  // TODO: move alloc outside this func
  float **pp = posteriors;    
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it, pp++)
    *pp = tree_it->getPosterior(patch_data);       

  // sum them up
  pp = posteriors;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it, pp++)
    add(classes_, sig, *pp, sig);

  delete [] posteriors;
  posteriors = NULL;
      
  // full quantization (experimental)
  #if 0
    int n_max = 1<<8 - 1;
    int sum_max = (1<<4 - 1)*trees_.size();
    int shift = 0;    
    while ((sum_max>>shift) > n_max) shift++;
    
    for (int i = 0; i < classes_; ++i) {
      sig[i] = int(sig[i] + .5) >> shift;
      if (sig[i]>n_max) sig[i] = n_max;
    }

    static bool warned = false;
    if (!warned) {
      printf("[WARNING] Using full quantization (RTreeClassifier::getSignature)! shift=%i\n", shift);
      warned = true;
    }
  #else
    // TODO: get rid of this multiply (-> number of trees is known at train 
    // time, exploit it in RandomizedTree::finalize())
    float normalizer = 1.0f / trees_.size();
    for (int i = 0; i < classes_; ++i)
      sig[i] *= normalizer;
  #endif
}


// sum up 50 byte vectors of length 176
// assume 5 bits max for input vector values
// final shift is 3 bits right
void sum_50c_176c(uint8_t **pp, uint8_t *sig)
{
  
}

void RTreeClassifier::getSignature(IplImage* patch, uint8_t *sig)
{  
  // Need pointer to 32x32 patch data
  uchar buffer[RandomizedTree::PATCH_SIZE * RandomizedTree::PATCH_SIZE];
  uchar* patch_data;
  if (patch->widthStep != RandomizedTree::PATCH_SIZE) {
    //printf("[INFO] patch is padded, data will be copied (%i/%i).\n", 
    //       patch->widthStep, RandomizedTree::PATCH_SIZE);
    uchar* data = getData(patch);
    patch_data = buffer;
    for (int i = 0; i < RandomizedTree::PATCH_SIZE; ++i) {
      memcpy((void*)patch_data, (void*)data, RandomizedTree::PATCH_SIZE);
      data += patch->widthStep;
      patch_data += RandomizedTree::PATCH_SIZE;
    }
    patch_data = buffer;
  } else {
    patch_data = getData(patch);
  }
    
  std::vector<RandomizedTree>::iterator tree_it;
 
  // get posteriors
  if (posteriors_ == NULL)
    {
      posteriors_ = new uint8_t*[trees_.size()];  
      posix_memalign((void **)&ptemp_, 16, classes_*sizeof(uint16_t));
    }
  uint8_t **pp = posteriors_;    
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it, pp++)
    *pp = tree_it->getPosterior2(patch_data);       
  pp = posteriors_;

#if 1
  // sum them up  
  sum_50t_176c(pp,sig,ptemp_);

#else
  memset((void*)sig, 0, classes_ * sizeof(sig[0]));
  uint16_t *sig16 = new uint16_t[classes_];           // TODO: make member, no alloc here
  memset((void*)sig16, 0, classes_ * sizeof(sig16[0]));
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it, pp++)
    add(classes_, sig16, *pp, sig16);

  // full quantization (experimental, later implicit)
  // find out the required right-shift needed to fit all sig values into an uint8_t
  int n_max = (1 << (8*sizeof(uint8_t))) - 1;
  int sum_max = ((1<<4) - 1)*trees_.size();
  int shift = 0;    
  while ((sum_max>>shift) > n_max) shift++;
  
  for (int i = 0; i < classes_; ++i)
    sig[i] = sig16[i] >> shift;

  static bool warned = false;
  if (!warned) {
    printf("[NOTE] RTC: signature quantization, shift=%i\n", shift);
    warned = true;
  }
#endif
}


void RTreeClassifier::getSparseSignature(IplImage *patch, float *sig, float thresh)
{   
   getFloatSignature(patch, sig);
   for (int i=0; i<classes_; ++i, sig++)
      if (*sig < thresh) *sig = 0.f;
}

int RTreeClassifier::countNonZeroElements(float *vec, int n, double tol)
{
   int res = 0;
   while (n-- > 0)
      res += (fabs(*vec++) > tol);
   return res;
}

void RTreeClassifier::read(const char* file_name)
{
  std::ifstream file(file_name, std::ifstream::binary);
  read(file);
  file.close();
}

void RTreeClassifier::read(std::istream &is)
{
  int num_trees = 0;
  is.read((char*)(&num_trees), sizeof(num_trees));
  is.read((char*)(&classes_), sizeof(classes_));
  is.read((char*)(&original_num_classes_), sizeof(original_num_classes_));

  trees_.resize(num_trees);
  std::vector<RandomizedTree>::iterator tree_it;

  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it) {
    tree_it->setKeepFloatPosteriors(keep_floats_);
    tree_it->read(is);
  }
}

void RTreeClassifier::write(const char* file_name) const
{
  std::ofstream file(file_name, std::ofstream::binary);
  write(file);  
  file.close();
}

void RTreeClassifier::write(std::ostream &os) const
{
  int num_trees = trees_.size();
  os.write((char*)(&num_trees), sizeof(num_trees));
  os.write((char*)(&classes_), sizeof(classes_));
  os.write((char*)(&original_num_classes_), sizeof(original_num_classes_));

  std::vector<RandomizedTree>::const_iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it)
    tree_it->write(os);
}

void RTreeClassifier::saveAllPosteriors(std::string url)
{  
  for (int i=0; i<(int)trees_.size(); ++i)
    trees_[i].savePosteriors(url, (i==0 ? false : true));
  printf("[DEBUG] All posteriors saved to %s.\n", url.c_str());
}

void RTreeClassifier::setPosteriorsFromTextfile(std::string url, int red_dim)
{
  std::ifstream ifs(url.c_str());
  for (int i=0; i<(int)trees_.size(); ++i)
    for (int k=0; k<trees_[i].num_leaves_; ++k) {
      float *post = trees_[i].getPosteriorByIndex(k);
      for (int j=0; j<red_dim; ++j, ++post)
        ifs >> *post;
      assert(ifs.good());
    }
  ifs.close();
  printf("[EXP] read entire tree from '%s'\n", url.c_str());  
}

} // namespace features
