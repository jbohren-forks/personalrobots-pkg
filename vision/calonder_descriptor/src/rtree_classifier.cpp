#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"
#include <fstream>
#include <cstring>
#include <boost/foreach.hpp>
#include <cmath>

namespace features {


RTreeClassifier::RTreeClassifier()
  : classes_(0)
{
  posteriors_ = NULL;  
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
    printf("INVALID PARAMS in RTreeClassifier::train: reduced_num_dim{%i} > base_set.size(){%i}\n",
           reduced_num_dim, base_set.size());
    return;
  }
  
  num_quant_bits_ = num_quant_bits;
  classes_ = reduced_num_dim; // base_set.size();
  original_num_classes_ = base_set.size();
  trees_.resize(num_trees);  
  
  printf("[OK] Training trees: base size=%i, reduced size=%i\n", base_set.size(), reduced_num_dim); 
  
  int count = 1;
  printf("[OK] Trained 0 / %i trees", num_trees);  fflush(stdout);
  BOOST_FOREACH( RandomizedTree &tree, trees_ ) {
    tree.train(base_set, rng, make_patch, depth, views, reduced_num_dim, num_quant_bits_);
    printf("\r[OK] Trained %i / %i trees", count++, num_trees);
    fflush(stdout);
  }  
      
  printf("\n");
  countZeroElements();
  printf("\n\n");  
}

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
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it, pp++) {
    *pp = tree_it->getPosterior(patch_data);       
    assert(*pp != NULL);
  }

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
//void sum_50c_176c(uint8_t **pp, uint8_t *sig)
//{
  
//}

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

   #if 1    // SSE2 optimized code     
     sum_50t_176c(pp, sig, ptemp_);    // sum them up  
   #else
     static bool warned = false;
     
     memset((void*)sig, 0, classes_ * sizeof(sig[0]));
     uint16_t *sig16 = new uint16_t[classes_];           // TODO: make member, no alloc here
     memset((void*)sig16, 0, classes_ * sizeof(sig16[0]));
     for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it, pp++)
       add(classes_, sig16, *pp, sig16);

     // squeeze signatures into an uint8_t
     const bool full_shifting = true;
     int shift;
     if (full_shifting) {
        float num_add_bits_f = log((float)trees_.size())/log(2.f);     // # additional bits required due to summation
        int num_add_bits = int(num_add_bits_f);
        if (num_add_bits_f != float(num_add_bits)) ++num_add_bits;                  
        shift = num_quant_bits_ + num_add_bits - 8*sizeof(uint8_t);
//shift = num_quant_bits_ + num_add_bits - 2;
//shift = 6;
        if (shift>0)
          for (int i = 0; i < classes_; ++i)
            sig[i] = (sig16[i] >> shift);      // &3 cut off all but lowest 2 bits, 3(dec) = 11(bin)
        
        if (!warned) 
           printf("[OK] RTC: quantizing by FULL RIGHT SHIFT, shift = %i\n", shift);
     } 
     else {
        printf("[ERROR] RTC: not implemented!\n");
        exit(0);
     }
     
     if (!warned)
        printf("[WARNING] RTC: unoptimized signature computation\n");       
     warned = true;       
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
  is.read((char*)(&num_quant_bits_), sizeof(num_quant_bits_));
  
  if (num_quant_bits_<1 || num_quant_bits_>8) {
    printf("[WARNING] RTC: suspicious value num_quant_bits_=%i found; setting to %i.\n", 
           num_quant_bits_, DEFAULT_NUM_QUANT_BITS);
    num_quant_bits_ = DEFAULT_NUM_QUANT_BITS;
  }
  
  trees_.resize(num_trees);
  std::vector<RandomizedTree>::iterator tree_it;

  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it) {
    tree_it->read(is, num_quant_bits_);
  }    
  
  printf("[OK] Loaded RTC, quantization=%i bits\n", num_quant_bits_);

  countZeroElements();
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
  os.write((char*)(&num_quant_bits_), sizeof(num_quant_bits_));
printf("RTreeClassifier::write: num_quant_bits_=%i\n", num_quant_bits_);

  std::vector<RandomizedTree>::const_iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it)
    tree_it->write(os);
}

void RTreeClassifier::saveAllFloatPosteriors(std::string url)
{  
  printf("[DEBUG] writing all float posteriors to %s...\n", url.c_str());
  for (int i=0; i<(int)trees_.size(); ++i)
    trees_[i].savePosteriors(url, (i==0 ? false : true));
  printf("[DEBUG] done\n");
}

void RTreeClassifier::saveAllBytePosteriors(std::string url)
{  
  printf("[DEBUG] writing all byte posteriors to %s...\n", url.c_str());
  for (int i=0; i<(int)trees_.size(); ++i)
    trees_[i].savePosteriors2(url, (i==0 ? false : true));
  printf("[DEBUG] done\n");
}


void RTreeClassifier::setFloatPosteriorsFromTextfile_176(std::string url)
{   
   std::ifstream ifs(url.c_str());
      
   for (int i=0; i<(int)trees_.size(); ++i) {
      int num_classes = trees_[i].classes_;
      assert(num_classes == 176);     // TODO: remove this limitation (arose due to SSE2 optimizations)
      for (int k=0; k<trees_[i].num_leaves_; ++k) {
         float *post = trees_[i].getPosteriorByIndex(k);
         for (int j=0; j<num_classes; ++j, ++post)
            ifs >> *post;
         assert(ifs.good());
      }
   }
   classes_ = 176;
   
   //setQuantization(num_quant_bits_);
   
   ifs.close();
   printf("[EXPERIMENTAL] read entire tree from '%s'\n", url.c_str());  
}


float RTreeClassifier::countZeroElements()
{   
   int flt_zeros = 0;
   int ui8_zeros = 0;
   int num_elem = trees_[0].classes();   
   for (int i=0; i<(int)trees_.size(); ++i)
      for (int k=0; k<(int)trees_[i].num_leaves_; ++k) {
         float *p = trees_[i].getPosteriorByIndex(k);
         uint8_t *p2 = trees_[i].getPosteriorByIndex2(k);
         assert(p); assert(p2);
         for (int j=0; j<num_elem; ++j, ++p, ++p2) {
            if (*p == 0.f) flt_zeros++;      
            if (*p2 == 0) ui8_zeros++;
         }   
      }
   num_elem = trees_.size()*trees_[0].num_leaves_*num_elem;
   float flt_perc = 100.*flt_zeros/num_elem;
   float ui8_perc = 100.*ui8_zeros/num_elem;
   printf("[OK] RTC: overall %i/%i (%.3f%%) zeros in float leaves\n", flt_zeros, num_elem, flt_perc);
   printf("          overall %i/%i (%.3f%%) zeros in uint8 leaves\n", ui8_zeros, num_elem, ui8_perc);

   return flt_perc;
}

void RTreeClassifier::setQuantization(int num_quant_bits)
{        
   for (int i=0; i<(int)trees_.size(); ++i)
      trees_[i].applyQuantization(num_quant_bits);
   
   printf("[OK] signature quantization is now %i bits (before: %i)\n", num_quant_bits, num_quant_bits_);
   num_quant_bits_ = num_quant_bits;
}

void RTreeClassifier::discardFloatPosteriors()
{
   for (int i=0; i<(int)trees_.size(); ++i)
      trees_[i].discardFloatPosteriors();
   printf("[OK] RTC: discarded float posteriors of all trees\n");
}

} // namespace features


















