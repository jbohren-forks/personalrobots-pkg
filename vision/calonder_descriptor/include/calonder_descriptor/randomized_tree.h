#ifndef FEATURES_RANDOMIZED_TREE_H
#define FEATURES_RANDOMIZED_TREE_H

#include "calonder_descriptor/rng.h"
#include "calonder_descriptor/patch_generator.h"
#include <cvwimage.h>
#include <vector>
#include <iostream>
#include <boost/random.hpp>

class RTTester;

namespace features {

struct RTreeNode;

struct BaseKeypoint
{
  int x;
  int y;
  IplImage* image;

  BaseKeypoint()
    : x(0), y(0), image(NULL)
  {}
  
  BaseKeypoint(int x, int y, IplImage* image)
    : x(x), y(y), image(image)
  {}
};

class CSMatrixGenerator {
public:
   typedef enum { PDT_GAUSS=1, PDT_BERNOULLI, PDT_DBFRIENDLY } PHI_DISTR_TYPE;
   ~CSMatrixGenerator();
   static float* getCSMatrix(int m, int n, PHI_DISTR_TYPE dt);     // do NOT free returned pointer   
   
   
private:
   static float *cs_phi_;    // matrix for compressive sensing
   static int cs_phi_m_, cs_phi_n_;
};

class RandomizedTree
{  
public:
  friend class RTreeClassifier;  
  friend class ::RTTester;
  
  static const int PATCH_SIZE = 32;
  static const int DEFAULT_DEPTH = 9;
  static const int DEFAULT_VIEWS = 5000;
  static const size_t DEFAULT_REDUCED_NUM_DIM = 176;  
  static const float LOWER_QUANT_PERC = .03f;
  static const float UPPER_QUANT_PERC = .92f;
  
  RandomizedTree();
  ~RandomizedTree();
  
  void train(std::vector<BaseKeypoint> const& base_set, Rng &rng,
             int depth, int views, size_t reduced_num_dim, int num_quant_bits);
  void train(std::vector<BaseKeypoint> const& base_set, Rng &rng,
             PatchGenerator &make_patch, int depth, int views, size_t reduced_num_dim,
             int num_quant_bits);

  // following two funcs are EXPERIMENTAL (do not use unless you know exactly what you do)
  static void quantizeVector(float *vec, int dim, int N, float bnds[2], int clamp_mode=0);
  static void quantizeVector(float *src, int dim, int N, float bnds[2], uint8_t *dst);  

  // patch_data must be a 32x32 array (no row padding)
  float* getPosterior(uchar* patch_data);
  const float* getPosterior(uchar* patch_data) const;
  uint8_t* getPosterior2(uchar* patch_data);

  void read(const char* file_name, int num_quant_bits);
  void read(std::istream &is, int num_quant_bits);
  void write(const char* file_name) const;
  void write(std::ostream &os) const;

  int classes() { return classes_; }
  int depth() { return depth_; }
    
  //void setKeepFloatPosteriors(bool b) { keep_float_posteriors_ = b; }
  void discardFloatPosteriors() { freePosteriors(1); }
  
  inline void applyQuantization(int num_quant_bits) { makePosteriors2(num_quant_bits); }
  
  // debug
  void savePosteriors(std::string url, bool append=false);
  void savePosteriors2(std::string url, bool append=false);
    
private:
  int classes_;
  int depth_;
  int num_leaves_;  
  std::vector<RTreeNode> nodes_;  
  float **posteriors_;        // 16-bytes aligned posteriors
  uint8_t **posteriors2_;     // 16-bytes aligned posteriors
  std::vector<int> leaf_counts_;

  void createNodes(int num_nodes, Rng &rng);
  void allocPosteriorsAligned(int num_leaves, int num_classes);
  void freePosteriors(int which);    // which: 1=posteriors_, 2=posteriors2_, 3=both
  void init(int classes, int depth, Rng &rng);
  void addExample(int class_id, uchar* patch_data);
  void finalize(size_t reduced_num_dim, int num_quant_bits);  
  int getIndex(uchar* patch_data) const;
  inline float* getPosteriorByIndex(int index);
  inline uint8_t* getPosteriorByIndex2(int index);
  inline const float* getPosteriorByIndex(int index) const;
  //void makeRandomMeasMatrix(float *cs_phi, PHI_DISTR_TYPE dt, size_t reduced_num_dim);  
  void convertPosteriorsToChar();
  void makePosteriors2(int num_quant_bits);
  void compressLeaves(size_t reduced_num_dim);  
  void estimateQuantPercForPosteriors(float perc[2]);
};


inline uchar* getData(IplImage* image)
{
  return reinterpret_cast<uchar*>(image->imageData);
}

inline float* RandomizedTree::getPosteriorByIndex(int index)
{
  return const_cast<float*>(const_cast<const RandomizedTree*>(this)->getPosteriorByIndex(index)); 
}

inline const float* RandomizedTree::getPosteriorByIndex(int index) const
{
  return posteriors_[index]; 
}

inline uint8_t* RandomizedTree::getPosteriorByIndex2(int index)
{
  return posteriors2_[index];
}


template < typename PointT >
cv::WImageView1_b extractPatch(cv::WImageView1_b const& image, PointT pt, int patch_sz = RandomizedTree::PATCH_SIZE)
{
  const int offset = patch_sz / 2;

  // TODO: WImage{C}.View really should have const version
  cv::WImageView1_b &img_ref = const_cast< cv::WImageView1_b& >(image);
  return img_ref.View(pt.x - offset, pt.y - offset, patch_sz, patch_sz);
}

template < typename PointT >
cv::WImageView3_b extractPatch3(cv::WImageView3_b const& image, PointT pt)
{
  static const int offset = RandomizedTree::PATCH_SIZE / 2;

  // TODO: WImage{C}.View really should have const version
  cv::WImageView3_b &img_ref = const_cast< cv::WImageView3_b& >(image);
  return img_ref.View(pt.x - offset, pt.y - offset,
                      RandomizedTree::PATCH_SIZE, RandomizedTree::PATCH_SIZE);
}

struct RTreeNode
{
  short offset1, offset2;

  RTreeNode() {}

  RTreeNode(uchar x1, uchar y1, uchar x2, uchar y2)
    : offset1(y1*RandomizedTree::PATCH_SIZE + x1),
      offset2(y2*RandomizedTree::PATCH_SIZE + x2)
  {}

  //! Left child on 0, right child on 1
  inline bool operator() (uchar* patch_data) const
  {
    return patch_data[offset1] > patch_data[offset2];
  }
};

template <typename FLT_T>
inline FLT_T sample_normal(FLT_T mean, FLT_T sigma)
{
    using namespace boost;
    
    // Create a Mersenne twister random number generator
    static mt19937 rng(23);
 
    // select Gaussian probability distribution
    normal_distribution<FLT_T> norm_dist(mean, sigma);
 
    // bind random number generator to distribution, forming a function
    variate_generator<mt19937&, normal_distribution<FLT_T> >  normal_sampler(rng, norm_dist);
 
    // sample from the distribution
    return normal_sampler();
}

} // namespace features


#endif
