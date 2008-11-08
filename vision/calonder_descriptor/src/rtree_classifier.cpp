#define BOOST_UBLAS_SHALLOW_ARRAY_ADAPTOR
#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/patch_generator.h"
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/random.hpp>

namespace features {

static double sampleNormal(double mean, double sigma)
{
    using namespace boost;
    
    // Create a Mersenne twister random number generator
    static mt19937 rng(23);
 
    // select Gaussian probability distribution
    normal_distribution<double> norm_dist(mean, sigma);
 
    // bind random number generator to distribution, forming a function
    variate_generator<mt19937&, normal_distribution<double> >  normal_sampler(rng, norm_dist);
 
    // sample from the distribution
    return normal_sampler();
}


RTreeClassifier::RTreeClassifier()
  : classes_(0), threshold_(0)
{
}

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
DenseSignature RTreeClassifier::getDenseSignature(IplImage* patch) const
{
  DenseSignature sig = ublas::zero_vector<float>(classes_);

  std::vector<RandomizedTree>::const_iterator tree_it;
  for (tree_it = trees_.begin(); tree_it != trees_.end(); ++tree_it) {
    const float* post_array = tree_it->getPosterior(patch);
    // Cram float* into uBLAS-friendly type without copying
    typedef const ublas::shallow_array_adaptor<float> PostStorage;
    typedef const ublas::vector<float, PostStorage> PostVec;
    PostVec post(classes_, PostStorage(classes_, const_cast<float*>(post_array)) );
    sig += post;
  }

  // TODO: get rid of this multiply
  sig *= (1.0 / trees_.size());

  return sig;
}

DenseSignature RTreeClassifier::getCompressedSignature(IplImage* patch, bool from_sparse)
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
}

void RTreeClassifier::makeRandomMeasMatrix(size_t dim_m, PHI_DISTR_TYPE dt)
{
   // set param according to selected distribution
   float par;
   switch (dt) {
      case PHI_GAUSS:
         par = (float)(1./dim_m); break;
      case PHI_BERNOULLI:
         par = (float)(1./sqrt(dim_m)); break;
      case PHI_DBFRIENDLY:
         par = (float)sqrt(3./dim_m); break;
      default: 
         throw("this is impossible");
   }
   
   // create random measurement matrix
   Rng rng(23);
   cs_phi_.resize(dim_m, classes_);
   for (size_t m=0; m<dim_m; ++m)
      for (int n=0; n<classes_; ++n)
         if (dt == PHI_GAUSS)
            cs_phi_(m,n) = sampleNormal(0., par);
         else if (dt == PHI_BERNOULLI)
            cs_phi_(m,n) = (rng(2)==0 ? par : -par);
         else {
            int i = rng(6);
            cs_phi_(m,n) = (i==0 ? par : (i==1 ? -par : 0.f));
         }
 
   printf("[OK] created CS meas matrix - dims %i x %i .\n", dim_m, classes_);
}

void RTreeClassifier::setMeasMatrix(std::string filename, size_t dim_m)
{
   cs_phi_.resize(dim_m, classes_);
   std::ifstream ifs(filename.c_str());
   for (size_t i=0; i<dim_m; ++i) {
      for (int k=0; k<classes_; ++k)
         ifs >> cs_phi_(i,k);
      if (!ifs.good() && !(i==dim_m-1 && ifs.eof())) {
         printf("[WARNING] Not enough values in meas matrix file. Using makeRandomMeasMatrix instead.\n");
         makeRandomMeasMatrix(dim_m);
         break;
      }      
   }
   ifs.close();
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
