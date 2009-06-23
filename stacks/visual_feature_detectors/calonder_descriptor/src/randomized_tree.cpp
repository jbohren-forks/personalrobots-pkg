#include "calonder_descriptor/randomized_tree.h"
#include "calonder_descriptor/rng.h"
#include <fstream>
#include <boost/foreach.hpp>
#include <cblas.h>
#include <highgui.h>

namespace features {

float *CSMatrixGenerator::cs_phi_   = NULL;
int    CSMatrixGenerator::cs_phi_m_ = 0;
int    CSMatrixGenerator::cs_phi_n_ = 0;   

RandomizedTree::RandomizedTree() 
  : posteriors_(NULL), posteriors2_(NULL)
{ 
}

RandomizedTree::~RandomizedTree()
{
   freePosteriors(3);
}

void RandomizedTree::createNodes(int num_nodes, Rng &rng)
{
  nodes_.reserve(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    nodes_.push_back( RTreeNode(rng(RandomizedTree::PATCH_SIZE),
                                rng(RandomizedTree::PATCH_SIZE),
                                rng(RandomizedTree::PATCH_SIZE),
                                rng(RandomizedTree::PATCH_SIZE)) );
  }
}

int RandomizedTree::getIndex(uchar* patch_data) const
{
  int index = 0;
  for (int d = 0; d < depth_; ++d) {
    int child_offset = nodes_[index](patch_data);
    index = 2*index + 1 + child_offset;
  }
  return index - nodes_.size();
}

void RandomizedTree::train(std::vector<BaseKeypoint> const& base_set,
                           Rng &rng, int depth, int views, size_t reduced_num_dim,
                           int num_quant_bits)
{
  PatchGenerator make_patch(NULL, rng);
  train(base_set, rng, make_patch, depth, views, reduced_num_dim, num_quant_bits);
}

void RandomizedTree::train(std::vector<BaseKeypoint> const& base_set,
                           Rng &rng, PatchGenerator &make_patch,
                           int depth, int views, size_t reduced_num_dim,
                           int num_quant_bits)
{
  init(base_set.size(), depth, rng);
  
  IplImage* patch = cvCreateImage(cvSize(PATCH_SIZE, PATCH_SIZE), IPL_DEPTH_8U, 1);

  // Estimate posterior probabilities using random affine views
  std::vector<BaseKeypoint>::const_iterator keypt_it;
  int class_id = 0;
  for (keypt_it = base_set.begin(); keypt_it != base_set.end(); ++keypt_it, ++class_id) {
    make_patch.setSource(keypt_it->image);
    
    for (int i = 0; i < views; ++i) {
      make_patch(cvPoint(keypt_it->x, keypt_it->y), patch);
      addExample(class_id, getData(patch));
    }
  }
  
  finalize(reduced_num_dim, num_quant_bits);

  cvReleaseImage(&patch);
}

void RandomizedTree::allocPosteriorsAligned(int num_leaves, int num_classes)
{  
  freePosteriors(3);
  int err_cnt = 0;
  
  posteriors_ = new float*[num_leaves]; //(float**) malloc(num_leaves*sizeof(float*)); 
  for (int i=0; i<num_leaves; ++i) {
    err_cnt += posix_memalign((void**)&posteriors_[i], 16, num_classes*sizeof(float));  //(float*)malloc(num_classes*sizeof(float));
    memset(posteriors_[i], 0, num_classes*sizeof(float));
  }
  
  posteriors2_ = new uint8_t*[num_leaves];
  for (int i=0; i<num_leaves; ++i) {
    err_cnt += posix_memalign((void**)&posteriors2_[i], 16, num_classes*sizeof(uint8_t));
    memset(posteriors2_[i], 0, num_classes*sizeof(uint8_t));
  }
  
  if (err_cnt) {
    printf("Something went wrong in posix_memalign()! err_cnt=%i\n", err_cnt);
    exit(0);
  }
  
  classes_ = num_classes;
}

void RandomizedTree::freePosteriors(int which)
{
   if (posteriors_ && (which&1)) {      
      for (int i=0; i<num_leaves_; ++i) {
         if (posteriors_[i]) {
            free(posteriors_[i]); //delete [] posteriors_[i];
            posteriors_[i] = NULL;
         }
      }
      delete [] posteriors_;
      posteriors_ = NULL;
   }
   
   if (posteriors2_ && (which&2)) {
      for (int i=0; i<num_leaves_; ++i)
         free(posteriors2_[i]);
      delete [] posteriors2_;
      posteriors2_ = NULL;
   }
   
   classes_ = -1;
}

void RandomizedTree::init(int num_classes, int depth, Rng &rng)
{
  depth_ = depth;
  num_leaves_ = 1 << depth;        // 2**d
  int num_nodes = num_leaves_ - 1; // 2**d - 1
  
  // Initialize probabilities and counts to 0
  allocPosteriorsAligned(num_leaves_, num_classes);      // will set classes_ correctly
  for (int i = 0; i < num_leaves_; ++i)
    memset((void*)posteriors_[i], 0, num_classes*sizeof(float));
  leaf_counts_.resize(num_leaves_);

  for (int i = 0; i < num_leaves_; ++i)
    memset((void*)posteriors2_[i], 0, num_classes*sizeof(uint8_t));

  createNodes(num_nodes, rng);
}

void RandomizedTree::addExample(int class_id, uchar* patch_data)
{
  int index = getIndex(patch_data);
  float* posterior = getPosteriorByIndex(index);
  ++leaf_counts_[index];
  ++posterior[class_id];
}

// returns the p% percentile of data (length n vector)
static float percentile(float *data, int n, float p)
{
   assert(n>0);
   assert(p>=0 && p<=1);
   std::vector<float> vec(data, data+n);
   sort(vec.begin(), vec.end());
   int ix = (int)(p*(n-1));
   return vec[ix];
}

void RandomizedTree::finalize(size_t reduced_num_dim, int num_quant_bits)
{     
   // Normalize by number of patches to reach each leaf   
   for (int index = 0; index < num_leaves_; ++index) {
      float* posterior = posteriors_[index];
      assert(posterior != NULL);
      int count = leaf_counts_[index];
      if (count != 0) {
         float normalizer = 1.0f / count;
         for (int c = 0; c < classes_; ++c) {
            *posterior *= normalizer;
            ++posterior;
         }
      } 
   }
   leaf_counts_.clear();

   // apply compressive sensing
   if ((int)reduced_num_dim != classes_)
      compressLeaves(reduced_num_dim);
   else {
      static bool notified = false;
      if (!notified)
         printf("\n[OK] NO compression to leaves applied, dim=%i\n", reduced_num_dim);
      notified = true;
   }
  
   // convert float-posteriors to char-posteriors (quantization step)
   makePosteriors2(num_quant_bits);
}

void RandomizedTree::compressLeaves(size_t reduced_num_dim)
{   
   static bool warned = false;
   if (!warned) {
      printf("\n[OK] compressing leaves with phi %i x %i\n", reduced_num_dim, classes_);
      warned = true;
   }

   static bool warned2 = false;
   if ((int)reduced_num_dim == classes_) {
     if (!warned2)
       printf("[WARNING] RandomizedTree::compressLeaves:  not compressing because reduced_dim == classes()\n");
     warned2 = true;
     return;
   }
   
   // DO NOT FREE RETURNED POINTER
   float *cs_phi = CSMatrixGenerator::getCSMatrix(reduced_num_dim, classes_, CSMatrixGenerator::PDT_BERNOULLI);

   float *cs_posteriors = new float[num_leaves_ * reduced_num_dim];         // temp, num_leaves_ x reduced_num_dim
   for (int i=0; i<num_leaves_; ++i) {
      float *post = getPosteriorByIndex(i);
      float *prod = &cs_posteriors[i*reduced_num_dim];
      cblas_sgemv(CblasRowMajor, CblasNoTrans, reduced_num_dim, classes_, 1.f, cs_phi,
                  classes_, post, 1, 0.f, prod, 1);       
   }

   // copy new posteriors
   freePosteriors(3);
   allocPosteriorsAligned(num_leaves_, reduced_num_dim);
   for (int i=0; i<num_leaves_; ++i)
      memcpy(posteriors_[i], &cs_posteriors[i*reduced_num_dim], reduced_num_dim*sizeof(float));
   classes_ = reduced_num_dim;

   delete [] cs_posteriors;   
}

void RandomizedTree::makePosteriors2(int num_quant_bits)
{   
   int N = (1<<num_quant_bits) - 1;   

   float perc[2];
   estimateQuantPercForPosteriors(perc);

   assert(posteriors_ != NULL);
   for (int i=0; i<num_leaves_; ++i)      
      quantizeVector(posteriors_[i], classes_, N, perc, posteriors2_[i]);

   // printf("makePosteriors2 quantization bounds: %.3e, %.3e (num_leaves=%i, N=%i)\n", 
   //        perc[0], perc[1], num_leaves_, N);      
}

void RandomizedTree::estimateQuantPercForPosteriors(float perc[2])
{
   // _estimate_ percentiles for this tree
   // TODO: do this more accurately
   assert(posteriors_ != NULL);
   perc[0] = perc[1] = .0f;
   for (int i=0; i<num_leaves_; i++) {
      perc[0] += percentile(posteriors_[i], classes_, LOWER_QUANT_PERC);
      perc[1] += percentile(posteriors_[i], classes_, UPPER_QUANT_PERC);
   }
   perc[0] /= num_leaves_;
   perc[1] /= num_leaves_;
}


float* RandomizedTree::getPosterior(uchar* patch_data)
{
  return const_cast<float*>(const_cast<const RandomizedTree*>(this)->getPosterior(patch_data));
}

const float* RandomizedTree::getPosterior(uchar* patch_data) const
{
  return getPosteriorByIndex( getIndex(patch_data) );
}

uint8_t* RandomizedTree::getPosterior2(uchar* patch_data)
{
   return getPosteriorByIndex2( getIndex(patch_data) );
}

void RandomizedTree::quantizeVector(float *vec, int dim, int N, float bnds[2], int clamp_mode)
{   
   float map_bnd[2] = {0.f,(float)N};          // bounds of quantized target interval we're mapping to
   for (int k=0; k<dim; ++k, ++vec) {
      *vec = float(int((*vec - bnds[0])/(bnds[1] - bnds[0])*(map_bnd[1] - map_bnd[0]) + map_bnd[0]));
      // 0: clamp both, lower and upper values
      if (clamp_mode == 0)      *vec = (*vec<map_bnd[0]) ? map_bnd[0] : ((*vec>map_bnd[1]) ? map_bnd[1] : *vec);
      // 1: clamp lower values only
      else if (clamp_mode == 1) *vec = (*vec<map_bnd[0]) ? map_bnd[0] : *vec;
      // 2: clamp upper values only
      else if (clamp_mode == 2) *vec = (*vec>map_bnd[1]) ? map_bnd[1] : *vec;
      // 4: no clamping
      else if (clamp_mode == 4) ; // yep, nothing
      else {
         printf("clamp_mode == %i is not valid (%s:%i).\n", clamp_mode, __FILE__, __LINE__);
         exit(1);
      }
   }

}

void RandomizedTree::quantizeVector(float *vec, int dim, int N, float bnds[2], uint8_t *dst)
{   
   int map_bnd[2] = {0, N};          // bounds of quantized target interval we're mapping to
   int tmp;
   for (int k=0; k<dim; ++k) {
      tmp = int((*vec - bnds[0])/(bnds[1] - bnds[0])*(map_bnd[1] - map_bnd[0]) + map_bnd[0]);
      *dst = (uint8_t)((tmp<0) ? 0 : ((tmp>N) ? N : tmp));
      ++vec;
      ++dst;
   }
}


void RandomizedTree::read(const char* file_name, int num_quant_bits)
{
  std::ifstream file(file_name, std::ifstream::binary);
  read(file, num_quant_bits);
  file.close();
}

void RandomizedTree::read(std::istream &is, int num_quant_bits)
{
  is.read((char*)(&classes_), sizeof(classes_));
  is.read((char*)(&depth_), sizeof(depth_));  
  
  num_leaves_ = 1 << depth_;
  int num_nodes = num_leaves_ - 1;

  nodes_.resize(num_nodes);
  is.read((char*)(&nodes_[0]), num_nodes * sizeof(nodes_[0]));

  //posteriors_.resize(classes_ * num_leaves_);
  //freePosteriors(3);
  //printf("[DEBUG] reading: %i leaves, %i classes\n", num_leaves_, classes_);  
  allocPosteriorsAligned(num_leaves_, classes_);
  for (int i=0; i<num_leaves_; i++)
    is.read((char*)posteriors_[i], classes_ * sizeof(*posteriors_[0]));

  // make char-posteriors from float-posteriors
  makePosteriors2(num_quant_bits);
}

void RandomizedTree::write(const char* file_name) const
{
  std::ofstream file(file_name, std::ofstream::binary);
  write(file);
  file.close();
}

void RandomizedTree::write(std::ostream &os) const
{
  if (!posteriors_) {
    printf("WARNING: Cannot write float posteriors (posteriors_ = NULL).\n");
    return;
  }
  
  os.write((char*)(&classes_), sizeof(classes_));
  os.write((char*)(&depth_), sizeof(depth_));  
   
  os.write((char*)(&nodes_[0]), nodes_.size() * sizeof(nodes_[0]));  
  for (int i=0; i<num_leaves_; i++) {
    os.write((char*)posteriors_[i], classes_ * sizeof(*posteriors_[0]));
  }
}


void RandomizedTree::savePosteriors(std::string url, bool append)
{   
   std::ofstream file(url.c_str(), (append?std::ios::app:std::ios::out));
   for (int i=0; i<num_leaves_; i++) {
      float *post = posteriors_[i];      
      char buf[20];
      for (int i=0; i<classes_; i++) {
         sprintf(buf, "%.10e", *post++);
         file << buf << ((i<classes_-1) ? " " : "");
      }
      file << std::endl;      
   }
   file.close();
}

void RandomizedTree::savePosteriors2(std::string url, bool append)
{   
   std::ofstream file(url.c_str(), (append?std::ios::app:std::ios::out));
   for (int i=0; i<num_leaves_; i++) {
      uint8_t *post = posteriors2_[i];      
      for (int i=0; i<classes_; i++)
         file << int(*post++) << (i<classes_-1?" ":"");
      file << std::endl;
   }
   file.close();
}

float* CSMatrixGenerator::getCSMatrix(int m, int n, PHI_DISTR_TYPE dt)
{
   assert(m <= n);
   
   if (cs_phi_m_!=m || cs_phi_n_!=n || cs_phi_==NULL) {
      if (cs_phi_) delete [] cs_phi_;
      cs_phi_ = new float[m*n];
   }

   #if 0 // debug - load the random matrix from a file (for reproducability of results)
      //assert(m == 176);
      //assert(n == 500);
      //const char *phi = "/u/calonder/temp/dim_red/kpca_phi.txt";
      const char *phi = "/u/calonder/temp/dim_red/debug_phi.txt";
      std::ifstream ifs(phi);
      for (size_t i=0; i<m*n; ++i) {
         if (!ifs.good()) {
            printf("[ERROR] RandomizedTree::makeRandomMeasMatrix: problem reading '%s'\n", phi);
            exit(0);
         }
         ifs >> cs_phi[i];
      }   
      ifs.close(); 

      static bool warned=false;
      if (!warned) {
         printf("[NOTE] RT: reading %ix%i PHI matrix from '%s'...\n", m, n, phi);
         warned=true;
      }

      return;   
   #endif

   float *cs_phi = cs_phi_;
   
   if (m == n) {
      // special case - set to 0 for safety
      memset(cs_phi, 0, m*n*sizeof(float));
      printf("[WARNING] %s:%i: square CS matrix (-> no reduction)\n", __FILE__, __LINE__);
   }
   else {
      Rng rng(23);
      
      // par is distr param, cf 'Favorable JL Distributions' (Baraniuk et al, 2006)      
      if (dt == PDT_GAUSS) {
         float par = (float)(1./m);
         for (int i=0; i<m*n; ++i)
            *cs_phi++ = sample_normal<float>(0., par);
      }
      else if (dt == PDT_BERNOULLI) {
         float par = (float)(1./sqrt(m));
         for (int i=0; i<m*n; ++i)
            *cs_phi++ = (rng(2)==0 ? par : -par);
      }
      else if (dt == PDT_DBFRIENDLY) {
         float par = (float)sqrt(3./m); 
         for (int i=0; i<m*n; ++i) {
            int i = rng(6);
            *cs_phi++ = (i==0 ? par : (i==1 ? -par : 0.f));
         }
      }
      else
         throw("PHI_DISTR_TYPE not implemented.");
   }
   
   return cs_phi_;
}

CSMatrixGenerator::~CSMatrixGenerator() 
{
   if (cs_phi_) delete [] cs_phi_;
   cs_phi_ = NULL;
}


} // namespace features
