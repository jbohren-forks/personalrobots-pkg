#include "calonder_descriptor/randomized_tree.h"
#include "calonder_descriptor/rng.h"
#include <fstream>
#include <boost/foreach.hpp>
#include <cblas.h>
//#include <boost/numeric/ublas/vector.hpp>
//#include <boost/numeric/ublas/operation.hpp>

namespace features {

RandomizedTree::RandomizedTree()
{
   posteriors_ = NULL;
}

RandomizedTree::~RandomizedTree()
{
   freePosteriors();
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
  
  IplImage* patch = cvCreateImage(cvSize(PATCH_SIZE, PATCH_SIZE),
                                  IPL_DEPTH_8U, 1);

  // Estimate posterior probabilities using random affine views
  std::vector<BaseKeypoint>::const_iterator keypt_it;
  int class_id = 0;
  for (keypt_it = base_set.begin(); keypt_it != base_set.end();
       ++keypt_it, ++class_id) {
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
  posteriors_ = new float*[num_leaves];
  for (int i=0; i<num_leaves; ++i)
    posix_memalign((void**)&posteriors_[i], 16, num_classes*sizeof(float));
  //printf("[DEBUG] tree alloc'ed: num_classes = %i, num_leaves = %i\n", num_classes, num_leaves);    
}

void RandomizedTree::freePosteriors()
{
   if (posteriors_) {
      for (int i=0; i<num_leaves_; ++i)
         free(posteriors_[i]);
      delete [] posteriors_;
      posteriors_ = NULL;
      //printf("[DEBUG] tree freed\n");
   }
}

void RandomizedTree::init(int classes, int depth, Rng &rng)
{
  classes_ = classes;
  depth_ = depth;
  num_leaves_ = 1 << depth;        // 2**d
  int num_nodes = num_leaves_ - 1; // 2**d - 1
  
  // Initialize probabilities and counts to 0
  allocPosteriorsAligned(num_leaves_, classes_);
  for (int i = 0; i < num_leaves_; ++i)
    memset((void*)posteriors_[i], 0, classes_*sizeof(float));
  leaf_counts_.resize(num_leaves_);

  createNodes(num_nodes, rng);
}

void RandomizedTree::addExample(int class_id, uchar* patch_data)
{
  int index = getIndex(patch_data);
  float* posterior = getPosteriorByIndex(index);
  ++leaf_counts_[index];
  ++posterior[class_id];
}

// returns the p% percentile of data
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
      int count = leaf_counts_[index];
      if (count != 0) {
         float normalizer = 1.0f / count;
         for (int c = 0; c < classes_; ++c) {
            *posterior *= normalizer;
            ++posterior;
         }
      } 
   }

   // apply compressive sensing
   if ((int)reduced_num_dim != classes_) {
      float *cs_phi = new float[reduced_num_dim * classes_];           // reduced_num_dim x classes_ matrix
      makeRandomMeasMatrix(cs_phi, PDT_BERNOULLI, reduced_num_dim);

      float *cs_posteriors = new float[num_leaves_ * reduced_num_dim];         // num_leaves_ x reduced_num_dim
      for (int i=0; i<num_leaves_; ++i) {
         float *post = getPosteriorByIndex(i);
         float *prod = &cs_posteriors[i*reduced_num_dim];
         cblas_sgemv(CblasRowMajor, CblasNoTrans, reduced_num_dim, classes_, 1.f, cs_phi,
                     classes_, post, 1, 0.f, prod, 1);       
      }

      // copy new posteriors
      freePosteriors();
      allocPosteriorsAligned(num_leaves_, reduced_num_dim);
      for (int i=0; i<num_leaves_; ++i)         
         memcpy(posteriors_[i], &cs_posteriors[i*reduced_num_dim], reduced_num_dim*sizeof(float));
      classes_ = reduced_num_dim;
   }
   
   // quantize (experimantal)   
   if (num_quant_bits > 0) {
      int N = (1<<num_quant_bits) - 1;
      printf("[WARNING] Quantization is active! N=%i\n", N);
      // estimate percentiles over all data (this here is approximative, fair enough)
      float perc[2] = {0,0};
      for (int i=0; i<num_leaves_; i++) {
         perc[0] += percentile(posteriors_[i], reduced_num_dim, .05);
         perc[1] += percentile(posteriors_[i], reduced_num_dim, .95);
      }
      perc[0] /= num_leaves_;
      perc[1] /= num_leaves_;

      for (int i=0; i<num_leaves_; ++i) {
         quantize_vector(posteriors_[i], reduced_num_dim, N, perc);      
         // DEBUG
         #if 0
         if (i == 13) {
            post = posteriors_[i];
            std::ofstream ofs("/wg/stor1/calonder/dev/fw/train_base/debug/post13.txt");
            for (size_t i=0; i<reduced_num_dim; i++)
               ofs << *post++ << std::endl;
            ofs.close();
         }
         #endif
      }   
   }
       
   leaf_counts_.clear();
}

void RandomizedTree::quantize_vector(float *vec, int dim, int N, float perc[2])
{
   // TODO: find percentiles here (no hardcoding!)
   float map_bnd[2] = {0.f,(float)N};          // bounds of quantized target interval we're mapping to
   for (int k=0; k<dim; ++k, ++vec) {
      int p = int((*vec - perc[0])/(perc[1] - perc[0])*(map_bnd[1] - map_bnd[0]) + map_bnd[0]);
      *vec = (float)p;
      *vec = (*vec<map_bnd[0]) ? map_bnd[0] : ((*vec>map_bnd[1]) ? map_bnd[1] : *vec);
   }

}

float* RandomizedTree::getPosterior(uchar* patch_data)
{
  return const_cast<float*>(const_cast<const RandomizedTree*>(this)->getPosterior(patch_data));
}

const float* RandomizedTree::getPosterior(uchar* patch_data) const
{
  return getPosteriorByIndex( getIndex(patch_data) );
}

void RandomizedTree::read(const char* file_name)
{
  std::ifstream file(file_name, std::ifstream::binary);
  read(file);
  file.close();
}

void RandomizedTree::read(std::istream &is)
{
  is.read((char*)(&classes_), sizeof(classes_));
  is.read((char*)(&depth_), sizeof(depth_));

  num_leaves_ = 1 << depth_;
  int num_nodes = num_leaves_ - 1;

  nodes_.resize(num_nodes);
  is.read((char*)(&nodes_[0]), num_nodes * sizeof(nodes_[0]));

  //posteriors_.resize(classes_ * num_leaves_);
  freePosteriors();
  allocPosteriorsAligned(num_leaves_, classes_);  
  for (int i=0; i<num_leaves_; i++)
    is.read((char*)posteriors_[i], classes_ * sizeof(*posteriors_[0]));
}

void RandomizedTree::write(const char* file_name) const
{
  std::ofstream file(file_name, std::ofstream::binary);
  write(file);
  file.close();
}

void RandomizedTree::write(std::ostream &os) const
{
  os.write((char*)(&classes_), sizeof(classes_));
  os.write((char*)(&depth_), sizeof(depth_));

  os.write((char*)(&nodes_[0]), nodes_.size() * sizeof(nodes_[0]));  
  for (int i=0; i<num_leaves_; i++)
    os.write((char*)posteriors_[i], classes_ * sizeof(*posteriors_[0]));
}

void RandomizedTree::makeRandomMeasMatrix(float *cs_phi, PHI_DISTR_TYPE dt, size_t reduced_num_dim)
{
   if ((int)reduced_num_dim == classes_) {
      // special case - will not make use of Compressive Sensing AT ALL (set to 0 for safety)
      memset(cs_phi, 0, reduced_num_dim*classes_*sizeof(float));
   }
   else {
      Rng rng(23);
      
      // par is distr param, cf 'Favorable JL Distributions' (Baraniuk et al, 2006)
      if (dt == PDT_GAUSS) {
         float par = (float)(1./reduced_num_dim);
         for (size_t i=0; i<reduced_num_dim*classes_; ++i)
            *cs_phi++ = sample_normal<float>(0., par);
      }
      else if (dt == PDT_BERNOULLI) {
         float par = (float)(1./sqrt(reduced_num_dim));
         for (size_t i=0; i<reduced_num_dim*classes_; ++i)
            *cs_phi++ = (rng(2)==0 ? par : -par);
      }
      else if (dt == PDT_DBFRIENDLY) {
         float par = (float)sqrt(3./reduced_num_dim); 
         for (size_t i=0; i<reduced_num_dim*classes_; ++i) {
            int i = rng(6);
            *cs_phi++ = (i==0 ? par : (i==1 ? -par : 0.f));
         }
      }
      else
         throw("PHI_DISTR_TYPE not implemented.");
   }
   
   //printf("[OK] created %i x %i CS meas matrix.\n", reduced_num_dim, classes_);
}

/*
void RandomizedTree::setMeasMatrix(std::string filename, size_t dim_m)
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
}*/


} // namespace features
