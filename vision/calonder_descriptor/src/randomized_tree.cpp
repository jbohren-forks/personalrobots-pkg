#include "calonder_descriptor/randomized_tree.h"
#include "calonder_descriptor/rng.h"
#include <fstream>
#include <cassert> // DEBUG
#include <cstdio> // DEBUG

namespace features {

RandomizedTree::RandomizedTree()
{}

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

//int RandomizedTree::getIndex(cv::WImageView1_b const& patch) const
int RandomizedTree::getIndex(IplImage* patch) const
{
  int index = 0;
  for (int d = 0; d < depth_; ++d) {
    int child_offset = nodes_[index](patch);
    index = 2*index + 1 + child_offset;
  }
  return index - nodes_.size();
}

inline float* RandomizedTree::getPosteriorByIndex(int index)
{
  return const_cast<float*>(const_cast<const RandomizedTree*>(this)->getPosteriorByIndex(index));
}

inline const float* RandomizedTree::getPosteriorByIndex(int index) const
{
  return &posteriors_[index * classes_];
}

void RandomizedTree::train(std::vector<BaseKeypoint> const& base_set,
                           Rng &rng, int depth, int views)
{
  PatchGenerator make_patch(NULL, rng);
  train(base_set, rng, make_patch, depth, views);
}

void RandomizedTree::train(std::vector<BaseKeypoint> const& base_set,
                           Rng &rng, PatchGenerator &make_patch,
                           int depth, int views)
{
  init(base_set.size(), depth, rng);
  /*
  cv::WImageBuffer1_b patch(RandomizedTree::PATCH_SIZE,
                            RandomizedTree::PATCH_SIZE);
  */
  IplImage* patch = cvCreateImage(cvSize(PATCH_SIZE, PATCH_SIZE),
                                  IPL_DEPTH_8U, 1);

  // Estimate posterior probabilities using random affine views
  std::vector<BaseKeypoint>::const_iterator keypt_it;
  int class_id = 0;
  for (keypt_it = base_set.begin(); keypt_it != base_set.end();
       ++keypt_it, ++class_id) {
    make_patch.setSource(keypt_it->image);
    
    for (int i = 0; i < views; ++i) {
      //make_patch(cvPoint(keypt_it->x, keypt_it->y), patch.Ipl());
      //addExample(class_id, patch.Ipl()); // Ipl() to avoid copying header
      make_patch(cvPoint(keypt_it->x, keypt_it->y), patch);
      addExample(class_id, patch);
    }
  }

  finalize();

  cvReleaseImage(&patch);
}

void RandomizedTree::init(int classes, int depth, Rng &rng)
{
  classes_ = classes;
  depth_ = depth;
  num_leaves_ = 1 << depth;        // 2**d
  int num_nodes = num_leaves_ - 1; // 2**d - 1
  
  // Initialize probabilities and counts to 0
  posteriors_.resize(classes_ * num_leaves_);
  leaf_counts_.resize(num_leaves_);

  createNodes(num_nodes, rng);
}

//void RandomizedTree::addExample(int class_id, cv::WImageView1_b const& patch)
void RandomizedTree::addExample(int class_id, IplImage* patch)
{
  int index = getIndex(patch);
  float* posterior = getPosteriorByIndex(index);
  ++leaf_counts_[index];
  ++posterior[class_id];
}

void RandomizedTree::finalize()
{
  // Normalize by number of patches to reach each leaf
  float* posterior = &posteriors_[0];
  for (int index = 0; index < num_leaves_; ++index) {
    int count = leaf_counts_[index];
    if (count != 0) {
      float normalizer = 1.0f / count;
      for (int c = 0; c < classes_; ++c) {
        *posterior *= normalizer;
        ++posterior;
      }
    } else {
      posterior += classes_;
    }
  }

  leaf_counts_.clear();
}

//float* RandomizedTree::getPosterior(cv::WImageView1_b const& patch)
float* RandomizedTree::getPosterior(IplImage* patch)
{
  return const_cast<float*>(const_cast<const RandomizedTree*>(this)->getPosterior(patch));
}

//const float* RandomizedTree::getPosterior(cv::WImageView1_b const& patch) const
const float* RandomizedTree::getPosterior(IplImage* patch) const
{
  return getPosteriorByIndex( getIndex(patch) );
}

// TODO: error-checking
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

  posteriors_.resize(classes_ * num_leaves_);
  is.read((char*)(&posteriors_[0]), posteriors_.size() * sizeof(posteriors_[0]));
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
  os.write((char*)(&posteriors_[0]), posteriors_.size() * sizeof(posteriors_[0]));
}

} // namespace features
