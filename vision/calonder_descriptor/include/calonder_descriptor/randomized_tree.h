#ifndef FEATURES_RANDOMIZED_TREE_H
#define FEATURES_RANDOMIZED_TREE_H

#include "calonder_descriptor/rng.h"
#include "calonder_descriptor/patch_generator.h"
#include <cvwimage.h>
#include <vector>
#include <iostream>

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

class RandomizedTree
{
public:
  static const int PATCH_SIZE = 32;
  static const int DEFAULT_DEPTH = 10;
  static const int DEFAULT_VIEWS = 5000;

  RandomizedTree();
  
  void train(std::vector<BaseKeypoint> const& base_set, Rng &rng,
             int depth = RandomizedTree::DEFAULT_DEPTH,
             int views = RandomizedTree::DEFAULT_VIEWS);
  void train(std::vector<BaseKeypoint> const& base_set, Rng &rng,
             PatchGenerator &make_patch,
             int depth = RandomizedTree::DEFAULT_DEPTH,
             int views = RandomizedTree::DEFAULT_VIEWS);
  
  // TODO: uBLASify?
  float* getPosterior(IplImage* patch);
  const float* getPosterior(IplImage* patch) const;

  void read(const char* file_name);
  void read(std::istream &is);
  void write(const char* file_name) const;
  void write(std::ostream &os) const;

  int classes() { return classes_; }
  int depth() { return depth_; }
  
private:
  int classes_;
  int depth_;
  int num_leaves_;
  std::vector<RTreeNode> nodes_;
  std::vector<float> posteriors_;
  std::vector<int> leaf_counts_;

  void createNodes(int num_nodes, Rng &rng);
  void init(int classes, int depth, Rng &rng);
  void addExample(int class_id, IplImage* patch);
  void finalize();
  int getIndex(IplImage* patch) const;
  float* getPosteriorByIndex(int index);
  const float* getPosteriorByIndex(int index) const;
};

template < typename PointT >
cv::WImageView1_b extractPatch(cv::WImageView1_b const& image, PointT pt)
{
  static const int offset = RandomizedTree::PATCH_SIZE / 2;

  // TODO: WImage{C}.View really should have const version
  cv::WImageView1_b &img_ref = const_cast< cv::WImageView1_b& >(image);
  return img_ref.View(pt.x - offset, pt.y - offset,
                      RandomizedTree::PATCH_SIZE, RandomizedTree::PATCH_SIZE);
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

// TODO: use single offset to pixel value? (need to know widthstep)
struct RTreeNode
{
  uchar x1, y1, x2, y2;

  RTreeNode() {}
  
  RTreeNode(uchar x1, uchar y1, uchar x2, uchar y2)
    : x1(x1), y1(y1), x2(x2), y2(y2)
  {}

  //! Left child on 0, right child on 1
  inline bool operator() (IplImage* patch) const
  {
    return CV_IMAGE_ELEM(patch, uchar, y1, x1) > CV_IMAGE_ELEM(patch, uchar, y2, x2);
  }
};

} // namespace features

#endif
