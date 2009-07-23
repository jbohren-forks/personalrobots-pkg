#ifndef LIFELONG_MAPPING_POSE_GRAPH_H
#define LIFELONG_MAPPING_POSE_GRAPH_H

#include <utility>
using std::make_pair; // get TORO to compile
#include <treeoptimizer3.hh>
#include <tf/transform_datatypes.h>

namespace lifelong_mapping {

class PoseGraph
{
public:
  PoseGraph();

  void initializeOptimization();

  // Graph updates
  uint32_t addFreeNode();
  // @todo: need covariance parameter
  void addConstraint(uint32_t parent, uint32_t child, const tf::Transform& t);

  void optimize(); // #iterations? timeout?

  // Spatial queries

  // File I/O
  bool save(const std::string& file_name);
  bool load(const std::string& file_name);

protected:
  typedef AISNavigation::TreeOptimizer3::Transformation Transformation;
  typedef AISNavigation::TreeOptimizer3::Covariance Covariance;
  typedef AISNavigation::TreeOptimizer3::Information Information;
  typedef AISNavigation::TreeOptimizer3::Translation Translation;
  typedef AISNavigation::TreeOptimizer3::Rotation Rotation;
  typedef AISNavigation::TreeOptimizer3::Pose Pose;
  typedef AISNavigation::TreeOptimizer3::Vertex Vertex;
  typedef AISNavigation::TreeOptimizer3::Edge Edge;

  AISNavigation::TreeOptimizer3 graph_;
  uint32_t next_node_id_;
};

} //namespace lifelong_mapping

#endif
