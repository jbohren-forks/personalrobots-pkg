#include "lifelong_mapping/pose_graph.h"

namespace lifelong_mapping {

PoseGraph::PoseGraph()
  : next_node_id_(0)
{
  graph_.verboseLevel = 0;
  graph_.restartOnDivergence = false;
}

void PoseGraph::initializeOptimization()
{
  graph_.initializeOnlineOptimization();
}

uint32_t PoseGraph::addFreeNode()
{
  return next_node_id_++;
}

void PoseGraph::addConstraint(uint32_t parent, uint32_t child, const tf::Transform& t)
{
  // @todo: keep track of affected EdgeSet?
  // @todo: stop using made-up covariances
  static const double XY_INF    = 1.0 / 0.01;
  static const double Z_INF     = 1.0 / 0.01;
  static const double ROLL_INF  = 1.0 / 0.0002;
  static const double PITCH_INF = 1.0 / 0.0002;
  static const double YAW_INF   = 1.0 / 0.0002;

  Information information(6, 6);
  information[0][0] = XY_INF;
  information[1][1] = XY_INF;
  information[2][2] = Z_INF;
  information[3][3] = ROLL_INF;
  information[4][4] = PITCH_INF;
  information[5][5] = YAW_INF;

  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw, pitch, roll);
  Transformation transform(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(),
                           yaw, pitch, roll);
  
  graph_.addIncrementalEdge(parent, child, transform, information);
}

void PoseGraph::optimize()
{
  // @todo: #iterations? timeout?
  static const unsigned ITERATIONS = 100;
  
  graph_.initializeOnlineIterations();
  unsigned i = 0;
  while (i < ITERATIONS) {
    graph_.iterate(); // uses preconditioner
    ++i;
  }
  graph_.recomputeAllTransformations();
}

bool PoseGraph::save(const std::string& file_name)
{
  return graph_.save(file_name.c_str());
}

bool PoseGraph::load(const std::string& file_name)
{
  // @todo: set next_node_id_
  return graph_.load(file_name.c_str());
}

} //namespace lifelong_mapping
