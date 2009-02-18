/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <ros/console.h>
#include <visual_nav/roadmap_impl.h>


namespace visual_nav
{

using std::vector;
using boost::format;
using boost::weight_map;
using boost::get;
using boost::associative_property_map;


/************************************************************
 * Internal
 ************************************************************/

bool isNeighbor (const RoadmapVertex v, const RoadmapVertex w, const RoadmapGraph& g)
{
  AdjIterPair iter_pair = adjacent_vertices(v,g);
  return (iter_pair.second != find(iter_pair.first, iter_pair.second, w));
}
   

inline double distance (const Pose& pose1, const Pose& pose2)
{
  double dx=pose1.x-pose2.x;
  double dy=pose1.y-pose2.y;
  return sqrt(dx*dx+dy*dy);
}

inline double relPoseLength (const Transform2D& rel_pose)
{
  return sqrt(rel_pose.dx*rel_pose.dx+rel_pose.dy*rel_pose.dy);
}



EdgeInfo::EdgeInfo(const Pose& p1, const Pose& p2) : edge_from_start_node(false), length(distance(p1,p2))
{}

EdgeInfo::EdgeInfo(const Transform2D& rel_pose) : edge_from_start_node(true), length(relPoseLength(rel_pose)), rel_pose(rel_pose) 
{}





/************************************************************
 * RoadmapImpl ops
 ************************************************************/

VisualNavRoadmap::RoadmapImpl::RoadmapImpl ()
{
  start_node_id_=0;
  next_node_id_=1;
  id_vertex_map_[start_node_id_] = add_vertex(NodeInfo(start_node_id_), graph_);
}

NodeId VisualNavRoadmap::RoadmapImpl::addNode (const Pose& pose)
{
  id_vertex_map_[next_node_id_] = add_vertex(NodeInfo(pose, next_node_id_), graph_);
  return next_node_id_++;
}

void VisualNavRoadmap::RoadmapImpl::addEdge (const NodeId i, const NodeId j)
{
  // Lookup nodes
  RoadmapVertex v = idVertex(i);
  RoadmapVertex w = idVertex(j);

  // Error checks
  if (i==j) {
    throw SelfEdgeException(i);
  }
  else if (isNeighbor(v, w, graph_)) {
    throw ExistingEdgeException(i,j);
  }
  else if (i==start_node_id_) {
    throw StartEdgeException(j);
  }
  else if (j==start_node_id_) {
    throw StartEdgeException(i);
  }

  // Add
  add_edge (v, w, EdgeInfo(graph_[v].getPose(), graph_[w].getPose()), graph_);
}

void VisualNavRoadmap::RoadmapImpl::addEdgeFromStart (const NodeId i, const Transform2D& rel_pose)
{
  // Lookup node
  RoadmapVertex v = idVertex(i);
  RoadmapVertex start = idVertex(start_node_id_);
  
  // Error checks
  if (i==start_node_id_) {
    throw SelfEdgeException(i);
  }
  else if (isNeighbor(start, v, graph_)) {
    throw ExistingEdgeException(start_node_id_, i);
  }

  // Add
  add_edge (start, v, EdgeInfo(rel_pose), graph_);
}
  


struct TransformVertexToId
{
  TransformVertexToId(const RoadmapGraph& graph) : graph(graph) {}
  NodeId operator() (const RoadmapVertex& v) { return graph[v].index; }
  const RoadmapGraph& graph;
};

PathPtr VisualNavRoadmap::RoadmapImpl::pathToGoal (const NodeId goal_id) 
{
  typedef map<RoadmapVertex, RoadmapVertex> PredecessorMap;

  // Set up and call dijkstra_shortest_paths
  // For speed, could add a visitor to dijkstra that terminates when goal is found
  RoadmapVertex goal = idVertex(goal_id);
  RoadmapVertex start = idVertex(start_node_id_);
  PredecessorMap predecessor_map;
  associative_property_map<PredecessorMap> adapted_pred_map(predecessor_map);
  dijkstra_shortest_paths(graph_, start, weight_map(get(&EdgeInfo::length, graph_))
                          .vertex_index_map(get(&NodeInfo::index, graph_))
                          .predecessor_map(adapted_pred_map).visitor(DijkstraVisitor(goal)));
  
  // Extract path from predecessor map
  vector<RoadmapVertex> reverse_path(1,goal);
  RoadmapVertex current = goal;

  while (current!=predecessor_map[current]) {
    ROS_DEBUG_NAMED("dijkstra", "Predecessor of %d is %d", graph_[current].index, graph_[predecessor_map[current]].index);
    reverse_path.push_back(current=predecessor_map[current]);
  }
  if (current!=start){
      throw UnreachableGoalException(goal_id);
  }

  // Put it in source-to-goal order, replace vertex descriptors with ids, and return
  PathPtr path(new vector<int>(reverse_path.size()));
  transform(reverse_path.rbegin(), reverse_path.rend(), path->begin(), TransformVertexToId(graph_));
  return path;
}

                          


/************************************************************
 * RoadmapImpl internal
 ************************************************************/



RoadmapVertex VisualNavRoadmap::RoadmapImpl::idVertex(const NodeId id) const
{
  IdVertexMap::const_iterator pos = id_vertex_map_.find(id);
  if (pos == id_vertex_map_.end()) {
    throw UnknownNodeIdException (id);
  }
  else {
    return pos->second;
  }
}


/************************************************************
 * VisualNavRoadmap api
 * Constructor just creates the implementation object
 * Other functions forward ops
 ************************************************************/

VisualNavRoadmap::VisualNavRoadmap() : roadmap_impl_(new RoadmapImpl) {}

NodeId VisualNavRoadmap::addNode (const Pose& pose)
{
  return roadmap_impl_->addNode(pose);
}

NodeId VisualNavRoadmap::addNode (double x, double y, double theta)
{
  return roadmap_impl_->addNode(Pose(x,y,theta));
}

void VisualNavRoadmap::addEdge (const NodeId i, const NodeId j)
{
  roadmap_impl_->addEdge(i, j);
}

void VisualNavRoadmap::addEdgeFromStart (const NodeId i, const Transform2D& relative_pose)
{
  roadmap_impl_->addEdgeFromStart(i, relative_pose);
}

PathPtr VisualNavRoadmap::pathToGoal (const NodeId goal_id) const
{
  return roadmap_impl_->pathToGoal(goal_id);
}


} // visual_nav
