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
#include <boost/bind.hpp>
#include <ros/console.h>
#include <ros/assert.h>
#include <fstream>
#include <visual_nav/roadmap_impl.h>


namespace visual_nav
{

using std::vector;
using std::string;
using std::ifstream;
using boost::format;
using boost::weight_map;
using boost::get;
using boost::associative_property_map;
using boost::bind;

/************************************************************
 * Internal
 ************************************************************/

bool isNeighbor (const RoadmapVertex v, const RoadmapVertex w, const RoadmapGraph& g)
{
  AdjIterPair iter_pair = adjacent_vertices(v,g);
  return (iter_pair.second != find(iter_pair.first, iter_pair.second, w));
}

RoadmapEdge getEdgeBetween (const RoadmapVertex v, const RoadmapVertex w, const RoadmapGraph& g)
{
  pair<RoadmapEdge, bool> result = edge(v,w,g);
  ROS_ASSERT_MSG (result.second, "Tried to find edge between non-neighbors %d and %d", g[v].index, g[w].index);
  return result.first;
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


bool VisualNavRoadmap::RoadmapImpl::distanceLessThan (const Pose& pose, NodeId id, double r) const
{
  idVertex(id);
  NodeInfo n(graph_[idVertex(id)]);
  double dist = n.start_node ? 0 : distance(pose, n.getPose());
  ROS_DEBUG_STREAM_NAMED ("path_exit", "Distance to vertex " << id << " is " << dist);
  return dist<r;
}


Pose VisualNavRoadmap::RoadmapImpl::estimatedPose (PathPtr p) const
{
  if (p->size()<2) {
    throw InsufficientlyLongPathException();
  }
  NodeId first=(*p)[0];
  if (first!=0) {
    throw InvalidPathException(first);
  }

  RoadmapVertex second=idVertex((*p)[1]);
  RoadmapEdge edge = getEdgeBetween(idVertex(first), second, graph_);
  return transform(inverse(graph_[edge].getRelPose()), graph_[second].getPose());
}
  


Pose VisualNavRoadmap::RoadmapImpl::pathExitPoint (PathPtr p, double r) const
{
  Pose start_pose = estimatedPose(p);

  Path::reverse_iterator pos = find_if(p->rbegin(), p->rend(), bind(&VisualNavRoadmap::RoadmapImpl::distanceLessThan, this, start_pose, _1, r));
  
  if (pos==p->rbegin()) {
    throw InsufficientlyLongPathException();
  }
  ROS_ASSERT_MSG (pos!=p->rend(), "Unexpectedly couldn't find any points on path within radius %f", r);
  return graph_[idVertex(*(--pos))].getPose();
}


uint VisualNavRoadmap::RoadmapImpl::numNodes () const
{
  return num_vertices(graph_);
}

Pose VisualNavRoadmap::RoadmapImpl::nodePose (NodeId i) const
{
  return graph_[idVertex(i)].getPose();
}

vector<NodeId> VisualNavRoadmap::RoadmapImpl::neighbors (NodeId i) const
{
  vector<NodeId> neighbors;
  return neighbors;
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

Pose VisualNavRoadmap::pathExitPoint (PathPtr p, double r) const
{
  return roadmap_impl_->pathExitPoint(p,r);
}

Pose VisualNavRoadmap::estimatedPose (PathPtr p) const
{
  return roadmap_impl_->estimatedPose(p);
}

uint VisualNavRoadmap::numNodes () const
{
  return roadmap_impl_->numNodes();
}

Pose VisualNavRoadmap::nodePose (NodeId i) const
{
  return roadmap_impl_->nodePose (i);
}

vector<NodeId> VisualNavRoadmap::neighbors (NodeId i) const 
{
  return roadmap_impl_->neighbors(i);
}


/************************************************************
 * Reading from file
 ************************************************************/

RoadmapPtr readRoadmapFromFile (const string& filename)
{
  ifstream stream(filename.c_str());
  if (stream.fail()) {
    throw ReadRoadmapException(filename);
  }
  
  int num_nodes;
  stream >> num_nodes;
  RoadmapPtr roadmap(new VisualNavRoadmap);
  ROS_DEBUG_STREAM_NAMED ("file", "Reading roadmap with " << num_nodes << " nodes from " << filename);

  for (int i=0; i<num_nodes; ++i) {
    Pose pose;
    uint num_edges;
    stream >> pose;
    ROS_DEBUG_STREAM_NAMED ("file", " Adding node " << 1+i);
    ROS_ASSERT(roadmap->addNode(pose)==1+i);
    stream >> num_edges;
    for (uint j=0; j<num_edges; ++j) {
      NodeId neighbor;
      stream >> neighbor;
      roadmap->addEdge (1+i, neighbor);
      ROS_DEBUG_STREAM_NAMED ("file", " Adding edge between " << 1+i << " and " << neighbor);
    }
  }
  
  ROS_DEBUG_NAMED ("file", "Reading edges from start node");
  uint num_edges_from_start;
  stream >> num_edges_from_start;
  for (uint i=0; i<num_edges_from_start; ++i) {
    uint id;
    Transform2D relative_pose;
    stream >> id;
    stream >> relative_pose;
    ROS_DEBUG_STREAM_NAMED ("file", "Adding edge to " << id << " with relative pose " << relative_pose);
    roadmap->addEdgeFromStart (id, relative_pose);
  }
  ROS_DEBUG_NAMED ("file", "Done reading");
  return roadmap;
}

} // visual_nav
