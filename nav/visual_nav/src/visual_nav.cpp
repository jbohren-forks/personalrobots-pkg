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

EdgeInfo::EdgeInfo (const Pose& p1, const Pose& p2) : length(distance(p1,p2))
{
}




/************************************************************
 * RoadmapImpl ops
 ************************************************************/

VisualNavRoadmap::RoadmapImpl::RoadmapImpl() : next_node_id_(0)
{
}

NodeId VisualNavRoadmap::RoadmapImpl::addNode (const Pose& pose)
{
  id_vertex_map_[next_node_id_] = add_vertex(NodeInfo(pose, next_node_id_), graph_);
  nodes_.push_back(next_node_id_);
  ROS_DEBUG_STREAM_NAMED("api", "Added node with pose " << pose << " and id " << next_node_id_);
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

  // Add
  add_edge (v, w, EdgeInfo(graph_[v].pose, graph_[w].pose), graph_);
  ROS_DEBUG_STREAM_NAMED("api", "Added edge between nodes " << i << " and " << j);
}


struct TransformVertexToId
{
  TransformVertexToId(const RoadmapGraph& graph) : graph(graph) {}
  NodeId operator() (const RoadmapVertex& v) { return graph[v].index; }
  const RoadmapGraph& graph;
};

PathPtr VisualNavRoadmap::RoadmapImpl::pathToGoal (const NodeId start_id, const NodeId goal_id) 
{
  typedef map<RoadmapVertex, RoadmapVertex> PredecessorMap;

  // Set up and call dijkstra_shortest_paths
  // For speed, could add a visitor to dijkstra that terminates when goal is found
  RoadmapVertex goal = idVertex(goal_id);
  RoadmapVertex start = idVertex(start_id);
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
  double dist = distance(pose, n.pose);
  ROS_DEBUG_STREAM_NAMED ("path_exit", "Distance to vertex " << id << " is " << dist);
  return dist<r;
}


Pose VisualNavRoadmap::RoadmapImpl::pathExitPoint (PathPtr p, double r) const
{
  ROS_ASSERT_MSG (p->size()>0, "Tried to find exit point of length-zero path");
  Pose start_pose = graph_[idVertex(*(p->begin()))].pose;
  if (p->size()==1) {
    return start_pose;
  }
  else {
    for (uint i=0; i<p->size(); i++) {
      ROS_DEBUG_STREAM_NAMED ("path_exit", "Path node " << i << " is " << (*p)[i]);
    }
    Path::reverse_iterator pos = find_if(p->rbegin()+1, p->rend(), bind(&VisualNavRoadmap::RoadmapImpl::distanceLessThan, this, start_pose, _1, r));
    ROS_ASSERT_MSG (pos!=p->rend(), "Unexpectedly couldn't find any points on path within radius %f", r);
    return graph_[idVertex(*(--pos))].pose;
  }
}


uint VisualNavRoadmap::RoadmapImpl::numNodes () const
{
  return num_vertices(graph_);
}

Pose VisualNavRoadmap::RoadmapImpl::nodePose (NodeId i) const
{
  return graph_[idVertex(i)].pose;
}



struct GetVertexId
{
  GetVertexId(const RoadmapGraph& graph) : graph(graph) {}
  NodeId operator() (const RoadmapVertex& v) { return graph[v].index; }
  const RoadmapGraph& graph;
};



NodeVector VisualNavRoadmap::RoadmapImpl::neighbors (NodeId i) const
{
  vector<NodeId> neighbors;
  AdjIterPair iters = adjacent_vertices(idVertex(i), graph_);
  transform (iters.first, iters.second, back_inserter(neighbors), GetVertexId(graph_));
  return neighbors;
}

NodeVector VisualNavRoadmap::RoadmapImpl::nodes () const
{
  return nodes_;
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

NodeId VisualNavRoadmap::addNode (const double x, const double y, const double theta)
{
  return roadmap_impl_->addNode(Pose(x,y,theta));
}

void VisualNavRoadmap::addEdge (const NodeId i, const NodeId j)
{
  roadmap_impl_->addEdge(i, j);
}

PathPtr VisualNavRoadmap::pathToGoal (const NodeId start_id, const NodeId goal_id) const
{
  return roadmap_impl_->pathToGoal(start_id, goal_id);
}

Pose VisualNavRoadmap::pathExitPoint (const PathPtr p, const double r) const
{
  return roadmap_impl_->pathExitPoint(p,r);
}

uint VisualNavRoadmap::numNodes () const
{
  return roadmap_impl_->numNodes();
}

Pose VisualNavRoadmap::nodePose (const NodeId i) const
{
  return roadmap_impl_->nodePose (i);
}

NodeVector VisualNavRoadmap::neighbors (const NodeId i) const 
{
  return roadmap_impl_->neighbors(i);
}

NodeVector VisualNavRoadmap::nodes () const
{
  return roadmap_impl_->nodes();
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
    ROS_DEBUG_STREAM_NAMED ("file", " Adding node " << i);
    ROS_ASSERT(roadmap->addNode(pose)==i);
    stream >> num_edges;
    for (uint j=0; j<num_edges; ++j) {
      NodeId neighbor;
      stream >> neighbor;
      roadmap->addEdge (i, neighbor);
      ROS_DEBUG_STREAM_NAMED ("file", " Adding edge between " << i << " and " << neighbor);
    }
  }
  
  ROS_DEBUG_NAMED ("file", "Done reading");
  return roadmap;
}

} // visual_nav
