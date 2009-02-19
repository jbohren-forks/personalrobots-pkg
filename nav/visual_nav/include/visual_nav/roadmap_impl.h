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

#ifndef VISUAL_NAV_ROADMAP_IMPL_H
#define VISUAL_NAV_ROADMAP_IMPL_H

#include "visual_nav.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <map>
#include <visual_nav/exceptions.h>

namespace visual_nav
{

using boost::listS;
using boost::undirectedS;
using boost::adjacency_list;
using boost::graph_traits;
using std::map;
using std::pair;
using std::vector;


typedef unsigned int uint;
typedef vector<int> Path; // Note dependency between this and PathPtr

/************************************************************
 * RoadmapGraph
 ************************************************************/

struct NodeInfo
{
public:
  NodeInfo(const Pose pose, const NodeId index) : start_node(false), index(index), pose(pose) {}
  NodeInfo(const NodeId index) : start_node(true), index(index) {}

  Pose getPose() const { if (start_node) throw StartNodePoseException(); else return pose; }

  // Flag for whether this is the start node (the only one that doesn't have an associated 2d pose)
  bool start_node;

  NodeId index;

private:
  // Only meaningful if start_node is false
  Pose pose;

};

class EdgeInfo
{
public:
  EdgeInfo(const Pose& p1, const Pose& p2);
  EdgeInfo(const Transform2D& rel_pose);

  Transform2D getRelPose() const { if (edge_from_start_node) return rel_pose; else throw NonstartRelPoseException(); }

  // Is this an edge from the start node?
  bool edge_from_start_node;
  
  double length;

private:
  // Only meaningful if edge_from_start_node is true
  Transform2D rel_pose;
};

typedef adjacency_list<listS, listS, undirectedS, NodeInfo, EdgeInfo> RoadmapGraph;
typedef graph_traits<RoadmapGraph>::vertex_descriptor RoadmapVertex;
typedef graph_traits<RoadmapGraph>::edge_descriptor RoadmapEdge;
typedef graph_traits<RoadmapGraph>::adjacency_iterator AdjacencyIterator;
typedef pair<AdjacencyIterator, AdjacencyIterator> AdjIterPair;

typedef map<NodeId, RoadmapVertex> IdVertexMap;


/************************************************************
 * Visitor for Dijkstra's algorithm
 ************************************************************/

/// \todo make the dijkstra visitor exit early when goal is discovered
class DijkstraVisitor : public boost::default_dijkstra_visitor
{
public:
  DijkstraVisitor (const RoadmapVertex& goal) : goal_(goal) {}
  void discover_vertex(const RoadmapVertex& v, const RoadmapGraph& graph) const
  {
    if (graph[v].start_node) {
      ROS_DEBUG_NAMED("dijkstra", "Discovering start node %d", graph[v].index);
    }
    else {
      ROS_DEBUG_NAMED("dijkstra", "Discovering vertex %d with coords (%f, %f)", graph[v].index, graph[v].getPose().x, graph[v].getPose().y);
    }
  }
private:
  RoadmapVertex goal_;
};



/************************************************************
 * RoadmapImpl
 ************************************************************/

class VisualNavRoadmap::RoadmapImpl
{
public:
  RoadmapImpl();

  NodeId addNode (const Pose& pos);
  void addEdge (const NodeId i, const NodeId j);
  void addEdgeFromStart (const NodeId i, const Transform2D& relative_pos);
  PathPtr pathToGoal (const NodeId goal_id);
  Pose pathExitPoint (PathPtr p, double r) const;
  Pose estimatedPose (PathPtr p) const;
  
  bool distanceLessThan (const Pose& pose, NodeId id, double r) const;

private:

  RoadmapVertex idVertex (const NodeId id) const;

  RoadmapGraph graph_;

  IdVertexMap id_vertex_map_;

  NodeId next_node_id_;
  NodeId start_node_id_;

  RoadmapImpl& operator= (const RoadmapImpl&);
  RoadmapImpl(const RoadmapImpl&);
};




} // namespace visual_nav

#endif
