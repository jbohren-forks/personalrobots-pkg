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
 *
 */


/**
 * \file
 *
 * Implements internally used Roadmap data structure
 *
 * \author Bhaskara Marthi
 */

#include <topological_map/roadmap.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <algorithm>
#include <ros/console.h>
#include <ros/assert.h>
#include <topological_map/exception.h>


namespace topological_map
{

using boost::tie;
using std::map;


typedef map<RoadmapVertex, double> DistanceMap;
typedef map<RoadmapVertex, RoadmapVertex> PredecessorMap;
typedef boost::associative_property_map<DistanceMap> DistancePmap;
typedef boost::associative_property_map<PredecessorMap> PredecessorPmap;


/************************************************************
 * Basic ops
 ************************************************************/

ConnectorId Roadmap::addNode (const Point2D& p)
{
  RoadmapVertex v=add_vertex(NodeInfo(next_id_, p), graph_);
  id_vertex_map_[next_id_]=v;
  ROS_DEBUG_STREAM_NAMED ("roadmap", "Added connector " << next_id_ << " at " << p);
  return next_id_++;
}

void Roadmap::setCost (const ConnectorId i, const ConnectorId j, double cost)
{
  const RoadmapEdge e = ensureEdge(idVertex(i),idVertex(j));
  graph_[e].cost = cost;
  ROS_DEBUG_STREAM_NAMED ("roadmap", "Set cost between connectors " << i << " and " << j << " to " << cost);
}

void Roadmap::removeNode (const ConnectorId i)
{
  const RoadmapVertex v = idVertex(i);
  clear_vertex(v, graph_);
  remove_vertex(v, graph_);
  id_vertex_map_.erase(id_vertex_map_.find(i));
  ROS_DEBUG_STREAM_NAMED ("roadmap", "Removed connector " << i);
}



struct CorrespondsToPoint
{
  CorrespondsToPoint (const RoadmapImpl& graph, const Point2D& point) : graph(graph), point(point) {}
  bool operator() (const RoadmapVertex& v) { return graph[v].point==point; }
  const RoadmapImpl& graph;
  const Point2D& point;
};

ConnectorId Roadmap::pointId (const Point2D& p) const
{
  VertexIterator iter, end;
  tie(iter, end) = vertices(graph_);
  VertexIterator pos = find_if(iter, end, CorrespondsToPoint(graph_, p));
  if (pos==end) {
    throw UnknownConnectorException(p.x, p.y);
  }
  return graph_[*pos].id;
}


struct CorrespondsToId 
{
  CorrespondsToId (const RoadmapImpl& graph, const ConnectorId id) : graph(graph), id(id) {}
  bool operator() (const RoadmapVertex& v) { return graph[v].id==id; }
  const RoadmapImpl& graph;
  const ConnectorId id;
};

Point2D Roadmap::connectorPoint (const ConnectorId id) const
{
  VertexIterator iter, end;
  tie (iter, end) = vertices(graph_);
  VertexIterator pos = find_if(iter, end, CorrespondsToId(graph_, id));
  if (pos==end) {
    throw UnknownConnectorIdException(id);
  }
  return graph_[*pos].point;
}


/************************************************************
 * Shortest paths
 ************************************************************/


/// \todo make the dijkstra visitor exit early when goal is discovered
struct DijkstraVisitor : public boost::default_dijkstra_visitor
{
  DijkstraVisitor (const RoadmapVertex& goal) : goal(goal) {}
  void discover_vertex(const RoadmapVertex& v, const RoadmapImpl& graph) const
  {
    ROS_DEBUG_STREAM_NAMED("dijkstra", "Discovering vertex " << graph[v].id << " at point " << graph[v].point);
  }
  RoadmapVertex goal;
};


double Roadmap::costBetween (const ConnectorId i, const ConnectorId j)
{
  const RoadmapVertex v = idVertex(i);
  const RoadmapVertex w = idVertex(j);

  resetIndices();
  DistanceMap distances;
  PredecessorMap predecessors; // We shouldn't actually need this, but adding it removes a warning about boost/dijkstra_shortest_paths.hpp...

  dijkstra_shortest_paths(graph_, v, weight_map(get(&EdgeInfo::cost, graph_)).
                          vertex_index_map(get(&NodeInfo::index, graph_)).
                          distance_map(DistancePmap(distances)).visitor(DijkstraVisitor(w)).
                          predecessor_map(PredecessorPmap(predecessors)));
  ROS_ASSERT_MSG(distances.find(w)!=distances.end(), "Couldn't find vertex %u in returned distance map from Dijkstra", j);
  return distances[w];
}


struct TransformVertexToId
{
  TransformVertexToId(const RoadmapImpl& graph) : graph(graph) {}
  ConnectorId operator() (const RoadmapVertex& v) { return graph[v].id; }
  const RoadmapImpl& graph;
};

                         
ConnectorIdVector Roadmap::shortestPath (const ConnectorId i, const ConnectorId j)
{
  const RoadmapVertex v = idVertex(i);
  const RoadmapVertex w = idVertex(j);

  resetIndices();
  PredecessorMap predecessors;

  dijkstra_shortest_paths(graph_, v, weight_map(get(&EdgeInfo::cost, graph_)).
                          vertex_index_map(get(&NodeInfo::index, graph_)).
                          predecessor_map(PredecessorPmap(predecessors)).
                          visitor(DijkstraVisitor(w)));
    


  // Extract path from predecessor map
  vector<RoadmapVertex> reverse_path(1,w);
  RoadmapVertex current = w;

  while (current!=predecessors[current]) {
    ROS_DEBUG_NAMED("dijkstra", "Predecessor of %d is %d", graph_[current].id, graph_[predecessors[current]].id);
    reverse_path.push_back(current=predecessors[current]);
  }
  if (current!=v){
    throw NoPathFoundException(i,j);
  }

  // Put it in source-to-goal order, replace vertex descriptors with ids, and return
  ConnectorIdVector path(reverse_path.size());
  transform(reverse_path.rbegin(), reverse_path.rend(), path.begin(), TransformVertexToId(graph_));
  return path;
}



/************************************************************
 * internal
 ************************************************************/

RoadmapEdge Roadmap::ensureEdge(const RoadmapVertex v, const RoadmapVertex w)
{
  pair<RoadmapEdge, bool> p = edge(v,w,graph_);
  if (!(p.second)) {
    add_edge(v,w,graph_);
    p = edge(v,w,graph_);
    ROS_ASSERT(p.second);
  }
  return p.first;
}

RoadmapVertex Roadmap::idVertex(const ConnectorId i) const
{
  ConnectorIdVertexMap::const_iterator iter = id_vertex_map_.find(i);
  if (iter==id_vertex_map_.end()) {
    throw UnknownConnectorIdException(i);
  }
  return iter->second;
}

void Roadmap::resetIndices()
{
  VertexIterator iter, end;
  uint ind=0;
  for (tie(iter,end) = vertices(graph_); iter!=end; ++iter) {
    graph_[*iter].index=ind++;
  }
}











} // namespace
