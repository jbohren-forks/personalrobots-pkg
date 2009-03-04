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
 * Internally used data structure for a roadmap over 2d points
 *
 * \author Bhaskara Marthi
 */


#ifndef TOPOLOGICAL_MAP_ROADMAP_H
#define TOPOLOGICAL_MAP_ROADMAP_H

#include <topological_map/topological_map.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

namespace topological_map {

using boost::listS;
using boost::undirectedS;
using boost::adjacency_list;
using boost::graph_traits;
using std::map;
using std::vector;


struct NodeInfo
{
  NodeInfo(ConnectorId id, const Point2D& point) : id(id), point(point) {}
  ConnectorId id; // Stable id 
  uint index; // Not stable; used internally by dijkstra search
  Point2D point;
};

struct EdgeInfo
{
  double cost;
};

typedef adjacency_list<listS, listS, undirectedS, NodeInfo, EdgeInfo> RoadmapImpl;
typedef graph_traits<RoadmapImpl>::vertex_descriptor RoadmapVertex;
typedef graph_traits<RoadmapImpl>::edge_descriptor RoadmapEdge;
typedef graph_traits<RoadmapImpl>::adjacency_iterator RoadmapAdjacencyIterator;
typedef graph_traits<RoadmapImpl>::vertex_iterator VertexIterator;
typedef map<ConnectorId, RoadmapVertex> ConnectorIdVertexMap;

typedef vector<ConnectorId> ConnectorIdVector;

/// Internally used graph that wraps a boost graph over connectors (and also the start and end
/// point of a given navigation problem)
class Roadmap
{
public:
  
  Roadmap() : next_id_(1) {}

  Roadmap(istream& str);

  void writeToStream (ostream& stream) const;

  ConnectorId addNode (const Point2D& p);
  void setCost (ConnectorId i, ConnectorId j, double cost);
  void removeNode (ConnectorId i);

  ConnectorId pointId (const Point2D& p) const;
  Point2D connectorPoint (const ConnectorId id) const;

  pair<bool, double> costBetween (ConnectorId i, ConnectorId j);
  ConnectorIdVector shortestPath (ConnectorId i, ConnectorId j);

private:

  // Disallow copy and assign
  Roadmap (const Roadmap&);
  Roadmap& operator= (const Roadmap&);

  RoadmapEdge ensureEdge(RoadmapVertex v, RoadmapVertex w);
  RoadmapVertex idVertex(ConnectorId i) const;
  bool idExists(ConnectorId i) const;
  void resetIndices();
  void addNode (const Point2D& p, ConnectorId id);

  ConnectorId next_id_;
  ConnectorIdVertexMap id_vertex_map_;
  RoadmapImpl graph_;

};

} // namespace

#endif



