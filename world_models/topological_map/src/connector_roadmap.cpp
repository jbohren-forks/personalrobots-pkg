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

/************************************************************
 * Roadmap modification
 ************************************************************/

ConnectorId Roadmap::addNode (const Point2D& p)
{
  add_vertex(NodeInfo(next_id_, p), graph_);
  return next_id_++;
}

void Roadmap::setCost (const ConnectorId i, const ConnectorId j, double cost)
{
  const RoadmapEdge e = ensureEdge(idVertex(i),idVertex(j));
  graph_[e].cost = cost;
}

void Roadmap::removeNode (const ConnectorId i)
{
  const RoadmapVertex v = idVertex(i);
  clear_vertex(v, graph_);
  remove_vertex(v, graph_);
}


/************************************************************
 * Shortest paths
 ************************************************************/

double Roadmap::costBetween (const ConnectorId i, const ConnectorId j)
{
  const RoadmapVertex v = idVertex(i);
  const RoadmapVertex w = idVertex(j);

  resetIndices();

  dijkstra_shortest_paths(graph_, start, weight_map(get(&EdgeInfo::cost, graph_)).
                          vertex_index_map(get(&NodeInfo::index, graph_)));

  // Need to extract cost
}
                         
ConnectorIdVector Roadmap::shortestPath (const ConnectorId i, const ConnectorId j)
{
  // shortest path with pred map
}


/************************************************************
 * internal
 ************************************************************/

RoadmapEdge Roadmap::ensureEdge(const RoadmapVertex v, const RoadmapVertex w);

RoadmapVertex Roadmap::idVertex(const ConnectorId i) const;

void Roadmap::resetIndices();


}











} // namespace
