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

using boost::listS;
using boost::undirectedS;
using boost::adjacency_list;
using boost::graph_traits;

namespace visual_nav
{


/************************************************************
 * RoadmapGraph typedefs
 ************************************************************/

struct NodeInfo
{
  // Flag for whether this is the start node (the only one that doesn't have an associated 2d position)
  bool start_node;
  // Only meaningful if start_node is false
  Position2D position;
};

struct EdgeInfo
{
  // Is this an edge from the start node?
  bool edge_from_start_node;
  // Only meaningful if edge_from_start_node is true
  Position2D offset;
};

typedef adjacency_list<listS, listS, undirectedS, NodeInfo, EdgeInfo> RoadmapGraph;
typedef graph_traits<VisualNavRoadmap>::vertex_descriptor RoadmapVertex;
typedef graph_traits<VisualNavRoadmap>::edge_descriptor RoadmapEdge;
typedef graph_traits<VisualNavRoadmap>::adjacency_iterator AdjacencyIterator;



/************************************************************
 * RoadmapImpl
 ************************************************************/

class RoadmapImpl
{
private:
  RoadmapGraph graph_;
};


} // namespace visual_nav

#endif
