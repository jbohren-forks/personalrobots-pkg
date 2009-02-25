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
 * Internally used data structure for a graph over regions (sets of grid cells)
 */

#ifndef TOPOLOGICAL_MAP_REGION_GRAPH_H
#define TOPOLOGICAL_MAP_REGION_GRAPH_H

#include <topological_map/topological_map.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

namespace topological_map
{

using boost::listS;
using boost::undirectedS;
using boost::adjacency_list;
using boost::graph_traits;
using std::map;

struct RegionInfo
{
  int type;
  RegionPtr region;
  RegionId id;
  
  RegionInfo ();
  RegionInfo (const int region_type, const RegionPtr region_cells, const RegionId region_id) :
    type(region_type), region(region_cells), id(region_id) {}
};

typedef adjacency_list<listS, listS, undirectedS, RegionInfo> RegionGraphImpl;
typedef graph_traits<RegionGraphImpl>::vertex_descriptor RegionGraphVertex;
typedef graph_traits<RegionGraphImpl>::adjacency_iterator AdjacencyIterator;
typedef map<Cell2D, RegionId> RegionMap;
typedef map<RegionId, RegionGraphVertex> IdVertexMap;


/// Internally used graph that wraps a boost graph over regions, together with some
/// indexing data structures
class RegionGraph
{
public:

  RegionGraph() : next_id_(1) {}

  RegionId containingRegion(const Cell2D& cell) const;

  int regionType(RegionId id) const;

  RegionPtr regionCells(RegionId id) const;

  RegionIdVector neighbors(RegionId id) const;

  const RegionIdVector& allRegions() const;

  RegionId addRegion (RegionPtr region, int region_type);

  void removeRegion (RegionId id);
  
private:
  
  // Disallow copy and assign
  RegionGraph(const RegionGraph&);
  RegionGraph& operator= (const RegionGraph&);

  RegionGraphVertex idVertex(RegionId id) const;

  // Map from region id to graph vertex
  IdVertexMap id_vertex_map_;

  // Map from cell to region id
  RegionMap region_map_;

  // The actual boost graph
  RegionGraphImpl graph_;

  // Sorted vector of existing region ids
  RegionIdVector regions_;

  // Id of the next region that will be added
  RegionId next_id_;

};

} // namespace

#endif
