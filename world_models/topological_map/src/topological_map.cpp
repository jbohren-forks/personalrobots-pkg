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


#include "topological_map/topological_map_graph.h"

using boost::tie;

namespace topological_map
{

RegionId TopologicalMap::containingRegion (const Cell2D& p) const
{
  if ((RegionMap::iterator pos = region_map_.find(p)) != region_map_.end()) {
    return pos->second;
  }
  else {
    throw UnknownGridCellException(p);
  }
}

int RegionType (const RegionId id) const
{
  return graph_[idVertex(id)].type;
}


/// Look up the vertex, get the neighbor list from the graph, then get their ids
RegionIdVector neighbors (const RegionId id) const
{
  // Functor for mapping neighbor vertices to ids
  struct GetNeighborId
  {
    GetNeighborId(const TopologicalGraph& graph) : g(graph) {}
    RegionId operator() (const TopologicalGraphVertex& v) { return g[v].id; }
    const TopologicalGraph& g;
  };


  AdjacencyIterator v, v_end;
  tie(v,v_end) = adjacent_vertices(idVertex(id), graph_);
  RegionIdVector neighbors;
  transform (v, v_end, back_inserter(neighbors), GetNeighborId(graph_));
  return neighbors;
}


// Add new vertex with new unique id, and do bookkeeping
RegionId addRegion(const RegionPtr region, const int type) 
{
  // Functor for updating region map, checking overlap, and adding edges as appropriate
  struct UpdateRegionMap 
  {
    UpdateRegionMap (RegionMap& m, RegionId i) : region_map(m), id(i) {}
    void operator() (const Cell2D& c)
    {
      if (m.find(c)!=m.end()) {
        throw OverlappingRegionException(c, m.find(c)->second);
      }
      else {
        m[c]=id;
        vector<Cell
      }
    }
    RegionMap& region_map;
    RegionId id;
    vector<BottleneckVertex> neighboring_regions;
  };
  

  RegionId new_id=next_id_++;
  for_each(region->begin(), region->end(), UpdateRegionMap(region_map_, new_id));
  TopologicalGraphVertex v=add_vertex(RegionInfo(type, region, new_id), graph_);
  id_vertex_map_[new_id] = v;
  return new_id;
}



void removeRegion (const RegionId id)
{
  
  // Iterate over neighbors and remove the edge
  // Remove the region info
  // Update id vertex map
  // Update region map
}



bool knownGridCell (const Cell2D& p) const
{
  return region_map_.find(p) != region_map_.end();
}

TopologicalGraphVertex idVertex (const RegionId id) const
{
  if ((IdVertexMap::iterator pos=id_vertex_map_.find(id)) != id_vertex_map_.end()) {
    return pos->second;
  }
  else {
    throw UnknownRegionException(id);
  }
}
  




} // namespace topological_map
