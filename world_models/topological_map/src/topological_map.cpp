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
 * Implements the basic topological_map api
 *
 * \author Bhaskara Marthi
 */

#include <topological_map/topological_map_graph.h>
#include <algorithm>
#include <topological_map/exception.h>

// For debugging
#include <iostream>
using std::cout;
using std::endl;


using boost::tie;
using std::map;
using std::vector;
using std::set;

namespace topological_map
{

RegionId TopologicalMap::GraphImpl::containingRegion (const Cell2D& p) const
{
  RegionMap::const_iterator pos=region_map_.find(p);
  if (pos!=region_map_.end()) {
    return pos->second;
  }
  else {
    throw UnknownGridCellException(p);
  }
}


int TopologicalMap::GraphImpl::regionType (const RegionId id) const
{
  return graph_[idVertex(id)].type;
}


// Functor for mapping neighbor vertices to ids
struct GetNeighborId
{
  GetNeighborId(const TopologicalGraph& graph) : g(graph) {}
  RegionId operator() (const TopologicalGraphVertex& v) { return g[v].id; }
  const TopologicalGraph& g;
};

RegionIdVector TopologicalMap::GraphImpl::neighbors (const RegionId id) const
{
  AdjacencyIterator v, v_end;
  tie(v,v_end) = adjacent_vertices(idVertex(id), graph_);
  RegionIdVector neighbors;
  transform (v, v_end, back_inserter(neighbors), GetNeighborId(graph_));
  return neighbors;
}

const RegionIdVector& TopologicalMap::GraphImpl::allRegions () const
{
  return regions_;
}


RegionId TopologicalMap::GraphImpl::addRegion (const RegionPtr region, const int type)
{

  // Make sure there's no overlap with existing regions
  for (Region::iterator region_iter=region->begin(); region_iter!=region->end(); ++region_iter) {
    if (region_map_.find(*region_iter)!=region_map_.end()) {
      throw OverlappingRegionException(*region_iter, region_map_.find(*region_iter)->second);
    }
  }

  // maintains invariant that regions_ is sorted
  regions_.push_back(next_id_);

  // Add vertex and edges
  TopologicalGraphVertex v=add_vertex(RegionInfo(type, region, next_id_), graph_);

  // Neighbors that have been added so far
  set<TopologicalGraphVertex> seen_neighbors;

  // Update the region index and add edges to neighboring regions if necessary
  for (Region::iterator region_iter=region->begin(); region_iter!=region->end(); ++region_iter) {
    region_map_[*region_iter]=next_id_;
    vector<Cell2D> cell_neighbors=cellNeighbors(*region_iter);
    for (vector<Cell2D>::iterator neighbor_iter=cell_neighbors.begin(); neighbor_iter!=cell_neighbors.end(); ++neighbor_iter) {

      RegionMap::iterator neighbor_ptr=region_map_.find(*neighbor_iter);
      if (neighbor_ptr!=region_map_.end() && neighbor_ptr->second!=next_id_) {
        TopologicalGraphVertex neighbor_vertex=idVertex(neighbor_ptr->second);
        if (seen_neighbors.find(neighbor_vertex)==seen_neighbors.end()) {
          seen_neighbors.insert(neighbor_vertex);
          add_edge(v, neighbor_vertex, graph_);
        }   
      }
    }
    id_vertex_map_[next_id_]=v; 
  }
  
  return next_id_++;
}



struct RemoveFromMap
{
  RemoveFromMap(RegionMap& m) : region_map(m) {}
  void operator() (const Cell2D& c) { region_map.erase(region_map.find(c)); }
  RegionMap& region_map;
};

void TopologicalMap::GraphImpl::removeRegion (const RegionId id)
{
  TopologicalGraphVertex v=idVertex(id);

  // Remove from regions_ and id_vertex_map - we don't do any check as it must exist if we got here without an exception
  regions_.erase(lower_bound(regions_.begin(), regions_.end(), id));
  id_vertex_map_.erase(id_vertex_map_.find(id));

  // Remove all cells in this region from the region map
  for_each(graph_[v].region->begin(), graph_[v].region->end(), RemoveFromMap(region_map_));

  // Remove the vertex
  clear_vertex(v, graph_);
  remove_vertex(v, graph_);
}


TopologicalGraphVertex TopologicalMap::GraphImpl::idVertex(const RegionId id) const
{
  IdVertexMap::const_iterator map_iter=id_vertex_map_.find(id);
  if (map_iter != id_vertex_map_.end()) {
    return map_iter->second;
  }
  else {
    throw UnknownRegionException(id);
  }
}



/************************************************************
 * Topological graph ops are forwarded to implementation
 ************************************************************/

TopologicalMap::TopologicalMap (double resolution) : graph_impl_(new GraphImpl) 
{
}

RegionId TopologicalMap::containingRegion (const Cell2D& p) const
{
  return graph_impl_->containingRegion(p);
}

RegionId TopologicalMap::containingRegion (const Point2D& p) const
{
  return 42;
}

int TopologicalMap::regionType (const RegionId id) const
{
  return graph_impl_->regionType(id);
}


RegionIdVector TopologicalMap::neighbors (const RegionId id) const
{
  return graph_impl_->neighbors(id);
}

// Add new vertex with new unique id, and do bookkeeping
RegionId TopologicalMap::addRegion(const RegionPtr region, const int type) 
{
  return graph_impl_->addRegion(region, type);
}

void TopologicalMap::removeRegion (const RegionId id)
{
  graph_impl_->removeRegion (id);
}

const RegionIdVector& TopologicalMap::allRegions () const
{
  return graph_impl_->allRegions();
}



} // namespace topological_map
