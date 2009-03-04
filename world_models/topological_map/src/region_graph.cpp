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
 * Implements internally used RegionGraph data structure
 *
 * \author Bhaskara Marthi
 */


#include <topological_map/region_graph.h>
#include <algorithm>
#include <ros/console.h>
#include <ros/assert.h>
#include <topological_map/exception.h>


namespace topological_map
{

using std::set;
using std::map;
using std::vector;
using std::min;
using std::max;
//using std::lower_bound;
//using std::upper_bound;

RegionId RegionGraph::containingRegion (const Cell2D& p) const
{
  RegionMap::const_iterator pos=region_map_.find(p);
  if (pos!=region_map_.end()) {
    return pos->second;
  }
  else {
    throw UnknownGridCellException(p);
  }
}

int RegionGraph::regionType (const RegionId id) const
{
  return graph_[idVertex(id)].type;
}


// Functor for mapping neighbor vertices to ids
struct GetNeighborId
{
  GetNeighborId(const RegionGraphImpl& graph) : g(graph) {}
  RegionId operator() (const RegionGraphVertex& v) { return g[v].id; }
  const RegionGraphImpl& g;
};

RegionIdVector RegionGraph::neighbors (const RegionId id) const
{
  AdjacencyIterator v, v_end;
  tie(v,v_end) = adjacent_vertices(idVertex(id), graph_);
  RegionIdVector neighbors;
  transform (v, v_end, back_inserter(neighbors), GetNeighborId(graph_));
  return neighbors;
}



RegionPtr RegionGraph::regionCells (const RegionId id) const 
{
  return graph_[idVertex(id)].region;
}



const RegionIdVector& RegionGraph::allRegions () const
{
  return regions_;
}


RegionId RegionGraph::addRegion (const RegionPtr region, const int type)
{
  addRegion (region, type, next_id_);
  return next_id_++;
}

void RegionGraph::addRegion (const RegionPtr region, const int type, const RegionId id)
{
  ROS_ASSERT_MSG (lower_bound(regions_.begin(), regions_.end(), id) == upper_bound(regions_.begin(), regions_.end(), id),
                  "Attempted to add region with existing id %u", id);

  // Make sure there's no overlap with existing regions
  for (Region::iterator region_iter=region->begin(); region_iter!=region->end(); ++region_iter) {
    if (region_map_.find(*region_iter)!=region_map_.end()) {
      throw OverlappingRegionException(*region_iter, region_map_.find(*region_iter)->second);
    }
  }

  // Insert while keeping sorted
  regions_.insert(upper_bound(regions_.begin(), regions_.end(), id), id);

  // Add vertex and edges
  RegionGraphVertex v=add_vertex(RegionInfo(type, region, id), graph_);
  ROS_DEBUG_STREAM_NAMED ("region_graph", "Added region " << id);

  // Neighbors that have been added so far
  set<RegionGraphVertex> seen_neighbors;

  // Update the region index and add edges to neighboring regions if necessary
  for (Region::iterator region_iter=region->begin(); region_iter!=region->end(); ++region_iter) {
    region_map_[*region_iter]=id;
    vector<Cell2D> cell_neighbors=cellNeighbors(*region_iter);
    for (vector<Cell2D>::iterator neighbor_iter=cell_neighbors.begin(); neighbor_iter!=cell_neighbors.end(); ++neighbor_iter) {

      RegionMap::iterator neighbor_ptr=region_map_.find(*neighbor_iter);
      if (neighbor_ptr!=region_map_.end() && neighbor_ptr->second!=id) {
        RegionGraphVertex neighbor_vertex=idVertex(neighbor_ptr->second);
        if (seen_neighbors.find(neighbor_vertex)==seen_neighbors.end()) {
          seen_neighbors.insert(neighbor_vertex);
          add_edge(v, neighbor_vertex, graph_);
          ROS_DEBUG_STREAM_NAMED ("region_graph", "Added edge between regions " << id << " and " << graph_[neighbor_vertex].id);
        }   
      }
    }
    id_vertex_map_[id]=v; 
  }
}





struct RemoveFromMap
{
  RemoveFromMap(RegionMap& m) : region_map(m) {}
  void operator() (const Cell2D& c) { region_map.erase(region_map.find(c)); }
  RegionMap& region_map;
};

void RegionGraph::removeRegion (const RegionId id)
{
  RegionGraphVertex v=idVertex(id);

  // Remove from regions_ and id_vertex_map - we don't do any check as it must exist if we got here without an exception
  regions_.erase(lower_bound(regions_.begin(), regions_.end(), id));
  id_vertex_map_.erase(id_vertex_map_.find(id));

  // Remove all cells in this region from the region map
  for_each(graph_[v].region->begin(), graph_[v].region->end(), RemoveFromMap(region_map_));

  // Remove the vertex
  clear_vertex(v, graph_);
  remove_vertex(v, graph_);
}


RegionGraphVertex RegionGraph::idVertex(const RegionId id) const
{
  IdVertexMap::const_iterator map_iter=id_vertex_map_.find(id);
  if (map_iter != id_vertex_map_.end()) {
    return map_iter->second;
  }
  else {
    throw UnknownRegionException(id);
  }
}


void RegionGraph::writeToStream (ostream& stream) const
{
  using std::endl;
  const RegionIdVector& regions = allRegions();
  stream << endl << regions.size() << " " << next_id_;
  for (RegionIdVector::const_iterator iter=regions.begin(); iter!=regions.end(); ++iter) {
    RegionPtr cells = regionCells(*iter);
    stream << endl << *iter << " " << regionType(*iter) << " " << cells->size();
    for (Region::iterator cell=cells->begin(); cell!=cells->end(); ++cell) {
      stream << " " << cell->r << " " << cell->c;
    }
  }
}


RegionGraph::RegionGraph (istream& stream) 
{
  uint num_regions;
  stream >> num_regions >> next_id_;
  for (uint r=0; r<num_regions; ++r) {
    uint num_cells;
    int type;
    RegionId id;
    stream >> id >> type >> num_cells;
    MutableRegionPtr region(new Region);
    for (uint c=0; c<num_cells; ++c) {
      Cell2D cell;
      stream >> cell.r >> cell.c;
      region->insert(cell);
    }
    
    addRegion(region, type, id);
  }
}



} // namespace
