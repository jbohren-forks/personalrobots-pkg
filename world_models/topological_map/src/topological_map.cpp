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
#include <ros/console.h>
#include <ros/assert.h>
#include <topological_map/exception.h>

// For debugging
#include <iostream>
using std::cout;
using std::endl;


using boost::tie;
using std::map;
using std::vector;
using std::set;
using std::min;
using std::max;

namespace topological_map
{


uint numRows(const OccupancyGrid& grid)
{
  const occ_grid_size* dims=grid.shape();
  return dims[0];
}

uint numCols(const OccupancyGrid& grid)
{
  const occ_grid_size* dims=grid.shape();
  return dims[1];
}

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

RegionId TopologicalMap::GraphImpl::containingRegion (const Point2D& p) const
{
  Cell2D c=containingCell(p);
  ROS_DEBUG_STREAM_NAMED ("point_cell_map", "point " << p.x << ", " << p.y << " maps to cell " << c);
  return containingRegion(c);
}


ConnectorId TopologicalMap::GraphImpl::pointConnector (const Point2D& p) const
{
  return cellConnector(containingCell(p));
}

ConnectorId TopologicalMap::GraphImpl::cellConnector (const Cell2D& c) const
{
  CellConnectorMap::const_iterator iter=cell_connector_map_.find(c);

  if (iter==cell_connector_map_.end()) {
    throw UnknownConnectorException(c.r, c.c);
  }
  return iter->second; 
}

Point2D TopologicalMap::GraphImpl::connectorPosition (const ConnectorId id) const
{
  return cellCorner(connectorCell(id));
}

Cell2D TopologicalMap::GraphImpl::connectorCell (const ConnectorId id) const
{
  if (id<=connectors_.size() && id > 0) {
    return getConnector(id);
  }
  else {
    throw UnknownConnectorIdException(id);
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


vector<ConnectorId> TopologicalMap::GraphImpl::adjacentConnectors (const RegionId id) const
{
  RegionIdVector neighbors = this->neighbors(id);
  vector<ConnectorId> connectors(neighbors.size());
  for (uint i=0; i<neighbors.size(); i++) {
    connectors[i] = connectorBetween(id, neighbors[i]);
  }
  return connectors;
}

ConnectorId TopologicalMap::GraphImpl::connectorBetween (const RegionId r1, const RegionId r2) const
{
  RegionConnectorMap::const_iterator pos = region_connector_map_.find(RegionPair(min(r1,r2), max(r1,r2)));
  ROS_ASSERT_MSG(pos!=region_connector_map_.end(), "Failed to find connector between regions %d and %d", r1, r2);
  return pos->second;
}

Cell2D TopologicalMap::GraphImpl::getConnector(const ConnectorId id) const
{
  return connectors_[id-1];
}

RegionPair TopologicalMap::GraphImpl::adjacentRegions (const ConnectorId id) const
{
  Cell2D cell=getConnector(id);
  RegionId r=containingRegion(cell);
  RegionIdVector neighbors=this->neighbors(r);
  for (RegionIdVector::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter) {
    if (connectorBetween(r, *iter)==id) {
      return RegionPair(r,*iter);
    }
  }
  ROS_ASSERT("Error finding adjacent pair of regions for connector");
  return RegionPair(); // to prevent warning
}


bool TopologicalMap::GraphImpl::isObstacle (const Point2D& p) const
{
  Cell2D cell;
  try {
    cell=containingCell(p);
    RegionId r=containingRegion(cell);
    ROS_DEBUG_NAMED("obstacle", "not an obstacle as it belongs to region %u", r);
    return false;
  }
  catch (UnknownPointException& e) {
    ROS_DEBUG_NAMED("obstacle", "not an obstacle as it's off the map");
    return false;
  }
  catch (UnknownGridCellException& e) {
    ROS_DEBUG_STREAM_NAMED("obstacle", "obstacle as it's on the map at cell " << cell << " but has no region");
    return true;
  }
}


RegionPtr TopologicalMap::GraphImpl::regionCells (const RegionId id) const 
{
  return graph_[idVertex(id)].region;
}


const RegionIdVector& TopologicalMap::GraphImpl::allRegions () const
{
  return regions_;
}


struct IsNeighbor 
{
  IsNeighbor (RegionPtr region) : region(region) {}
  bool operator() (const Cell2D& cell) 
  {
    int r=cell.r;
    int c=cell.c;
    return (belongs(r+1,c) || belongs(r-1,c) || belongs(r,c+1) || belongs(r,c-1));
  }

  bool belongs (int r, int c)
  {
    return (region->count(Cell2D(r,c))>0);
  }
    
  RegionPtr region;
};

void TopologicalMap::GraphImpl::connectRegions (const TopologicalGraphVertex& v, const TopologicalGraphVertex& v2)
{
  add_edge (v, v2, graph_);
  Region::iterator iter = find_if(graph_[v].region->begin(), graph_[v].region->end(), IsNeighbor(graph_[v2].region));
  ROS_ASSERT (iter!=graph_[v].region->end());
  connectors_.push_back(*iter);
  ConnectorId id=connectors_.size();
  cell_connector_map_[*iter]=id;
  RegionId r1, r2;
  r1 = min(graph_[v].id, graph_[v2].id);
  r2 = max(graph_[v].id, graph_[v2].id);
  region_connector_map_[RegionPair(r1,r2)] = id;
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
          connectRegions(v, neighbor_vertex);
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

/// \todo this doesn't deal with connectors
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

Cell2D TopologicalMap::GraphImpl::containingCell (const Point2D& p) const
{
  if ((p.x<0) || (p.y<0) || (p.x>num_cols_*resolution_) || (p.y>num_rows_*resolution_)) {
    throw UnknownPointException(p.x,p.y);
  }
  return Cell2D(floor((float)p.y/resolution_), floor((float)p.x/resolution_));
}

Point2D TopologicalMap::GraphImpl::cellCorner (const Cell2D& cell) const
{
  return Point2D(cell.c*resolution_,cell.r*resolution_);
}



/************************************************************
 * Topological graph ops are forwarded to implementation
 ************************************************************/

TopologicalMap::TopologicalMap (uint nr, uint nc, double resolution) : graph_impl_(new GraphImpl(nr, nc, resolution)) 
{
}

RegionId TopologicalMap::containingRegion (const Cell2D& p) const
{
  return graph_impl_->containingRegion(p);
}

RegionId TopologicalMap::containingRegion (const Point2D& p) const
{
  return graph_impl_->containingRegion(p);
}

int TopologicalMap::regionType (const RegionId id) const
{
  return graph_impl_->regionType(id);
}


RegionIdVector TopologicalMap::neighbors (const RegionId id) const
{
  return graph_impl_->neighbors(id);
}

ConnectorId TopologicalMap::pointConnector (const Point2D& p) const
{
  return graph_impl_->pointConnector(p);
}

Point2D TopologicalMap::connectorPosition (const ConnectorId id) const
{
  return graph_impl_->connectorPosition(id);
}

vector<ConnectorId> TopologicalMap::adjacentConnectors (const RegionId id) const
{
  return graph_impl_->adjacentConnectors(id);
}

RegionPair TopologicalMap::adjacentRegions (const ConnectorId id) const
{
  return graph_impl_->adjacentRegions(id);
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

bool TopologicalMap::isObstacle (const Point2D& p) const
{
  return graph_impl_->isObstacle(p);
}

const RegionIdVector& TopologicalMap::allRegions () const
{
  return graph_impl_->allRegions();
}

RegionPtr TopologicalMap::regionCells (const RegionId id) const
{
  return graph_impl_->regionCells(id);
}

// Print the topological map in human readable form
ostream& operator<< (ostream& str, const TopologicalMap& m)
{
  str << "Topological map with " << m.allRegions().size() << " regions" << endl;
  for (RegionIdVector::const_iterator iter=m.allRegions().begin(); iter!=m.allRegions().end(); ++iter) {
    str << "Region " << *iter << " of type " << m.regionType(*iter) << endl << " Cells: ";
    RegionPtr region = m.regionCells(*iter);
    for (Region::const_iterator cell_iter=region->begin(); cell_iter!=region->end(); ++cell_iter) {
      str << *cell_iter << " ";
    }
    str << endl << " Neighbors: ";
    RegionIdVector neighbors = m.neighbors(*iter);
    for (RegionIdVector::const_iterator neighbor_iter=neighbors.begin(); neighbor_iter!=neighbors.end(); ++neighbor_iter) {
      str << *neighbor_iter << " ";
    }
    str << endl << " Connectors: ";
    vector<ConnectorId> connectors = m.adjacentConnectors(*iter);
    for (vector<ConnectorId>::const_iterator connector_iter=connectors.begin(); connector_iter!=connectors.end(); ++connector_iter) {
      str << *connector_iter << ": " << m.connectorPosition(*connector_iter).x << ", " << m.connectorPosition(*connector_iter).y << "; ";
    }
    str << endl;
  }
  return str;
}   




} // namespace topological_map
