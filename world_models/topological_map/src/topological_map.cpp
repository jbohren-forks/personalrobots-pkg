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

#include <topological_map/topological_map_impl.h>
#include <topological_map/region_graph.h>
#include <topological_map/roadmap.h>
#include <algorithm>
#include <ros/console.h>
#include <ros/assert.h>
#include <topological_map/exception.h>

// For debugging
#include <iostream>
using std::endl;
using std::ostream;

using boost::tie;
using std::map;
using std::vector;
using std::set;
using std::min;
using std::max;

namespace topological_map
{

/************************************************************
 * Miscellaneous
 ************************************************************/

ostream& operator<< (ostream& str, const Point2D& p)
{
  str << "(" << p.x << ", " << p.y << ")";
  return str;
}

bool operator== (const Point2D& p1, const Point2D& p2)
{
  return (p1.x==p2.x) && (p1.y==p2.y);
}

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


bool TopologicalMap::MapImpl::isObstacle (const Point2D& p) const
{
  if (pointOnMap(p)) {
    Cell2D cell = containingCell(p);
    return grid_[cell.r][cell.c];
  }
  return false;
}

bool TopologicalMap::MapImpl::pointOnMap (const Point2D& p) const
{
  return (p.x>=0) && (p.y>=0) && (p.x<numCols(grid_)*resolution_) && (p.y<numRows(grid_)*resolution_);
}

Cell2D TopologicalMap::MapImpl::containingCell (const Point2D& p) const
{
  if (!pointOnMap(p)) {
    throw UnknownPointException(p.x,p.y);
  }
  return Cell2D(floor((float)p.y/resolution_), floor((float)p.x/resolution_));
}

Point2D TopologicalMap::MapImpl::cellCorner (const Cell2D& cell) const
{
  return Point2D(cell.c*resolution_,cell.r*resolution_);
}




/************************************************************
 * MapImpl
 ************************************************************/

TopologicalMap::MapImpl::MapImpl(const OccupancyGrid& grid, double resolution) : 
  region_graph_(new RegionGraph), roadmap_(new Roadmap), resolution_(resolution), grid_(grid)
{
}


/****************************************
 * RegionGraph const ops
 ****************************************/

RegionId TopologicalMap::MapImpl::containingRegion (const Cell2D& p) const
{
  return region_graph_->containingRegion(p);
}

int TopologicalMap::MapImpl::regionType (const RegionId id) const
{
  return region_graph_->regionType(id);
}


RegionIdVector TopologicalMap::MapImpl::neighbors (const RegionId id) const
{
  return region_graph_->neighbors(id);
}




RegionPtr TopologicalMap::MapImpl::regionCells (const RegionId id) const 
{
  return region_graph_->regionCells(id);
}


const RegionIdVector& TopologicalMap::MapImpl::allRegions () const
{
  return region_graph_->allRegions();
}



/****************************************
 * RegionGraph modification
 ****************************************/


struct NeighborFinder 
{
  NeighborFinder (RegionPtr region) : found(false), region(region) {}
  void operator() (const Cell2D& cell) 
  {
    if (!found)
    { 
      cell1 = cell;
      int r=cell1.r;
      int c=cell1.c;
      found = (belongs(r+1,c) || belongs(r-1,c) || belongs(r,c+1) || belongs(r,c-1));
    }
  }

  bool belongs (int r, int c)
  {
    cell2=Cell2D(r,c);
    return region->find(cell2)!=region->end();
  }
    
  bool found; 
  RegionPtr region;
  Cell2D cell1, cell2;
};

  

RegionId TopologicalMap::MapImpl::addRegion (const RegionPtr region, const int type)
{
  RegionId region_id = region_graph_->addRegion(region, type);
  RegionIdVector neighbors = region_graph_->neighbors(region_id);


  for (RegionIdVector::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter) {
    
    NeighborFinder neighbor_finder = for_each(region->begin(), region->end(), NeighborFinder(region_graph_->regionCells(*iter)));
    ROS_ASSERT (neighbor_finder.found);
  
    Point2D border_point=findBorderPoint(neighbor_finder.cell1, neighbor_finder.cell2);

    ConnectorId connector_id = roadmap_->addNode(border_point);
    RegionId r1, r2;
    r1 = min(region_id, *iter);
    r2 = max(region_id, *iter);
    region_connector_map_[RegionPair(r1,r2)] = connector_id;
  }
  return region_id;
}




/// \todo this doesn't deal with connectors
void TopologicalMap::MapImpl::removeRegion (const RegionId id)
{
  region_graph_->removeRegion(id);
}



/****************************************
 * Connector-related ops
 ****************************************/



Point2D TopologicalMap::MapImpl::findBorderPoint(const Cell2D& cell1, const Cell2D& cell2) const
{
  ROS_DEBUG_STREAM_NAMED("border_point", "Looking for border point between cells " << cell1 << " and " << cell2);
  double dy=(cell2.r-cell1.r+1)/2.0;
  double dx=(cell2.c-cell1.c+1)/2.0;
  ROS_ASSERT_MSG (((dy==.5)||(dy==0)||(dy==1)) && ((dx==.5)||(dx==0)||(dx==1)), "(%f, %f) was an illegal value in findBorderPoint", dx, dy);
  Point2D p=cellCorner(cell1);
  p.y+=dy*resolution_;
  p.x+=dx*resolution_;
  ROS_DEBUG_STREAM_NAMED ("border_point", "Border point is " << p);
  return p;
}
  


RegionId TopologicalMap::MapImpl::containingRegion (const Point2D& p) const
{
  Cell2D c=containingCell(p);
  ROS_DEBUG_STREAM_NAMED ("point_cell_map", "point " << p.x << ", " << p.y << " maps to cell " << c);
  return containingRegion(c);
}


ConnectorId TopologicalMap::MapImpl::pointConnector (const Point2D& p) const
{
  return roadmap_->pointId(p);
}



Point2D TopologicalMap::MapImpl::connectorPosition (const ConnectorId id) const
{
  return roadmap_->connectorPoint(id);
}



vector<ConnectorId> TopologicalMap::MapImpl::adjacentConnectors (const RegionId id) const
{
  RegionIdVector neighbors = this->neighbors(id);
  vector<ConnectorId> connectors(neighbors.size());
  for (uint i=0; i<neighbors.size(); i++) {
    connectors[i] = connectorBetween(id, neighbors[i]);
  }
  return connectors;
}

ConnectorId TopologicalMap::MapImpl::connectorBetween (const RegionId r1, const RegionId r2) const
{
  RegionConnectorMap::const_iterator pos = region_connector_map_.find(RegionPair(min(r1,r2), max(r1,r2)));
  ROS_ASSERT_MSG(pos!=region_connector_map_.end(), "Failed to find connector between regions %d and %d", r1, r2);
  return pos->second;
}

  struct HasId 
  {
    HasId(const ConnectorId id) : id(id) {}
    bool operator() (pair<const RegionPair, ConnectorId> pair) { return pair.second==id; }
    const ConnectorId id;
  };


RegionPair TopologicalMap::MapImpl::adjacentRegions (const ConnectorId id) const
{
  RegionConnectorMap::const_iterator iter = find_if(region_connector_map_.begin(), region_connector_map_.end(), HasId(id));
  if (iter==region_connector_map_.end()) {
    ROS_ASSERT("Error finding adjacent pair of regions for connector");
  }
  
  return iter->first;
}


 
/************************************************************
 * TopologicalMap ops are forwarded to MapImpl
 ************************************************************/

TopologicalMap::TopologicalMap (const OccupancyGrid& grid, double resolution) : map_impl_(new MapImpl(grid, resolution))
{
}

RegionId TopologicalMap::containingRegion (const Cell2D& p) const
{
  return map_impl_->containingRegion(p);
}

RegionId TopologicalMap::containingRegion (const Point2D& p) const
{
  return map_impl_->containingRegion(p);
}

int TopologicalMap::regionType (const RegionId id) const
{
  return map_impl_->regionType(id);
}


RegionIdVector TopologicalMap::neighbors (const RegionId id) const
{
  return map_impl_->neighbors(id);
}

ConnectorId TopologicalMap::pointConnector (const Point2D& p) const
{
  return map_impl_->pointConnector(p);
}

Point2D TopologicalMap::connectorPosition (const ConnectorId id) const
{
  return map_impl_->connectorPosition(id);
}

vector<ConnectorId> TopologicalMap::adjacentConnectors (const RegionId id) const
{
  return map_impl_->adjacentConnectors(id);
}

RegionPair TopologicalMap::adjacentRegions (const ConnectorId id) const
{
  return map_impl_->adjacentRegions(id);
}

RegionId TopologicalMap::addRegion(const RegionPtr region, const int type) 
{
  return map_impl_->addRegion(region, type);
}

void TopologicalMap::removeRegion (const RegionId id)
{
  map_impl_->removeRegion (id);
}

bool TopologicalMap::isObstacle (const Point2D& p) const
{
  return map_impl_->isObstacle(p);
}

const RegionIdVector& TopologicalMap::allRegions () const
{
  return map_impl_->allRegions();
}

RegionPtr TopologicalMap::regionCells (const RegionId id) const
{
  return map_impl_->regionCells(id);
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
