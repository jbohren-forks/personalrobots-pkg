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
#include <topological_map/grid_graph.h>
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
using boost::make_tuple;
using boost::tuples::ignore;
using boost::extents;
using boost::multi_array;
using std::endl;
using std::map;
using std::vector;
using std::set;
using std::min;
using std::max;


namespace topological_map
{

typedef pair<bool, double> ReachableCost;

/************************************************************
 * Utility
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
    return (*grid_)[cell.r][cell.c];
  }
  return false;
}

bool TopologicalMap::MapImpl::pointOnMap (const Point2D& p) const
{
  return (p.x>=0) && (p.y>=0) && (p.x<numCols(*grid_)*resolution_) && (p.y<numRows(*grid_)*resolution_);
}

Cell2D TopologicalMap::MapImpl::containingCell (const Point2D& p) const
{
  if (!pointOnMap(p)) {
    throw UnknownPointException(p.x,p.y,numCols(*grid_)*resolution_,numRows(*grid_)*resolution_);
  }
  return Cell2D(floor((float)p.y/resolution_), floor((float)p.x/resolution_));
}





/************************************************************
 * I/o
 ************************************************************/


GridPtr readGrid(istream& stream)
{
  uint nr, nc;
  stream >> nr >> nc;
  GridPtr grid(new OccupancyGrid(extents[nr][nc]));
  for (uint r=0; r<nr; r++) {
    for (uint c=0; c<nc; c++) {
      stream >> (*grid)[r][c];
    }
  }
  return grid;
}


void writeGrid (GridPtr grid, ostream& stream)
{
  ROS_DEBUG_NAMED ("io", "Writing %ux%u grid", numRows(*grid), numCols(*grid));
  stream << endl << numRows(*grid) << " " << numCols(*grid);
  for (uint r=0; r<numRows(*grid); ++r) {
    for (uint c=0; c<numCols(*grid); ++c) {
      stream << " " << (*grid)[r][c];
    }
  }
}


shared_ptr<RegionGraph> readRegionGraph(istream& stream)
{
  ROS_DEBUG_NAMED ("io", "Reading region graph");
  return shared_ptr<RegionGraph>(new RegionGraph(stream));
}

shared_ptr<Roadmap> readRoadmap(istream& stream)
{
  ROS_DEBUG_NAMED ("io", "Reading roadmap");
  return shared_ptr<Roadmap>(new Roadmap(stream));
}

 
double readResolution(istream& stream)
{
  double resolution;
  stream >> resolution;
  ROS_DEBUG_NAMED ("io", "Read resolution %f", resolution);
  return resolution;
}

void writeRegionConnectorMap (const RegionConnectorMap& m, ostream& str) 
{
  str << endl << m.size();
  for (RegionConnectorMap::const_iterator iter=m.begin(); iter!=m.end(); ++iter) {
    str << endl << iter->first.first << " " << iter->first.second;
    ConnectorId id;
    Cell2D cell1, cell2;
    tie(id,cell1,cell2) = iter->second;
    str << " " << id << " " << cell1.r << " " << cell1.c << " " << cell2.r << " " << cell2.c;
  }
}

RegionConnectorMap readRegionConnectorMap (istream& str)
{
  ROS_DEBUG_NAMED ("io", "Reading region connector map");
  RegionConnectorMap m;
  uint size;
  str >> size;
  for (uint i=0; i<size; ++i) {
    RegionId r1, r2;
    ConnectorId id;
    Cell2D cell1, cell2;
    str >> r1 >> r2 >> id >> cell1.r >> cell1.c >> cell2.r >> cell2.c;
    m[RegionPair(r1,r2)] = make_tuple(id,cell1,cell2);
  }
  ROS_DEBUG_NAMED ("io", "Done reading region connector map");
  return m;
}


void TopologicalMap::MapImpl::writeToStream (ostream& stream)
{
  ROS_INFO ("Writing topological map of %ux%u grid to stream", numRows(*grid_), numCols(*grid_));
  bool goal_unset = goal_;
  Point2D goal_point;
  if (goal_) {
    // Temporarily unset the goal, as that is not saved
    goal_point = roadmap_->connectorPoint(goal_->id);
    unsetGoal();
  }

  writeGrid(grid_, stream);
  region_graph_->writeToStream(stream);
  roadmap_->writeToStream(stream);
  writeRegionConnectorMap(region_connector_map_, stream);
  stream << endl << resolution_;

  // Add the goal back in if necessary
  if (goal_unset) {
    setGoal(goal_point);
  }
  ROS_INFO ("Done writing topological map");
}




typedef unsigned char uchar;

struct Color
{
  Color(uint r=0, uint g=0, uint b=0) : r(r), g(g), b(b) {}
  uint r, g, b;
};

typedef multi_array<Color, 2> ColorArray;

void writeBitmapPpm (const ColorArray& bitmap, ostream& stream)
{
  typedef ColorArray::size_type grid_size;
  const grid_size* dims = bitmap.shape();
  uint nr=dims[0];
  uint nc=dims[1];
  uint pos=0;
  char* buffer = new char[nr*nc*3];
  
  stream << "P6" << endl << nc << " " << nr << endl << "255" << endl;
  for (uint r=0; r<nr; ++r) {
    for (uint c=0; c<nc; ++c) {
      buffer[pos++] = bitmap[r][c].r;
      buffer[pos++] = bitmap[r][c].g;
      buffer[pos++] = bitmap[r][c].b;
    }
  }

  stream.write(buffer, pos);
  delete[] buffer;
}


void TopologicalMap::MapImpl::writePpm (ostream& str) const
{
  ROS_INFO ("Writing ppm representation of map");
  Color red(255,0,0);
  Color black(0,0,0);
  Color white(255,255,255);
  Color blue(0,0,255);

  uint num_rows = numRows(*grid_);
  uint num_cols = numCols(*grid_);

  multi_array<Color, 2> bitmap(extents[num_rows][num_cols]);
  
  for (uint r=0; r<num_rows; ++r) {
    for (uint c=0; c<num_cols; ++c) {
      if ((*grid_)[r][c]) {
        bitmap[r][c] = black;
      }
      else {
        RegionId region = containingRegion(Cell2D(r,c));
        if (regionType(region) == DOORWAY) {
          bitmap[r][c] = red;
        }
        else {
          bitmap[r][c] = white;
        }
      }
    }
  }
  
  for (RegionConnectorMap::const_iterator iter = region_connector_map_.begin(); iter!=region_connector_map_.end(); ++iter) {
    Cell2D cell = iter->second.get<1>();
    for (int r=cell.r-1; r<=cell.r+1; ++r) {
      for (int c=cell.c-1; c<=cell.c+1; ++c) {
        bitmap[r][c] = blue;
      }
    }
  }
  
  writeBitmapPpm(bitmap, str);
  ROS_INFO ("Done writing ppm representation of map");
}



/************************************************************
 * Construction
 ************************************************************/

ObstacleDistanceArray computeObstacleDistances(GridPtr grid)
{
  int num_rows = numRows(*grid);
  int num_cols = numCols(*grid);
  ObstacleDistanceArray distances(extents[num_rows][num_cols]);
  for (int r=0; r<num_rows; ++r) 
    for (int c=0; c<num_cols; ++c) 
      distances[r][c] = (*grid)[r][c] ? 0 : (num_rows+1)*(num_cols+1);

  bool done=false;
  while (!done) {
    done = true;
    for (int r=0; r<num_rows; ++r) {
      for (int c=0; c<num_cols; ++c) {
        for (uint vertical=0; vertical<2; ++vertical) {
          for (int mult=-1; mult<=1; mult+=2) {
            int dr=mult*(vertical ? 1 : 0);
            int dc=mult*(vertical ? 0 : 1);
            int r1 = min(num_rows-1, max(0,r+dr));
            int c1 = min(num_cols-1, max(0,c+dc));
            if (distances[r1][c1]<distances[r][c]-1) {
              done=false;
              distances[r][c] = distances[r1][c1]+1;
            }
          }
        }
      }
    }
  }
  return distances;
}
    

TopologicalMap::MapImpl::MapImpl(const OccupancyGrid& grid, double resolution) : 
  grid_(new OccupancyGrid(grid)), obstacle_distances_(computeObstacleDistances(grid_)), region_graph_(new RegionGraph), 
  roadmap_(new Roadmap), grid_graph_(new GridGraph(grid_)), resolution_(resolution)
{
}

TopologicalMap::MapImpl::MapImpl (istream& str) : 
  grid_(readGrid(str)), obstacle_distances_(computeObstacleDistances(grid_)), region_graph_(readRegionGraph(str)), 
  roadmap_(readRoadmap(str)), grid_graph_(new GridGraph(grid_)), region_connector_map_(readRegionConnectorMap(str)), 
  resolution_(readResolution(str))
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


const RegionIdSet& TopologicalMap::MapImpl::allRegions () const
{
  return region_graph_->allRegions();
}



/****************************************
 * RegionGraph modification
 ****************************************/



/// \todo this doesn't deal with connectors
void TopologicalMap::MapImpl::removeRegion (const RegionId id)
{
  region_graph_->removeRegion(id);
}




// Looks for the best (furthest from obstacles) cell on the border between two regions
struct NeighborFinder 
{
  NeighborFinder (RegionPtr region, const ObstacleDistanceArray& distances) : found(false), region(region), obstacle_distances(distances) {}
  void operator() (const Cell2D& cell) 
  {
    int r=cell.r;
    int c=cell.c;
    if (!found || obstacle_distances[r][c]>best_distance) {
      if (belongs(r+1,c)||belongs(r-1,c)||belongs(r,c+1)||belongs(r,c-1)) {
        found = true;
        cell1=cell;
        best_distance=obstacle_distances[r][c];
      }
    }
  }

  bool belongs (int r, int c)
  {
    Cell2D cell(r,c);
    if (region->find(cell)!=region->end()) {
      cell2=cell;
      return true;
    }
    return false;
  }
    
  bool found; 
  RegionPtr region;
  Cell2D cell1, cell2;
  int best_distance;
  const ObstacleDistanceArray& obstacle_distances;
};

RegionId TopologicalMap::MapImpl::addRegion (const RegionPtr region, const int type)
{
  RegionId region_id = region_graph_->addRegion(region, type);
  RegionIdVector neighbors = region_graph_->neighbors(region_id);


  for (RegionIdVector::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter) {

    // Find a neighboring cell.  Iterate over the smaller of the two regions for efficiency.
    RegionPtr neighbor_region = region_graph_->regionCells(*iter);
    RegionPtr region1, region2;
    if (region->size() < neighbor_region->size()) {
      region1 = region;
      region2 = neighbor_region;
      }
    else {
      region1 = neighbor_region;
      region2 = region;
    }
    NeighborFinder neighbor_finder = for_each(region1->begin(), region1->end(), NeighborFinder(region2, obstacle_distances_));
    ROS_ASSERT (neighbor_finder.found);

    Point2D border_point=findBorderPoint(neighbor_finder.cell1, neighbor_finder.cell2);

    ConnectorId connector_id = roadmap_->addNode(border_point);
    RegionId r1, r2;
    r1 = min(region_id, *iter);
    r2 = max(region_id, *iter);
    region_connector_map_[RegionPair(r1,r2)] = make_tuple(connector_id, neighbor_finder.cell1, neighbor_finder.cell2);
  }

  // Compute the distance from the new connector to all connectors touching either of the adjacent regions
  // 1. Loop over neighbors of new region (and corresponding connector)
  for (RegionIdVector::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter) {

    RegionId r1 = min(region_id, *iter);
    RegionId r2 = max(region_id, *iter);

    RegionConnectorMap::iterator rc_pos = region_connector_map_.find(RegionPair(r1,r2));
    ROS_ASSERT(rc_pos!=region_connector_map_.end());
    ConnectorId connector_id = rc_pos->second.get<0>();
    Cell2D cell = rc_pos->second.get<1>();

    // 2. Loop over which of the two adjacent regions to consider
    for (char first_region=0; first_region<2; ++first_region) {
      RegionId r = first_region ? r1 : r2;
      Region connector_region = *(region_graph_->regionCells(r));
      connector_region.insert(cell);
      ROS_DEBUG_STREAM_NAMED ("add_region", "Considering connector " << connector_id << " in region " << r);
      
      RegionIdVector connector_neighbors = region_graph_->neighbors(r);

      // 3. Loop over which of the neighboring regions (and corresponding connectors) to find path to
      for (RegionIdVector::iterator region_iter=connector_neighbors.begin(); region_iter!=connector_neighbors.end(); ++region_iter) {

        // Proceed only if the regions and connectors are different
        if (r!=*region_iter) {
          RegionConnectorMap::const_iterator pos = region_connector_map_.find(RegionPair(min(*region_iter,r), max(*region_iter,r)));
          ROS_ASSERT_MSG(pos!=region_connector_map_.end(), "Failed to find connector between regions %d and %d", r, *region_iter);
          ConnectorId other_connector = pos->second.get<0>();

          if (other_connector!=connector_id) {
            ROS_DEBUG_STREAM_NAMED("add_region", "Considering other connector " << other_connector << " in region " << *region_iter);

            // Compute shortest path
            Cell2D other_connector_cell=pos->second.get<1>();
            bool found;
            double distance;
            tie(found, distance, ignore) = grid_graph_->shortestPath(cell, other_connector_cell);
            if (found) {
              roadmap_->setCost(other_connector, connector_id, resolution_*distance);
            }
          }
        }
      }
    }
  }
  return region_id;
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
  Point2D p=cellCorner(cell1, resolution_);
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
  for (uint i=0; i<neighbors.size(); ++i) {
    connectors[i] = connectorBetween(id, neighbors[i]);
  }
  return connectors;
}

typedef tuple<ConnectorId, Cell2D, Cell2D> ConnectorDesc;

vector<ConnectorDesc> TopologicalMap::MapImpl::adjacentConnectorCells (const RegionId id) const
{
  RegionIdVector neighbors = this->neighbors(id);
  vector<ConnectorDesc> descs(neighbors.size());
  for (uint i=0; i<neighbors.size(); ++i) {
    descs[i] = connectorCellsBetween(id, neighbors[i]);
  }
  return descs;
}
  

ConnectorId TopologicalMap::MapImpl::connectorBetween (const RegionId r1, const RegionId r2) const
{
  RegionConnectorMap::const_iterator pos = region_connector_map_.find(RegionPair(min(r1,r2), max(r1,r2)));
  ROS_ASSERT_MSG(pos!=region_connector_map_.end(), "Failed to find connector between regions %u and %u", r1, r2);
  return pos->second.get<0>();
}

ConnectorDesc TopologicalMap::MapImpl::connectorCellsBetween (RegionId r1, RegionId r2) const
{
  RegionConnectorMap::const_iterator pos = region_connector_map_.find(RegionPair(min(r1, r2), max(r1,r2)));
  ROS_ASSERT_MSG(pos!=region_connector_map_.end(), "Failed to find connector between regions %u and %u", r1, r2);
  ConnectorId id;
  Cell2D cell1, cell2;
  tie(id,cell1,cell2) = pos->second;
  RegionPtr region1 = regionCells(r1);
  if (region1->find(cell1)==region1->end()) {
    return make_tuple(id,cell2,cell1);
  }
  else {
    return pos->second;
  }
}



struct HasId 
{
  HasId(const ConnectorId id) : id(id) {}
  bool operator() (pair<const RegionPair, ConnectorDesc> pair) { return pair.second.get<0>()==id; }
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



void TopologicalMap::MapImpl::setGoal (const Point2D& p)
{
  // Remove old goal if it exists (we could dispense with this - that would just 
  // mean the new node gets added before the old one is removed)
  unsetGoal();

  // Add new goal
  goal_ = TempNodePtr(new TemporaryRoadmapNode(this, p));
}

void TopologicalMap::MapImpl::unsetGoal ()
{
  goal_ = TempNodePtr();
}

void TopologicalMap::MapImpl::setGoal (const Cell2D& c)
{
  setGoal(cellCenter(c, resolution_));
}

ReachableCost TopologicalMap::MapImpl::goalDistance (ConnectorId id) const
{
  if (!goal_) {
    throw GoalNotSetException();
  }    
  return roadmap_->costBetween(id, goal_->id);
}


ReachableCost TopologicalMap::MapImpl::getDistance (const Point2D& p1, const Point2D& p2)
{
  TemporaryRoadmapNode start(this, p1);
  TemporaryRoadmapNode goal(this, p2);
  return roadmap_->costBetween(start.id, goal.id);
}

vector<pair<ConnectorId, double> > TopologicalMap::MapImpl::connectorCosts (const Point2D& p1, const Point2D& p2)
{
  TemporaryRoadmapNode start(this, p1);
  TemporaryRoadmapNode goal(this, p2);
  return roadmap_->connectorCosts(start.id, goal.id);
}

ConnectorId getId (const ConnectorDesc& desc)
{
  return desc.get<0>();
}

Cell2D getCell (const ConnectorDesc& desc)
{
  return desc.get<1>();
}

TopologicalMap::MapImpl::TemporaryRoadmapNode::TemporaryRoadmapNode (TopologicalMap::MapImpl* m, const Point2D& p)
  : map(m), id(m->roadmap_->addNode(p))
{
  Cell2D cell = m->containingCell(p);
  RegionId r = m->containingRegion(cell);
  ROS_DEBUG_STREAM_NAMED ("temp_node", "Adding temporary node " << id << " at cell " << cell << " in region " << r);

  // When computing distances, we need to separate this region from the rest of the graph
  RegionIsolator i(m->grid_graph_.get(), *(m->regionCells(r)));

  vector<ConnectorDesc> connector_descs = m->adjacentConnectorCells(r);
  vector<ConnectorId> connector_ids(connector_descs.size());
  vector<Cell2D> connector_cells(connector_descs.size());
  transform(connector_descs.begin(), connector_descs.end(), connector_ids.begin(), getId);
  transform(connector_descs.begin(), connector_descs.end(), connector_cells.begin(), getCell);

  vector<ReachableCost> costs = m->grid_graph_->singleSourceCosts(cell, connector_cells);

  for (uint i=0; i<connector_ids.size(); ++i) 
    if (costs[i].first) 
      m->roadmap_->setCost(id, connector_ids[i], m->resolution_*costs[i].second);
}


TopologicalMap::MapImpl::TemporaryRoadmapNode::~TemporaryRoadmapNode ()
{
  map->roadmap_->removeNode(id);
}


 
/************************************************************
 * TopologicalMap ops are forwarded to MapImpl
 ************************************************************/

TopologicalMap::TopologicalMap (const OccupancyGrid& grid, double resolution) : map_impl_(new MapImpl(grid, resolution))
{
}


TopologicalMap::TopologicalMap(istream& str) : map_impl_(new MapImpl(str))
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

ReachableCost TopologicalMap::distanceBetween (const Point2D& p1, const Point2D& p2)
{
  return map_impl_->getDistance(p1,p2);
}

vector<pair<ConnectorId, double> > TopologicalMap::connectorCosts (const Point2D& p1, const Point2D& p2)
{

  return map_impl_->connectorCosts(p1,p2);
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

const RegionIdSet& TopologicalMap::allRegions () const
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
  for (RegionIdSet::const_iterator iter=m.allRegions().begin(); iter!=m.allRegions().end(); ++iter) {
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

void TopologicalMap::writeToStream (ostream& str) const
{
  map_impl_->writeToStream (str);
}

void TopologicalMap::writePpm (ostream& str) const
{
  map_impl_->writePpm(str);
}
  
  
  
  



} // namespace topological_map
