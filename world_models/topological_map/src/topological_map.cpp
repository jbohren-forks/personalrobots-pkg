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
#include <topological_map/door_info.h>
#include <algorithm>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <ros/console.h>
#include <ros/assert.h>
#include <tinyxml/tinyxml.h>
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
using boost::bind;
using boost::ref;
using std::endl;
using std::map;
using std::vector;
using std::set;
using std::min;
using std::max;
using ros::Time;
using tf::Transform;
using tf::Vector3;
using tf::Quaternion;
using tf::Pose;
using robot_msgs::Point32;

namespace topological_map
{

typedef pair<bool, double> ReachableCost;
typedef boost::counting_iterator<unsigned int> Counter;

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
  const Point2D p2 = transformPoint(p);
  if (pointOnMap(p2)) {
    Cell2D cell = containingCell(p2);
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


Point2D TopologicalMap::MapImpl::transformPoint (const Point2D& p) const
{
  Vector3 v(p.x, p.y, 0.0);
  Vector3 w = transform_*v;
  Point2D p2(w.x(), w.y());
  ROS_DEBUG_STREAM_NAMED ("transform", "Transformed " << p << " to " << p2);
  return Point2D(w.getX(), w.getY());
}

Point2D TopologicalMap::MapImpl::inverseTransformPoint (const Point2D& p) const
{
  Vector3 v(p.x, p.y, 0.0);
  Vector3 w = transform_.inverse()*v;
  return Point2D(w.getX(), w.getY());
}  

OutletInfo transformOutlet (const Transform& transform, const OutletInfo& outlet) 
{
  Vector3 v(outlet.x, outlet.y, outlet.z);
  Quaternion q(outlet.qx, outlet.qy, outlet.qz, outlet.qw);
  Pose p(q,v);
  Pose p2 = transform*p;
  OutletInfo o2(outlet);
  Quaternion q2 = p2.getRotation();
  Vector3 v2 = p2.getOrigin();
  o2.x = v2.getX();
  o2.y = v2.getY();
  o2.z = v2.getZ();
  o2.qx = q2.x();
  o2.qy = q2.y();
  o2.qz = q2.z();
  o2.qw = q2.w();
  return o2;
}

Point32 transform (const Transform& transform, const Point32& p)
{
  Vector3 v(p.x, p.y, p.z);
  Vector3 v2 = transform*v;
  Point32 p2;
  p2.x = v2.x();
  p2.y = v2.y();
  p2.z = v2.z();
  return p2;
}



/************************************************************
 * I/o
 ************************************************************/


GridPtr readGrid(istream& stream)
{
  if (!stream.good()) {
    throw TopologicalMapException ("Input stream for reading topological map was not good");
  }
  uint nr, nc;
  stream >> nr >> nc;
  ROS_DEBUG_STREAM_NAMED ("io", "About to read " << nr << "x" << nc << " grid");
  GridPtr grid(new OccupancyGrid(extents[nr][nc]));
  for (uint r=0; r<nr; r++) {
    for (uint c=0; c<nc; c++) {
      stream >> (*grid)[r][c];
    }
  }
  ROS_DEBUG_STREAM_NAMED ("io", "Done reading grid");
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

RegionDoorMap readRegionDoorMap (istream& str, const double prior_open_prob)
{
  ROS_DEBUG_NAMED ("io", "Reading region door map");
  RegionDoorMap m;
  uint size;
  str >> size;
  for (uint i=0; i<size; ++i) {
    RegionId id;
    str >> id;
    m[id] = DoorInfoPtr(new DoorInfo(str, prior_open_prob));
  }
  return m;
}

void writeRegionDoorMap (const RegionDoorMap& m, ostream& str)
{
  str << endl << m.size();
  for (RegionDoorMap::const_iterator iter=m.begin(); iter!=m.end(); ++iter) {
    str << endl << iter->first;
    iter->second->writeToStream(str);
  }
}

void writeOutlet (ostream& str, const OutletInfo& outlet)
{    
    str << endl << outlet.x << " " << outlet.y << " " << outlet.z << " "
        << outlet.qx << " " << outlet.qy << " " << outlet.qz << " " << outlet.qw << " "
        << outlet.sockets_size << " " << outlet.sockets_color;

}

void writeOutlets (const OutletVector& outlets, ostream& str)
{
  str << endl << outlets.size();
  for_each (outlets.begin(), outlets.end(), bind(writeOutlet, ref(str), _1));
}

OutletInfo readOutlet (istream& str)
{
  return OutletInfo(str);
}
    
OutletVector readOutlets (istream& str)
{
  ROS_DEBUG_NAMED ("io", "Reading outlets");
  
  uint num_outlets;
  str >> num_outlets;
  OutletVector outlets(num_outlets);

  generate(outlets.begin(), outlets.end(), bind(readOutlet, ref(str)));

  ROS_DEBUG_NAMED ("io", "Done reading outlets");
  return outlets;
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
  writeRegionDoorMap(region_door_map_, stream);
  writeOutlets(outlets_, stream);
  stream << endl << resolution_;

  // Add the goal back in if necessary
  if (goal_unset) {
    setGoal(goal_point);
  }
  ROS_INFO ("Done writing topological map");
}


void TopologicalMap::MapImpl::writeGridAndOutletData (const string& filename) const
{
  TiXmlDocument doc;
  TiXmlElement* resElt = new TiXmlElement("resolution");
  doc.LinkEndChild(resElt);
  doc.SaveFile(filename);
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
  Color open[12];
  Color blue(0,0,255);
  open[0] = Color(255,255,255);
  open[1] = Color(0,255,0);
  open[2] = Color(255,0,255);
  open[3] = Color(255,255,0);
  open[4] = Color(0,255,255);
  open[6] = Color(0,128,0);
  open[7] = Color(0,0,128);
  open[8] = Color(0,128,128);
  open[9] = Color(128,0,128);
  open[10] = Color(128,128,0);
  open[11] = Color(128,128,128);
  

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
          bitmap[r][c] = open[region % 12];
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
    

TopologicalMap::MapImpl::MapImpl(const OccupancyGrid& grid, double resolution, double door_open_prior_prob, 
                                 double door_reversion_rate, double locked_door_cost, const Transform& transform) :
  grid_(new OccupancyGrid(grid)), obstacle_distances_(computeObstacleDistances(grid_)), region_graph_(new RegionGraph), 
  roadmap_(new Roadmap), grid_graph_(new GridGraph(grid_)), door_open_prior_prob_(door_open_prior_prob), 
  door_reversion_rate_(door_reversion_rate), locked_door_cost_(locked_door_cost), resolution_(resolution),
  transform_(transform)
{
}

TopologicalMap::MapImpl::MapImpl (istream& str, double door_open_prior_prob, double door_reversion_rate,
                                  double locked_door_cost, const Transform& transform) : 
  grid_(readGrid(str)), obstacle_distances_(computeObstacleDistances(grid_)), region_graph_(readRegionGraph(str)), 
  roadmap_(readRoadmap(str)), grid_graph_(new GridGraph(grid_)),
  region_connector_map_(readRegionConnectorMap(str)), door_open_prior_prob_(door_open_prior_prob),
  door_reversion_rate_(door_reversion_rate), locked_door_cost_(locked_door_cost), 
  region_door_map_(readRegionDoorMap(str, door_open_prior_prob_)), 
  outlets_(readOutlets(str)), resolution_(readResolution(str)), transform_(transform)
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
 * Doors
 ****************************************/

Door TopologicalMap::MapImpl::regionDoor (RegionId id) const
{
  if (region_graph_->regionType(id)!=DOORWAY) {
    throw NotDoorwayRegionException(id);
  }
  RegionDoorMap::const_iterator iter = region_door_map_.find(id);
  if (iter==region_door_map_.end()) {
    throw NoDoorInRegionException(id);
  }
  Door door = iter->second->getDoorMessage();
  
  Transform inv = transform_.inverse();
  ROS_DEBUG_STREAM_NAMED ("transform", "door has pts " << door.frame_p1.x << ", " << door.frame_p1.y << " and " << door.frame_p2.x << ", " << door.frame_p2.y);
  door.frame_p1 = transform(inv,door.frame_p1);
  door.frame_p2 = transform(inv,door.frame_p2);
  door.door_p1 = transform(inv,door.door_p1);
  door.door_p2 = transform(inv,door.door_p2);
  door.handle = transform(inv, door.handle);
  ROS_DEBUG_STREAM_NAMED ("transform", "door has pts " << door.frame_p1.x << ", " << door.frame_p1.y << " and " << door.frame_p2.x << ", " << door.frame_p2.y);
  return door;
}

void TopologicalMap::MapImpl::observeDoorMessage (RegionId id, const Door& msg)
{
  if (region_graph_->regionType(id)!=DOORWAY) {
    throw NotDoorwayRegionException(id);
  }

  Door msg2(msg);
  msg2.frame_p1 = transform(transform_,msg.frame_p1);
  msg2.frame_p2 = transform(transform_,msg.frame_p2);
  msg2.door_p1 = transform(transform_,msg.door_p1);
  msg2.door_p2 = transform(transform_,msg.door_p2);
  msg2.handle = transform(transform_,msg.handle);

  RegionDoorMap::iterator iter = region_door_map_.find(id);
  if (iter==region_door_map_.end()) {
    region_door_map_[id] = DoorInfoPtr(new DoorInfo(door_open_prior_prob_));
    region_door_map_[id]->observeDoorMessage(msg2);
  }
  else {
    iter->second->observeDoorMessage(msg2);
  }
}

void TopologicalMap::MapImpl::observeDoorTraversal (RegionId id, bool succeeded, const Time& stamp)
{
  if (region_graph_->regionType(id)!=DOORWAY) {
    throw NoDoorInRegionException(id);
  }
  RegionDoorMap::iterator iter=region_door_map_.find(id);
  ROS_ASSERT_MSG (iter!=region_door_map_.end(), "Unexpectedly couldn't find door info for region %u", id);
  DoorInfoPtr door = iter->second;
  if (stamp < door->getLastObsTime()) {
    throw ObservationOutOfSequenceException(stamp, door->getLastObsTime());
  }
  door->setOpenProb (succeeded ? 1.0 : 0.0);
  door->setLastObsTime (stamp);
  setDoorCosts(stamp);
}


double TopologicalMap::MapImpl::doorOpenProb (RegionId id, const Time& stamp)
{
  if (region_graph_->regionType(id)!=DOORWAY) {
    throw NoDoorInRegionException(id);
  }
  RegionDoorMap::iterator iter=region_door_map_.find(id);
  ROS_ASSERT_MSG (iter!=region_door_map_.end(), "Unexpectedly couldn't find door info for region %u", id);
  DoorInfoPtr door = iter->second;
  if (stamp < door->getLastObsTime()) {
    throw ObservationOutOfSequenceException(stamp, door->getLastObsTime());
  }
  double d = (stamp - door->getLastObsTime()).toSec();
  double w = exp(-d/door_reversion_rate_);
  return w*door->getOpenProb()+(1-w)*door_open_prior_prob_;
}

bool TopologicalMap::MapImpl::isDoorOpen (RegionId id, const Time& stamp)
{
  return doorOpenProb(id, stamp)>DOOR_OPEN_PROB_THRESHOLD;
}


// If this region is a door, set costs between connectors adjacent to it based on probability of it being open
void TopologicalMap::MapImpl::setDoorCost (RegionId id, const Time& t)
{
  typedef vector<ConnectorId> ConnectorVector;
  if (regionType(id)==DOORWAY) {
    
    double open_prob = doorOpenProb(id, t);
    double locked_cost = (1-open_prob)*locked_door_cost_;
    ConnectorVector connectors = adjacentConnectors(id);

    // Loop over pairs of connectors adjacent to door
    for (ConnectorVector::iterator c1=connectors.begin(); c1!=connectors.end(); ++c1) 
      for (ConnectorVector::iterator c2=connectors.begin(); c2!=connectors.end(); ++c2)
        if (*c1!=*c2) {
          Cell2D cell1 = containingCell(transformPoint(connectorPosition(*c1)));
          Cell2D cell2 = containingCell(transformPoint(connectorPosition(*c2)));
          bool found;
          double distance;
          tie (found, distance) = grid_graph_->distanceBetween(cell1, cell2);
          double cost = locked_cost + (found ? distance*resolution_ : locked_door_cost_);
          roadmap_->setCost(*c1, *c2, cost);
        }
  }
}


void TopologicalMap::MapImpl::setDoorCosts (const Time& t)
{
  RegionIdSet regions=allRegions();
  for_each(regions.begin(), regions.end(), bind(&TopologicalMap::MapImpl::setDoorCost, this, _1, t));
}


/****************************************
 * Outlets
 ****************************************/

OutletInfo& getOutletInfo (OutletVector& outlets, const OutletId id)
{
  if (id>=outlets.size())
    throw UnknownOutletException(id);
  else
    return outlets[id];
}

const OutletInfo& getOutletInfo (const OutletVector& outlets, const OutletId id)
{
  if (id>=outlets.size())
    throw UnknownOutletException(id);
  else
    return outlets[id];
}

struct CloserTo
{
  CloserTo(const double x, const double y) : x(x), y(y) {}
  bool operator() (const OutletInfo& o1, const OutletInfo& o2) 
  {
    return sqDist(o1.x, o1.y)<sqDist(o2.x,o2.y);
  }
  double sqDist(double x2, double y2)
  {
    return (pow(x2-x,2)+pow(y2-y,2));
  }
  double x, y;
};
  

OutletId TopologicalMap::MapImpl::nearestOutlet (const Point2D& p) const
{
  Point2D p2 = transformPoint(p);
  //For now, assume outlet data is untransformed
  //Point2D p2=p;
  OutletVector::const_iterator pos = min_element(outlets_.begin(), outlets_.end(), CloserTo(p2.x, p2.y));
  if (pos==outlets_.end()) 
    throw NoOutletException(p2.x, p2.y);
  return pos-outlets_.begin();
}

void TopologicalMap::MapImpl::observeOutletBlocked (const OutletId id)
{
  getOutletInfo(outlets_,id).blocked=true;
}

OutletInfo TopologicalMap::MapImpl::outletInfo (const OutletId id) const
{
  return getOutletInfo(outlets_,id);
  //return transformOutlet(transform_.inverse(), getOutletInfo(outlets_,id)); // makes a copy
}

OutletId TopologicalMap::MapImpl::addOutlet (const OutletInfo& outlet)
{
  outlets_.push_back(transformOutlet(transform_.inverse(), outlet));
  return outlets_.size()-1;
}

OutletIdSet TopologicalMap::MapImpl::allOutlets () const
{
  // For now, outlet ids are 0,...,num_outlets-1
  return OutletIdSet(Counter(0), Counter(outlets_.size()));
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
            tie(found, distance) = grid_graph_->distanceBetween(cell, other_connector_cell);
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
  Point2D p2 = transformPoint(p);
  Cell2D c=containingCell(p2);
  ROS_DEBUG_STREAM_NAMED ("point_cell_map", "point " << p2.x << ", " << p2.y << " maps to cell " << c);
  return containingRegion(c);
}


ConnectorId TopologicalMap::MapImpl::pointConnector (const Point2D& p) const
{
  return roadmap_->pointId(transformPoint(p));
}



Point2D TopologicalMap::MapImpl::connectorPosition (const ConnectorId id) const
{
  return inverseTransformPoint(roadmap_->connectorPoint(id));
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
  TemporaryRoadmapNode start(this, transformPoint(p1));
  TemporaryRoadmapNode goal(this, transformPoint(p2));
  return roadmap_->costBetween(start.id, goal.id);
}

vector<pair<ConnectorId, double> > TopologicalMap::MapImpl::connectorCosts (const Point2D& p1, const Point2D& p2)
{
  TemporaryRoadmapNode start(this, transformPoint(p1));
  TemporaryRoadmapNode goal(this, transformPoint(p2));
  return roadmap_->connectorCosts(start.id, goal.id);
}

vector<pair<ConnectorId, double> > TopologicalMap::MapImpl::connectorCosts (const Point2D& p1, const Point2D& p2, const Time& t)
{
  setDoorCosts(t);
  TemporaryRoadmapNode start(this, transformPoint(p1));
  TemporaryRoadmapNode goal(this, transformPoint(p2));
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

  for (uint i=0; i<connector_ids.size(); ++i) {
    if (costs[i].first) {
      double cost=m->resolution_*costs[i].second;
      ROS_DEBUG_STREAM_NAMED ("temp_node", "Connecting to " << connector_ids[i] << " with cost " << cost);
      m->roadmap_->setCost(id, connector_ids[i], cost);
    }
  }
}


TopologicalMap::MapImpl::TemporaryRoadmapNode::~TemporaryRoadmapNode ()
{
  map->roadmap_->removeNode(id);
}


 
/************************************************************
 * TopologicalMap ops are forwarded to MapImpl
 ************************************************************/

TopologicalMap::TopologicalMap (const OccupancyGrid& grid, double resolution, double door_open_prior_prob, 
                                double door_reversion_rate, double locked_door_cost, const Transform& transform) 
  : map_impl_(new MapImpl(grid, resolution, door_open_prior_prob, door_reversion_rate, locked_door_cost, transform))
{
}

TopologicalMap::TopologicalMap(istream& str, double door_open_prior_prob, double door_reversion_rate, double locked_door_cost, const Transform& transform) 
  : map_impl_(new MapImpl(str, door_open_prior_prob, door_reversion_rate, locked_door_cost, transform))
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

Door TopologicalMap::regionDoor (RegionId id) const
{
  return map_impl_->regionDoor(id);
}

void TopologicalMap::observeDoorMessage (RegionId id, const Door& msg)
{
  map_impl_->observeDoorMessage(id,msg);
}

void TopologicalMap::observeDoorTraversal (RegionId id, bool succeeded, const Time& stamp)
{
  map_impl_->observeDoorTraversal (id, succeeded, stamp);
}

double TopologicalMap::doorOpenProb (RegionId id, const Time& stamp)
{
  return map_impl_->doorOpenProb(id, stamp);
}

bool TopologicalMap::isDoorOpen (RegionId id, const Time& stamp)
{
  return map_impl_->isDoorOpen(id, stamp);
}

OutletId TopologicalMap::nearestOutlet (const Point2D& p) const
{
  return map_impl_->nearestOutlet(p);
}

void TopologicalMap::observeOutletBlocked (const OutletId id)
{
  map_impl_->observeOutletBlocked(id);
}

OutletInfo TopologicalMap::outletInfo (const OutletId id) const
{
  return map_impl_->outletInfo(id);
}


OutletId TopologicalMap::addOutlet (const OutletInfo& outlet)
{
  return map_impl_->addOutlet(outlet);
}

OutletIdSet TopologicalMap::allOutlets () const
{ 
  return map_impl_->allOutlets();
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

vector<pair<ConnectorId, double> > TopologicalMap::connectorCosts (const Point2D& p1, const Point2D& p2, const Time& t)
{
  return map_impl_->connectorCosts(p1, p2, t);
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
  
  
void TopologicalMap::writeGridAndOutletData (const string& filename) const
{
  map_impl_->writeGridAndOutletData (filename);
}
  
  



} // namespace topological_map
