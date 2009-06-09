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
#include <queue>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <ros/console.h>
#include <ros/assert.h>
#include <tinyxml/tinyxml.h>
#include <tf/transform_datatypes.h>
#include <topological_map/exception.h>


#define foreach BOOST_FOREACH

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
using boost::shared_ptr;
using std::endl;
using std::map;
using std::vector;
using std::set;
using std::min;
using std::stringstream;
using std::max;
using std::queue;
using std::pair;
using ros::Time;
using tf::Transform;
using tf::Vector3;
using tf::Quaternion;
using tf::Pose;
using tf::Point;
using robot_msgs::Point32;

namespace topological_map
{

typedef boost::counting_iterator<unsigned int> Counter;

/************************************************************
 * Utility
 ************************************************************/

ostream& operator<< (ostream& str, const Point2D& p)
{
  str << "(" << p.x << ", " << p.y << ")";
  return str;
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
  return pointOnMap(p) ? isObstacle(containingCell(p)) : false;
}

bool TopologicalMap::MapImpl::isObstacle (const Cell2D& cell) const
{
  if (cellOnMap(cell)) 
    return (*grid_)[cell.r][cell.c];
  else 
    throw UnknownGridCellException(cell);
}

double TopologicalMap::MapImpl::getResolution () const
{
  return resolution_;
}

bool TopologicalMap::MapImpl::cellOnMap (const Cell2D& cell) const
{
  return (cell.r>=0) && (cell.c>=0) && (cell.r<(int)numRows(*grid_)) && (cell.c<(int)numCols(*grid_));
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



RegionPtr TopologicalMap::MapImpl::squareRegion (const Point2D& p, const double radius) const
{
  ROS_DEBUG_STREAM_NAMED ("square_region", "Computing square region for point " << p << " with radius " << radius);
  int cell_radius = floor(radius/resolution_);
  MutableRegionPtr region(new Region);
  Cell2D cell = containingCell(p);
  for (int r = max(0,cell.r-cell_radius); r<=cell.r+cell_radius; ++r) {
    for (int c = max(0,cell.c-cell_radius); c<=cell.c+cell_radius; ++c) {
      region->insert(Cell2D(r,c));
      ROS_DEBUG_STREAM_NAMED ("square_region", " inserting " << Cell2D(r,c));
    }
  }
  return region;
}

bool closerToObstacles (const ObstacleDistanceArray& distances, const Cell2D& c1, const Cell2D& c2)
{
  double d1 = distances[c1.r][c1.c];
  double d2 = distances[c2.r][c2.c];
  return d1<d2;
}
  

Point2D TopologicalMap::MapImpl::centerPoint (const Cell2D& cell) const
{
  return cellCenter(cell, resolution_);
}


void TopologicalMap::MapImpl::readOutletApproachOverrides (const string& filename)
{
  TiXmlDocument doc(filename);
  if (!doc.LoadFile()) {
    throw XmlException(filename, "Unable to open");
  }
  
  TiXmlHandle root=TiXmlHandle(&doc).FirstChildElement();
  const string& root_name = root.Element()->ValueStr();
  if (root_name!=string("outlet_approach_overrides"))
  {
    throw XmlException(filename, "Root was not outlet_approach_overrides");
  }
  TiXmlElement* next_override;
  for (next_override=root.FirstChild().Element(); next_override; next_override=next_override->NextSiblingElement())
  {
    TiXmlHandle override_handle(next_override);
    OutletId id = atoi(override_handle.FirstChildElement("id").FirstChild().Node()->Value());
    double x = atof(override_handle.FirstChildElement("approach_point").FirstChildElement("x").FirstChild().Node()->Value());
    double y = atof(override_handle.FirstChildElement("approach_point").FirstChildElement("y").FirstChild().Node()->Value());
    Point2D p(x,y);
    outlet_approach_overrides_[id] = p;
    ROS_DEBUG_STREAM_NAMED("outlet_approach", "Adding override of outlet " << id << " to " << p);
  }
}


void TopologicalMap::MapImpl::readDoorApproachOverrides (const string& filename)
{
  TiXmlDocument doc(filename);
  if (!doc.LoadFile()) {
    throw XmlException(filename, "Unable to open");
  }
  
  TiXmlHandle root=TiXmlHandle(&doc).FirstChildElement();
  const string& root_name = root.Element()->ValueStr();
  if (root_name!=string("door_approach_overrides"))
  {
    throw XmlException(filename, "Root was not door_approach_overrides");
  }
  TiXmlElement* next_override;
  for (next_override=root.FirstChild().Element(); next_override; next_override=next_override->NextSiblingElement())
  {
    TiXmlHandle override_handle(next_override);
    ConnectorId id = atoi(override_handle.FirstChildElement("id").FirstChild().Node()->Value());
    double x = atof(override_handle.FirstChildElement("approach_point").FirstChildElement("x").FirstChild().Node()->Value());
    double y = atof(override_handle.FirstChildElement("approach_point").FirstChildElement("y").FirstChild().Node()->Value());
    Point2D p(x,y);
    door_approach_overrides_[id] = p;
    ROS_DEBUG_STREAM_NAMED("door_approach", "Adding override of door " << id << " to " << p);
  }
}




double TopologicalMap::MapImpl::getCost (const ConnectorId i, const ConnectorId j) const
{
  return roadmap_->getCost(i,j);
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

template <typename T>
TiXmlText* makeXmlTextElt(T x)
{
  stringstream out;
  out << x;
  return new TiXmlText(out.str());
}

template <typename T>
TiXmlElement* makeXmlElt (const char* name, T x)
{
  TiXmlElement* elt = new TiXmlElement(name);
  elt->LinkEndChild(makeXmlTextElt(x));
  return elt;
}

void TopologicalMap::MapImpl::writeGridAndOutletData (const string& filename) const
{

  ROS_DEBUG_NAMED ("io", "About to write gui input data");
  TiXmlDocument doc;
  TiXmlElement* top = new TiXmlElement("map_and_outlets");
  doc.LinkEndChild(top);
  
  const uint nr = numRows(*grid_);
  const uint nc = numCols(*grid_);

  top->LinkEndChild(makeXmlElt("resolution", resolution_));
  top->LinkEndChild(makeXmlElt("rows", nr));
  top->LinkEndChild(makeXmlElt("columns", nc));

  TiXmlElement* outlets = new TiXmlElement("outlets");
  top->LinkEndChild(outlets);
  foreach (uint id, allOutlets()) {
    TiXmlElement* outlet = new TiXmlElement("outlet");
    outlets->LinkEndChild(outlet);

    OutletInfo outlet_info = outletInfo(id);
    Point2D p(outlet_info.x, outlet_info.y);
    Cell2D cell = containingCell(p);
    outlet->LinkEndChild(makeXmlElt("row", cell.r));
    outlet->LinkEndChild(makeXmlElt("column", cell.c));
  }

  TiXmlElement* map = new TiXmlElement("grid");
  for (uint r=0; r<nr; ++r) {
    TiXmlElement* row = new TiXmlElement("row");
    map->LinkEndChild(row);
    for (uint c=0; c<nc; ++c) {
      TiXmlElement* cell = new TiXmlElement((*grid_)[r][c] ? "o" : "f");
      row->LinkEndChild(cell);
    }
  }
  top->LinkEndChild(map);

  doc.SaveFile(filename);
  ROS_DEBUG_NAMED ("io", "Done writing gui input data");

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
  queue<Cell2D> q;
  
  for (int r=0; r<num_rows; ++r) {
    for (int c=0; c<num_cols; ++c) {
      if ((*grid)[r][c]) {
        distances[r][c] = 0;
        Cell2D cell(r,c);
        q.push(cell);
      }
      else {
        distances[r][c] = (num_rows+1)*(num_cols+1);
      }
    }
  }
  
  while (!q.empty()) {
    Cell2D cell = q.front();
    q.pop();
    for (uint vertical=0; vertical<2; ++vertical) {
      for (int mult=-1; mult<=1; mult+=2) {
        int dr=mult*(vertical ? 1 : 0);
        int dc=mult*(vertical ? 0 : 1);
        int r1 = min(num_rows-1, max(0,cell.r+dr));
        int c1 = min(num_cols-1, max(0,cell.c+dc));
        if (distances[r1][c1]>distances[cell.r][cell.c]+1) {
          distances[r1][c1] = distances[cell.r][cell.c]+1;
          q.push(Cell2D(r1,c1));
        }
      }
    }
  }
  return distances;
}
    

TopologicalMap::MapImpl::MapImpl(const OccupancyGrid& grid, double resolution, double door_open_prior_prob, 
                                 double door_reversion_rate, double locked_door_cost) :
  grid_(new OccupancyGrid(grid)), obstacle_distances_(computeObstacleDistances(grid_)), region_graph_(new RegionGraph), 
  roadmap_(new Roadmap), grid_graph_(new GridGraph(grid_)), door_open_prior_prob_(door_open_prior_prob), 
  door_reversion_rate_(door_reversion_rate), locked_door_cost_(locked_door_cost), resolution_(resolution)
{
}

TopologicalMap::MapImpl::MapImpl (istream& str, double door_open_prior_prob, double door_reversion_rate,
                                  double locked_door_cost) :
  grid_(readGrid(str)), obstacle_distances_(computeObstacleDistances(grid_)), region_graph_(readRegionGraph(str)), 
  roadmap_(readRoadmap(str)), grid_graph_(new GridGraph(grid_)),
  region_connector_map_(readRegionConnectorMap(str)), door_open_prior_prob_(door_open_prior_prob),
  door_reversion_rate_(door_reversion_rate), locked_door_cost_(locked_door_cost), 
  region_door_map_(readRegionDoorMap(str, door_open_prior_prob_)), 
  outlets_(readOutlets(str)), resolution_(readResolution(str))
{
}



/****************************************
 * RegionGraph const ops
 ****************************************/

RegionId TopologicalMap::MapImpl::containingRegion (const Cell2D& c) const
{
  set<RegionId> containing_regions;
  set<Cell2D> seen;
  queue<Cell2D> queue ;
  queue.push(c);
  ROS_DEBUG_STREAM_NAMED ("containing_region", "Looking for containing region of " << c);

  while (!queue.empty()) {
    const Cell2D cell = queue.front();
    queue.pop();

    if (cellOnMap(cell) && seen.find(cell)==seen.end()) {
      seen.insert(cell);

      if (isObstacle(cell)) {
        for (int vertical=0; vertical<2; ++vertical) {
          for (int mult=-1; mult<=1; mult+=2) {
            const int dr = mult * (vertical ? 1 : 0);
            const int dc = mult * (vertical ? 0 : 1);
            queue.push(Cell2D(cell.r+dr, cell.c+dc));
          }
        }
        ROS_DEBUG_STREAM_NAMED ("containing_region", " Adding neighbors of obstacle cell " << cell);
      }
      
      else {
        RegionId region = region_graph_->containingRegion(cell);

        // The old way found all such regions and made sure there was just one
        // Now we just return the first one we find (which will be the closest)
        //containing_regions.insert(region);
        //ROS_DEBUG_STREAM_NAMED ("containing_region", " Adding containing region " << region << " of " << cell);

        return region;
        
      }
    }
    
    else {
      ROS_DEBUG_STREAM_NAMED ("containing_region", " Ignoring cell " << cell);
    }
  }

  if (containing_regions.size()!=1) 
    throw NoContainingRegionException(c, containing_regions.size());
  else
    return *(containing_regions.begin());
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
  return iter->second->getDoorMessage();
}

void TopologicalMap::MapImpl::observeDoorMessage (RegionId id, const Door& msg)
{
  if (region_graph_->regionType(id)!=DOORWAY) {
    throw NotDoorwayRegionException(id);
  }

  RegionDoorMap::iterator iter = region_door_map_.find(id);
  if (iter==region_door_map_.end()) {
    region_door_map_[id] = DoorInfoPtr(new DoorInfo(door_open_prior_prob_));
    region_door_map_[id]->observeDoorMessage(msg);
  }
  else {
    iter->second->observeDoorMessage(msg);
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
          Cell2D cell1 = containingCell(connectorPosition(*c1));
          Cell2D cell2 = containingCell(connectorPosition(*c2));
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

Point2D doorApproachCenter (const Door& door, const Point2D connector, const double r)
{
  const double x1 = door.frame_p1.x;
  const double x2 = door.frame_p2.x;
  const double y1 = door.frame_p1.y;
  const double y2 = door.frame_p2.y;

  // Compute the frame direction and normal
  const double cx = (x1+x2)/2;
  const double cy = (y1+y2)/2;
  const double dx = x2-x1;
  const double dy = y2-y1;
  const double nx = -dy;
  const double ny = dx;
  const double l = sqrt(nx*nx+ny*ny);

  // Compute connector offset
  const double ox = connector.x-cx;
  const double oy = connector.y-cy;
  
  // Set normal direction depending on whether normal points in same direction as connector
  const double mult = (nx*ox+ny*oy > 0) ? r/l : -r/l;

  return Point2D(cx+nx*mult, cy+ny*mult);
}

  

  

Point2D TopologicalMap::MapImpl::doorApproachPosition (const ConnectorId id, const double r1, const double r2) const
{
  map<ConnectorId, Point2D>::const_iterator pos = door_approach_overrides_.find(id);
  if (pos!=door_approach_overrides_.end()) {
    ROS_DEBUG_STREAM_NAMED ("door_approach", " overriding door approach position for " << id << " to " << pos->second);
    return pos->second;
  }
  else {
  

  // Get door and connector info from map
  const RegionPair adjacent_regions = adjacentRegions(id);
  const RegionId doorRegion = regionType(adjacent_regions.first) == DOORWAY ? adjacent_regions.first : adjacent_regions.second;
  const Door door = regionDoor(doorRegion);
  const Point2D connector = connectorPosition(id);
  
  // Compute center of approach positions, then find the one that's furthest from obstacles
  RegionPtr region = squareRegion(doorApproachCenter(door, connector, r1), r2);
  Region::iterator pos = max_element(region->begin(), region->end(), bind(closerToObstacles, ref(obstacle_distances_), _1, _2));
  
  if (pos==region->end() || obstacle_distances_[pos->r][pos->c]==0) 
    throw NoDoorApproachPositionException(id,r1,r2);

  ROS_DEBUG_STREAM_NAMED ("door_approach", " closest point to door connector " << id << " at " << connector << " is " << centerPoint(*pos));
  return centerPoint(*pos);

  }
}



/****************************************
 * Outlets
 ****************************************/

OutletInfo& getOutletInfo (OutletVector& outlets, const OutletId id)
{
  if ((id==0)||(id>outlets.size()))
    throw UnknownOutletException(id);
  else
    return outlets[id-1];
}

const OutletInfo& getOutletInfo (const OutletVector& outlets, const OutletId id)
{
  if ((id==0)||(id>outlets.size()))
    throw UnknownOutletException(id);
  else
    return outlets[id-1];
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
  OutletVector::const_iterator pos = min_element(outlets_.begin(), outlets_.end(), CloserTo(p.x, p.y));
  if (pos==outlets_.end()) 
    throw NoOutletException(p.x, p.y);
  return 1+(pos-outlets_.begin());
}

void TopologicalMap::MapImpl::observeOutletBlocked (const OutletId id)
{
  getOutletInfo(outlets_,id).blocked=true;
}

OutletInfo TopologicalMap::MapImpl::outletInfo (const OutletId id) const
{
  return getOutletInfo(outlets_,id);
}

OutletId TopologicalMap::MapImpl::addOutlet (const OutletInfo& outlet)
{
  outlets_.push_back(outlet);
  return outlets_.size();
}

OutletIdSet TopologicalMap::MapImpl::allOutlets () const
{
  // For now, outlet ids are 1,...,num_outlets
  return OutletIdSet(Counter(1), Counter(1+outlets_.size()));
}



Point2D outletApproachCenter(const OutletInfo& outlet, const double distance)
{
  
  Pose tf_pose(Quaternion(outlet.qx, outlet.qy, outlet.qz, outlet.qw),
               Vector3(outlet.x, outlet.y, outlet.z));

  Point point(distance, 0, 0);
  Point new_origin = tf_pose*point;
  tf_pose.setOrigin(new_origin);
  Point2D approach_point(tf_pose.getOrigin().x(), tf_pose.getOrigin().y());

  ROS_DEBUG_STREAM_NAMED ("outlet_approach", "Approach point for outlet at " << outlet.x << ", " << outlet.y << " is " << approach_point);
  return approach_point;
}
 
 

Point2D TopologicalMap::MapImpl::outletApproachPosition (const OutletId id, const double r1, const double r2) const
{
  map<OutletId, Point2D>::const_iterator pos = outlet_approach_overrides_.find(id);
  if (pos==outlet_approach_overrides_.end()) {
    RegionPtr region = squareRegion(outletApproachCenter(outletInfo(id), r1), r2);
    Region::iterator pos = max_element(region->begin(), region->end(), bind(closerToObstacles, ref(obstacle_distances_), _1, _2));
  
    if (pos==region->end() || obstacle_distances_[pos->r][pos->c]==0) 
      throw NoApproachPositionException(id,r1,r2);

    ROS_DEBUG_STREAM_NAMED ("outlet_approach", " closest cell is " << *pos);
    return centerPoint(*pos);
  }
  else {
    ROS_DEBUG_STREAM_NAMED ("outlet_approach", " overriding approach point for " << id << " to " << pos->second);
    return pos->second;
  }
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

RegionId TopologicalMap::MapImpl::addRegion (const RegionPtr region, const int type, bool update_distances)
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

  if (update_distances)
    updateDistances (region_id);

  return region_id;
}


void TopologicalMap::MapImpl::updateDistances (const RegionId region_id)
{
  ROS_DEBUG_STREAM_NAMED ("update_distances", "Updating connector distances for region " << region_id);
  RegionIdVector neighbors = region_graph_->neighbors(region_id);


  // Compute the distance from the new connector to all connectors touching either of the adjacent regions
  // 1. Loop over neighbors of new region (and corresponding connector)
  for (RegionIdVector::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter) {

    RegionId r1 = min(region_id, *iter);
    RegionId r2 = max(region_id, *iter);

    RegionConnectorMap::iterator rc_pos = region_connector_map_.find(RegionPair(r1,r2));
    ROS_ASSERT(rc_pos!=region_connector_map_.end());
    ConnectorId connector_id = rc_pos->second.get<0>();
    Cell2D cell = rc_pos->second.get<1>();

    vector<Cell2D> other_connector_cells;
    vector<ConnectorId> other_connectors;


    // 2. Loop over which of the two adjacent regions to consider
    for (char first_region=0; first_region<2; ++first_region) {
      RegionId r = first_region ? r1 : r2;
      Region connector_region = *(region_graph_->regionCells(r));
      connector_region.insert(cell);
      ROS_DEBUG_STREAM_NAMED ("add_region", "Considering connector " << connector_id << " in region " << r);
      
      RegionIdVector connector_neighbors = region_graph_->neighbors(r);

      // Loop over which of the neighboring regions (and corresponding connectors) to find path to
      for (RegionIdVector::iterator region_iter=connector_neighbors.begin(); region_iter!=connector_neighbors.end(); ++region_iter) {

        // Proceed only if the regions and connectors are different
        if (r!=*region_iter) {
          RegionConnectorMap::const_iterator pos = region_connector_map_.find(RegionPair(min(*region_iter,r), max(*region_iter,r)));
          ROS_ASSERT_MSG(pos!=region_connector_map_.end(), "Failed to find connector between regions %d and %d", r, *region_iter);
          ConnectorId other_connector = pos->second.get<0>();

          if (other_connector!=connector_id) {
            ROS_DEBUG_STREAM_NAMED("add_region", "Considering other connector " << other_connector << " in region " << *region_iter);

            other_connectors.push_back(other_connector);
            other_connector_cells.push_back(pos->second.get<1>());
          }
        }
      }
    }

    // 3. Do the shortest path computations all at once for the connector in the outer loop
    vector<pair<bool,double> > costs = grid_graph_->singleSourceCosts(cell, other_connector_cells);

    for (uint i=0; i<other_connectors.size(); i++) 
      if (costs[i].first) 
        roadmap_->setCost(other_connectors[i], connector_id, resolution_*costs[i].second);

  }
  ROS_DEBUG_STREAM_NAMED ("update_distances", "   Done updating connector distances for region " << region_id);

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

void TopologicalMap::MapImpl::recomputeConnectorDistances ()
{
  foreach (RegionId id, allRegions()) {
    updateDistances(id);
  }
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

set<ConnectorId> TopologicalMap::MapImpl::allConnectors () const
{
  set<ConnectorId> connectors;
  foreach (RegionId region, allRegions()) {
    foreach (ConnectorId connector, adjacentConnectors(region)) {
      connectors.insert(connector);
    }
  }
  return connectors;
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
  
  RegionConnectorMap::const_iterator start = region_connector_map_.begin(), end = region_connector_map_.end();
  RegionConnectorMap::const_iterator iter = find_if(start, end, HasId(id));

  if (iter==end) {
    // To deal with temporary connectors that aren't in the region connector map
    RegionId r = containingRegion(connectorPosition(id));
    return RegionPair(r, r);
  }

  else {
    return iter->first;
  }
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
  setGoal(centerPoint(c));
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
  // Hacky: we're hashing on a pair of floats
  pair<Point2D, Point2D> pair(p1 < p2 ? p1 : p2, p1 < p2 ? p2 : p1);
  if (roadmap_distance_cache_.find(pair)==roadmap_distance_cache_.end()) {

    TemporaryRoadmapNode start(this, p1);
    TemporaryRoadmapNode goal(this, p2);
    ReachableCost rcost = roadmap_->costBetween(start.id, goal.id);
    double cost = rcost.second;
    ROS_DEBUG_STREAM_NAMED ("roadmap_shortest_path", "Adding roadmap distance cache entry " << p1 << " " << p2 << " " << cost);
    roadmap_distance_cache_[pair] = cost;
    return rcost;
  }
  else {
    double cached_cost = roadmap_distance_cache_[pair];
    ROS_DEBUG_STREAM_NAMED ("roadmap_shortest_path", "Using cached roadmap cost " << cached_cost);
    return ReachableCost(true, cached_cost);
  }
                            
}

vector<pair<ConnectorId, double> > TopologicalMap::MapImpl::connectorCosts (const Point2D& p1, const Point2D& p2)
{
  TemporaryRoadmapNode start(this, p1);
  TemporaryRoadmapNode goal(this, p2);
  return roadmap_->connectorCosts(start.id, goal.id);
}

vector<pair<ConnectorId, double> > TopologicalMap::MapImpl::connectorCosts (const Point2D& p1, const Point2D& p2, const Time& t)
{
  setDoorCosts(t);
  TemporaryRoadmapNode start(this, p1);
  TemporaryRoadmapNode goal(this, p2);
  return roadmap_->connectorCosts(start.id, goal.id);
}


bool TopologicalMap::MapImpl::connectorsTouchSameRegion (const ConnectorId c1, const ConnectorId c2, const ConnectorId c3) const
{
  RegionPair r1 = adjacentRegions(c1);
  RegionPair r2 = adjacentRegions(c2);
  RegionPair r3 = adjacentRegions(c3);

  RegionId r;
  if ((r1.first == r2.first) || (r1.first == r2.second))
    r = r1.first;
  else if ((r1.second == r2.first) || (r1.second == r2.second))
    r = r1.second;
  else 
    return false;

  return (r == r3.first) || (r == r3.second);
}

  



ConnectorIdVector TopologicalMap::MapImpl::shortestConnectorPath (const Point2D& p1, const Point2D& p2)
{
  TemporaryRoadmapNode start(this, p1);
  TemporaryRoadmapNode goal(this, p2);

  if (containingRegion(p1)==containingRegion(p2)) {
    ConnectorIdVector v;
    v.push_back(start.id);
    v.push_back(goal.id);
    return v;
  }
  else {
    ConnectorIdVector path = roadmap_->shortestPath(start.id, goal.id);
    ConnectorIdVector pruned_path;
    uint length = path.size();
    for (uint i=0; i<length; i++) 
      if ((i==0) || (i==length-1) || !connectorsTouchSameRegion(path[i-1], path[i], path[i+1]))
        pruned_path.push_back(path[i]);
    return pruned_path;
  }
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
  //RegionIsolator i(m->grid_graph_.get(), *(m->regionCells(r)));

  vector<ConnectorDesc> connector_descs = m->adjacentConnectorCells(r);

  vector<ConnectorId> connector_ids(connector_descs.size());
  vector<Cell2D> connector_cells(connector_descs.size());
  transform(connector_descs.begin(), connector_descs.end(), connector_ids.begin(), getId);
  transform(connector_descs.begin(), connector_descs.end(), connector_cells.begin(), getCell);

  // vector<ReachableCost> costs = m->grid_graph_->singleSourceCosts(cell, connector_cells);

  for (uint i=0; i<connector_ids.size(); ++i) {
    Point2D p2 = m->connectorPosition(connector_ids[i]);
    m->roadmap_->setCost(id, connector_ids[i], sqrt(pow(p2.x-p.x,2)+pow(p2.y-p.y,2)));
//     if (costs[i].first) {
//       double cost=m->resolution_*costs[i].second;
//       ROS_DEBUG_STREAM_NAMED ("temp_node", "Connecting to " << connector_ids[i] << " with cost " << cost);
//       m->roadmap_->setCost(id, connector_ids[i], cost);
//     }
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
                                double door_reversion_rate, double locked_door_cost)
  : map_impl_(new MapImpl(grid, resolution, door_open_prior_prob, door_reversion_rate, locked_door_cost))
{
}

TopologicalMap::TopologicalMap(istream& str, double door_open_prior_prob, double door_reversion_rate, double locked_door_cost)
  : map_impl_(new MapImpl(str, door_open_prior_prob, door_reversion_rate, locked_door_cost))
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

Point2D TopologicalMap::doorApproachPosition (const ConnectorId id, const double r) const
{
  return map_impl_->doorApproachPosition(id, r, r/4);
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

Point2D TopologicalMap::outletApproachPosition (OutletId id, double r1, double r2) const
{
  return map_impl_->outletApproachPosition (id, r1, r2);
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

set<ConnectorId> TopologicalMap::allConnectors () const
{
  return map_impl_->allConnectors();
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

ConnectorIdVector TopologicalMap::shortestConnectorPath (const Point2D& p1, const Point2D& p2)
{
  return map_impl_->shortestConnectorPath(p1, p2);
}


RegionId TopologicalMap::addRegion(const RegionPtr region, const int type, bool recompute_distances) 
{
  return map_impl_->addRegion(region, type, recompute_distances);
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
  
void TopologicalMap::recomputeConnectorDistances ()
{
  map_impl_->recomputeConnectorDistances();
}

Point2D TopologicalMap::centerPoint (const Cell2D& cell) const
{
  return map_impl_->centerPoint(cell);
}

void TopologicalMap::readOutletApproachOverrides (const string& filename)
{
  map_impl_->readOutletApproachOverrides(filename);
}

void TopologicalMap::readDoorApproachOverrides (const string& filename)
{
  map_impl_->readDoorApproachOverrides(filename);
}

Cell2D TopologicalMap::containingCell (const Point2D& p) const
{
  return map_impl_->containingCell(p);
}


double TopologicalMap::getCost (const ConnectorId i, const ConnectorId j) const
{
  return map_impl_->getCost(i,j);
}

double TopologicalMap::getResolution () const
{
  return map_impl_->getResolution();
}


} // namespace topological_map
