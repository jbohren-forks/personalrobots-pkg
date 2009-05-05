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
 * Creating a topological map from known static map and door information
 *
 * \author Bhaskara Marthi
 *
 */


#include <topological_map/topological_map.h>
#include <utility>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ros/console.h>
#include <ros/assert.h>
#include <tinyxml/tinyxml.h>
#include <topological_map/grid_utils.h>
#include <topological_map/grid_graph.h>
#include <topological_map/exception.h>
#include <tf/transform_datatypes.h>


using std::min;
using std::max;
using std::ofstream;
using std::string;
using std::vector;
using boost::tie;
using boost::shared_ptr;
using boost::bind;
using robot_msgs::Door;

namespace topological_map
{

typedef vector<OutletInfo> OutletVector;

/************************************************************
 * Constants
 ************************************************************/

namespace 
{
const double LENGTH_ERROR_THRESHOLD = .05;
}

/************************************************************
 * Make topological map from doors
 ************************************************************/

// Projects points onto a fixed line segment
struct LineProjector
{
  LineProjector(const Point2D& p1, const Point2D& p2)
  {
    x1=p1.x;
    y1=p1.y;
    dx=p2.x-x1;
    dy=p2.y-y1;
    squared_dist = dx*dx+dy*dy;
  }

  pair<double, double> operator() (const Point2D& p)
  {
    double offset_x=p.x-x1;
    double offset_y=p.y-y1;
    
    double proportion = (offset_x*dx + offset_y*dy)/squared_dist;
    double dist = offset_x*offset_x + offset_y*offset_y - squared_dist*proportion*proportion;

    return std::make_pair(proportion, dist);
  }

  double x1, y1, dx, dy, squared_dist;
};


RegionPtr getDoorCells (const Point2D& p1, const Point2D& p2, double width, double length_error_threshold, double resolution)
{
  Cell2D c1 = pointToCell(p1,resolution);
  Cell2D c2 = pointToCell(p2,resolution);

  ROS_DEBUG_STREAM_NAMED ("ground_truth_map", "Generating door cells for cells " << c1 << " and " << c2);

  uint thickness = round(1+width/resolution);
  uint minrow = min(c1.r, c2.r);
  uint mincol = min(c1.c, c2.c);
  uint rmin = thickness <= minrow ? minrow - thickness : 0;
  uint rmax = max(c1.r, c2.r)+thickness;
  uint cmin = thickness <= mincol ? mincol - thickness : 0;
  uint cmax = max(c1.c, c2.c)+thickness;

  MutableRegionPtr region(new Region);
  LineProjector project(p1, p2);
  
  // Loop over cells in a bounding box of the line segment p1->p2
  for (uint r=rmin; r<=rmax; ++r) {
    for (uint c=cmin; c<=cmax; ++c) {

      // If the center of the cell is close to the line segment formed by p1 and p2, add the cell
      Point2D p=cellCenter(Cell2D(r, c), resolution);
      double d, l;
      tie(l,d) = project(p);
      if ((d<width*width) && (l > -length_error_threshold) && (l < 1+length_error_threshold)) {
        region->insert(Cell2D(r,c));
      }
    }
  }
  return region;
}


struct DoorFramePoints
{
  DoorFramePoints (const Point2D& p1, const Point2D& p2) : p1(p1), p2(p2) {}
  Point2D p1, p2;
};  



struct GetDoorCells
{
  GetDoorCells (double resolution, double width) : resolution(resolution), width(width) {}
  RegionPtr operator() (const DoorFramePoints& d) { return getDoorCells(d.p1, d.p2, width, LENGTH_ERROR_THRESHOLD, resolution); }
  const double resolution, width;
};



typedef shared_ptr<RegionIsolator> IsolatorPtr;

struct CutoffDoors
{
  CutoffDoors (GridGraph& g, vector<IsolatorPtr>& v) : graph(g), v(v) {}
  void operator() (RegionPtr r) 
  { 
    v.push_back(IsolatorPtr(new RegionIsolator(&graph, *r)));
  }
    
  GridGraph& graph;
  vector<IsolatorPtr>& v;
};


struct Contains
{
  Contains (const Cell2D& cell) : cell(cell) {}
  bool operator() (RegionPtr region) 
  {
    return region->find(cell)!=region->end();
  }
  const Cell2D& cell;
};



Door initialDoorEstimate (const DoorFramePoints& frame_points)
{
  Door msg;
  msg.frame_p1.x = frame_points.p1.x;
  msg.frame_p1.y = frame_points.p1.y;
  msg.frame_p1.z = 0.0;
  msg.frame_p2.x = frame_points.p2.x;
  msg.frame_p2.y = frame_points.p2.y;
  msg.frame_p2.z = 0.0;

  msg.door_p1.x = frame_points.p1.x;
  msg.door_p1.y = frame_points.p1.y;
  msg.door_p1.z = 0.0;
  msg.door_p2.x = frame_points.p2.x;
  msg.door_p2.y = frame_points.p2.y;
  msg.door_p2.z = 0.0;

  msg.handle.x = .6;
  msg.handle.y = 0;
  msg.handle.z = 1.0;

  msg.hinge = -1;
  msg.rot_dir = -1;
  return msg;
}

typedef vector<RegionPtr> DoorRegionVector;
typedef vector<DoorFramePoints> DoorFrameVector;

struct AddRegions
{
  AddRegions (TopologicalMapPtr map, const DoorRegionVector& door_regions, const DoorFrameVector& door_info) : 
    map(map), door_regions(door_regions), door_info(door_info) {}

  void operator() (RegionPtr region)
  {
    if (!region->empty()) {
      Cell2D cell = *(region->begin());
      DoorRegionVector::const_iterator pos = find_if(door_regions.begin(), door_regions.end(), Contains(cell));
      RegionType type = pos==door_regions.end() ? OPEN : DOORWAY;
      RegionId id = map->addRegion(region, type);
      if (type == DOORWAY) {
        DoorFramePoints frame_points = door_info[pos-door_regions.begin()];
        map->observeDoorMessage(id, initialDoorEstimate(frame_points));
        Door msg = map->regionDoor(id);
        ROS_DEBUG_NAMED ("door_info", "Added door info for region %u at %f, %f and %f, %f", 
                         id, msg.frame_p1.x, msg.frame_p1.y, msg.frame_p2.x, msg.frame_p2.y);
      }
    }
  }

  TopologicalMapPtr map;
  const DoorRegionVector& door_regions;
  const DoorFrameVector& door_info;
};


TopologicalMapPtr groundTruthTopologicalMap (const OccupancyGrid& grid, const DoorFrameVector& door_info, const OutletVector& outlets,
                                             const double resolution, const double width)
{
  shared_ptr<OccupancyGrid> grid_ptr(new OccupancyGrid(grid));
  GridGraph g(grid_ptr);
  
  DoorRegionVector door_regions(door_info.size());
  transform(door_info.begin(), door_info.end(), door_regions.begin(), GetDoorCells(resolution, width));
  
  // This will cause the doors to be severed from the rest of the grid for the rest of this scope
  vector<IsolatorPtr> v;
  for_each (door_regions.begin(), door_regions.end(), CutoffDoors(g,v));

  ROS_DEBUG_NAMED("ground_truth_map", "Looking for connected components");
  vector<MutableRegionPtr> comps = g.connectedComponents();

  ROS_DEBUG_STREAM_NAMED("ground_truth_map", "Found " << comps.size() << " connected components. Creating map from grid");
  TopologicalMapPtr map(new TopologicalMap(grid, resolution));
  
  ROS_DEBUG_NAMED("ground_truth_map", "Adding %u outlets", outlets.size());
  for_each (outlets.begin(), outlets.end(), bind(&TopologicalMap::addOutlet, map, _1));

  ROS_DEBUG_NAMED("ground_truth_map", "Done creating map from grid.  Adding door regions");
  for_each (comps.begin(), comps.end(), AddRegions(map, door_regions, door_info));

  return map;
}


DoorFrameVector loadDoorsFromFile (const string& filename, const double resolution) 
{
  TiXmlDocument doc(filename);
  if (!doc.LoadFile()) {
    throw XmlException(filename, "Unable to open");
  }

  TiXmlHandle root=TiXmlHandle(&doc).FirstChildElement();
  const string& root_name=root.Element()->ValueStr();
  if (root_name!=string("doormap")) {
    throw XmlRootException(filename, root_name, string("doormap"));
  }

  TiXmlElement* next_door;
  DoorFrameVector doors;
  for (next_door=root.FirstChild().Element(); next_door; next_door=next_door->NextSiblingElement()) {
    TiXmlHandle door_handle(next_door); 

    TiXmlHandle hinge = door_handle.FirstChildElement("hinge");
    TiXmlHandle end = door_handle.FirstChildElement("end");
    
    uint c1 = atoi(hinge.FirstChildElement("row").FirstChild().Node()->Value());
    uint c2 = atoi(end.FirstChildElement("row").FirstChild().Node()->Value());
    uint r1 = atoi(hinge.FirstChildElement("column").FirstChild().Node()->Value());
    uint r2 = atoi(end.FirstChildElement("column").FirstChild().Node()->Value());
    
    Point2D p1 = cellToPoint(Cell2D(r1,c1), resolution);
    Point2D p2 = cellToPoint(Cell2D(r2,c2), resolution);
    
    doors.push_back(DoorFramePoints(p1,p2));
  }
  
  return doors;
}





OutletVector loadOutletsFromFile (const string& filename)
{
  TiXmlDocument doc(filename);
  if (!doc.LoadFile()) {
    throw XmlException(filename, "Unable to open");
  }

  TiXmlHandle root=TiXmlHandle(&doc).FirstChildElement();
  const string& root_name=root.Element()->ValueStr();
  if (root_name!=string("outlets")) {
    throw XmlRootException(filename, root_name, string("outlets"));
  }

  TiXmlElement* next_outlet;
  OutletVector outlets;
  for (next_outlet=root.FirstChild().Element(); next_outlet; next_outlet=next_outlet->NextSiblingElement())
  {
    TiXmlHandle outlet_handle(next_outlet);
    TiXmlHandle origin = outlet_handle.FirstChildElement("origin");
    TiXmlHandle sockets = outlet_handle.FirstChildElement("sockets");
    
    typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(" ");


    const string xyz_str = origin.ToElement()->Attribute("xyz");
    Tokenizer xyz_tok(xyz_str, sep);
    Tokenizer::iterator pos=xyz_tok.begin(), end=xyz_tok.end();
    ROS_ASSERT_MSG(pos!=end, "Invalid xyz element");
    const double x=atof((pos++)->c_str());
    ROS_ASSERT_MSG(pos!=end, "Invalid xyz element");
    const double y=atof((pos++)->c_str());
    ROS_ASSERT_MSG(pos!=end, "Invalid xyz element");
    const double z=atof((pos++)->c_str());

    const string rpy_str = origin.ToElement()->Attribute("rpy");
    Tokenizer rpy_tok(rpy_str, sep);
    pos=rpy_tok.begin(), end=rpy_tok.end();
    ROS_ASSERT_MSG(pos!=end, "Invalid rpy element");
    const double roll=atof((pos++)->c_str());
    ROS_ASSERT_MSG(pos!=end, "Invalid rpy element");
    const double pitch=atof((pos++)->c_str());
    ROS_ASSERT_MSG(pos!=end, "Invalid rpy element");
    const double yaw=atof((pos++)->c_str());

    tf::Quaternion q(yaw, pitch, roll);

    const uint size = atoi(sockets.ToElement()->Attribute("size"));
    const string color = sockets.ToElement()->Attribute("color");
    
    ROS_DEBUG_STREAM_NAMED ("load_outlets", "Read outlet with xyz " << x << ", " << y << ", " << z  << " orientation = " 
                            << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << " size = " << size << " color = " << color);

    outlets.push_back(OutletInfo(x,y,z,q.x(),q.y(),q.z(),q.w(),size,color));
  }

  return outlets;
}







} // namespace





namespace tmap=topological_map;
namespace po=boost::program_options;

int main(int argc, char** argv)
{

  string door_file("");
  string outlet_file("");
  string ppm_file("");
  string top_map_file("");
  string static_map_file("");
  double resolution(-1), width(-1);
  unsigned inflation_radius = 0;



  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("resolution,r", po::value<double>(&resolution), "Resolution of grid.  Required.")
    ("inflation_radius,i", po::value<unsigned>(&inflation_radius), "Inflation radius of obstacles (in gridcells).  Defaults to 0.")
    ("width,w", po::value<double>(&width), "Width of doors in metres.  Required.")
    ("static_map_file,m", po::value<string>(&static_map_file), "pgm file containing the static map.  Required.")
    ("ppm_output_file,p", po::value<string>(&ppm_file), "Name of .ppm output file.")
    ("topological_map_output_file,t", po::value<string>(&top_map_file), "Name of topological map output file")
    ("outlet_file,o", po::value<string>(&outlet_file), "xml file containing outlet locations.  Required.")
    ("door_file,d", po::value<string>(&door_file), "xml file containing door locations.  Required.");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    

  if (vm.count("help") || !vm.count("door_file") || !vm.count("resolution") || !vm.count("static_map_file") || !vm.count("width") || !vm.count("outlet_file")) {
    std::cout << desc;
    return 1;
  }
  
  tmap::DoorFrameVector doors = tmap::loadDoorsFromFile(door_file, resolution);
  tmap::OccupancyGrid grid = tmap::loadOccupancyGrid(static_map_file);
  tmap::OutletVector outlets = tmap::loadOutletsFromFile(outlet_file);
  grid = tmap::inflateObstacles(grid, inflation_radius);
  tmap::TopologicalMapPtr tmap = groundTruthTopologicalMap(grid, doors, outlets, resolution, width);

  if (vm.count("ppm_output_file")) {
    ofstream stream(ppm_file.c_str());
    tmap->writePpm(stream);
  }
  if (vm.count("topological_map_output_file")) {
    ofstream stream(top_map_file.c_str());
    tmap->writeToStream(stream);
  }
}








