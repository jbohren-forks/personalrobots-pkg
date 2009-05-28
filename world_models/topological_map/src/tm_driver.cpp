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


#include <iostream>
#include <fstream>
#include <vector>
#include <getopt.h>
#include <sysexits.h>
#include <topological_map/topological_map.h>
#include <topological_map/visualization.h>
#include <topological_map/exception.h>
#include <ros/time.h>
#include <ros/ros.h>
#include <ros/assert.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

typedef unsigned int uint;
typedef const uint cuint;

using std::vector;

using ros::Duration;
using std::cout;
using std::endl;
using std::pair;
using std::ifstream;
using std::ofstream;
using boost::extents;
namespace tmap=topological_map;
using tmap::OccupancyGrid;
using tmap::TopologicalMapPtr;
using tmap::topologicalMapFromGrid;
using tmap::Point2D;
using tmap::Cell2D;
using tmap::RegionId;
using tmap::ConnectorId;
using ros::Time;
using tmap::TopologicalMap;
using tmap::OutletInfo;

void setV (tmap::OccupancyGrid& grid, cuint r0, cuint dr, cuint rmax, cuint c0, cuint dc, cuint cmax, bool val) 
{
  for (uint r=r0; r<rmax; r+=dr) 
    for (uint c=c0; c<cmax; c+=dc) 
      grid[r][c] = val;
}


void printConnectorCosts (TopologicalMap& m, const Point2D& p1, const Point2D& p2, double t)
{
  typedef vector<pair<ConnectorId, double> > CCosts;

  CCosts cc = m.connectorCosts(p1,p2, Time(t));

  for (CCosts::iterator iter=cc.begin(); iter!=cc.end(); ++iter) {
    ConnectorId i = iter->first;
    cout << "Connector " << i << " at " << m.connectorPosition(i) << " has cost " << iter->second << endl;
  }

}

int main (int argc, char* argv[])
{

  
  OccupancyGrid grid(extents[21][24]);
  setV(grid, 0, 1, 21, 0, 1, 24, false);
  setV(grid, 7, 7, 21, 0, 1, 24, true);
  setV(grid, 0, 1, 21, 8, 8, 24, true);
  setV(grid, 3, 7, 21, 8, 8, 24, false);
  setV(grid, 7, 7, 21, 4, 8, 24, false);
  
//   for (uint r=0; r<21; r++) {
//     for (uint c=0; c<24; c++) {
//       if (grid[r][c]) cout << "X"; else cout << ".";
//     }
//     cout << endl;
//   }
  

  TopologicalMapPtr m = topologicalMapFromGrid (grid, 1.0, 2, 1, 1, 0, "local");
  m->writeGridAndOutletData("local/gui_input.xml");


  // cout << *m;

  ofstream str("local/test");
  m->writeToStream(str);
  ofstream str2("local/out.ppm");
  m->writePpm(str2);
  m->connectorCosts(Point2D(1,1), Point2D(10,10));


  m->addOutlet(OutletInfo(1,2,3,4,5,6,7,8, "green"));
  m->addOutlet(OutletInfo(4,7,9,3,3.5,12,7.253,1, "blue"));
  m->readOutletApproachOverrides("/u/bhaskara/ros/ros-pkg/world_models/willow_maps/outlet_overrides.xml");
  m->readDoorApproachOverrides("/u/bhaskara/ros/ros-pkg/world_models/willow_maps/willow.tmap.door_overrides.xml");
  //str = ofstream("local/gui-input.xml");
  // m->writeOutletsAndMap(str);

  cout << "Approach position of outlet 1 is " << m->outletApproachPosition(1u, 1.0, 1.0) << endl;
  cout << "Approach position of outlet 2 is " << m->outletApproachPosition(2u, 1.0, 1.0) << endl;
  cout << "Approach position of connector 100 is " << m->doorApproachPosition(100, 1.0) << endl;


  
  ifstream str3("/u/bhaskara/ros/ros-pkg/world_models/willow_maps/willow.tmap");
  double dx=-.4;
  TopologicalMap m3(str3, 1.0, 1e9, 1e9);

  Point2D p1(19.1125, 29.1625);
  Point2D p2(12.7, 22.5);
  tmap::ConnectorIdVector path = m3.shortestConnectorPath(p1, p2);
  cout << " Path between " << p1 << " and " << p2 << ": ";
  foreach (ConnectorId connector, path) {
    cout << endl << " " << connector << ": ";
    try {
      cout << m3.connectorPosition(connector);
    }
    catch (tmap::UnknownConnectorIdException& e) {
      cout << "unknown";
    }
  }
  cout << endl;




  p1 = Point2D(51.53,22.3875);
  p2 = Point2D(55.93, 9.68);
  
  Point2D p3 = Point2D(12.7, 22.5);

  m3.distanceBetween(p1, p3);


  m3.distanceBetween(p1, p3);


  path = m3.shortestConnectorPath(p1, p2);
  cout << " Path between " << p1 << " and " << p2 << ": ";
  foreach (ConnectorId connector, path) {
    cout << endl << " " << connector << ": ";
    try {
      cout << m3.connectorPosition(connector);
    }
    catch (tmap::UnknownConnectorIdException& e) {
      cout << "unknown";
    }
  }
  cout << endl;

  printConnectorCosts (m3, Point2D(33.844, 36.379), Point2D(12.7, 22.5), 0);

  cout << "Approach position of outlet 1 at radius 2, .4 is " << m3.outletApproachPosition(1,2,.4) << endl;
  cout << "Approach position of outlet 1 at radius 3, .3 is " << m3.outletApproachPosition(1,3,.3) << endl;
  cout << "Approach position of outlet 1 at radius 3, .2 is " << m3.outletApproachPosition(1,3,.2) << endl;

  

  ros::init(argc, argv, "tm_driver"); 
  tmap::Visualizer v(m3);



  Duration dur(1);
  ros::NodeHandle n;
  while (n.ok()) {
    dur.sleep();
    // v.visualize();
  }


  p1 = Point2D(1-dx,1);
  p2 = Point2D(30-dx,30);
  cout << "Nearest outlet to " << p1 << " is " << m3.nearestOutlet(p1) << endl;
  cout << "Nearest outlet to " << p2 << " is " << m3.nearestOutlet(p2) << endl;

  cout << "Outlet 2 blocked is " << m3.outletInfo(2).blocked << endl;
  cout << "Outlet 1 blocked is " << m3.outletInfo(1).blocked << endl;
  m3.observeOutletBlocked(2);
  cout << "Outlet 2 blocked is " << m3.outletInfo(2).blocked << endl;
  cout << "Outlet 1 blocked is " << m3.outletInfo(1).blocked << endl;

  
  
  p1 = Point2D(25-dx,21);
  p2 = Point2D(15-dx,25);
  printConnectorCosts(m3,p1,p2, 0);

  RegionId door1 = m3.containingRegion(Point2D(21.5-dx,20.5));
  //RegionId door2 = m3.containingRegion(Point2D(50-dx,30));
  door_msgs::Door d = m3.regionDoor(m3.containingRegion(Point2D(50-dx,17.25)));
  cout << "Door at " << d.frame_p1.x << ", " << d.frame_p1.y << " and " << d.frame_p2.x << ", " << d.frame_p2.y << endl;
  cout << "Open prob at 0.0 is " << m3.doorOpenProb(door1, Time(0.0)) << endl;

  cout << "Open prob at 1.0 is " << m3.doorOpenProb(door1, Time(1.0)) << endl;
  m3.observeDoorTraversal(door1, false, Time(60.0));
  printConnectorCosts(m3,p1,p2, 60);
  cout << "Open prob at 60.0 is " << m3.doorOpenProb(door1, Time(60.0)) << " and open is " << m3.isDoorOpen(door1, Time(60.0)) << endl;
  cout << "Open prob at 1000.0 is " << m3.doorOpenProb(door1, Time(1000.0)) << " and open is " << m3.isDoorOpen(door1, Time(1000.0)) << endl;
  cout << "Open prob at 50000.0 is " << m3.doorOpenProb(door1, Time(50000.0)) << " and open is " << m3.isDoorOpen(door1, Time(50000.0)) << endl;
  cout << "Open prob at 1000000.0 is " << m3.doorOpenProb(door1, Time(1000000.0)) << " and open is " << m3.isDoorOpen(door1, Time(1000000.0)) << endl;
  cout << "Open prob at 10000000.0 is " << m3.doorOpenProb(door1, Time(10000000.0)) << " and open is " << m3.isDoorOpen(door1, Time(10000000.0)) << endl;



//   std::ifstream str3("local/willow.tmap");
//   tmap::TopologicalMap m2(str3);
//   std::ofstream str4("local/willow2.tmap");
//   m2.writeToStream(str4);
//   std::ofstream str5("local/willow2.ppm");
//   m2.writePpm(str5);
}

  
  
  
  
  
