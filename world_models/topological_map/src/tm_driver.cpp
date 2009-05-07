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
#include <ros/time.h>
#include <ros/node.h>
#include <ros/assert.h>

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

  ros::init(argc, argv); 
  ros::Node node("tm_driver");
  
  OccupancyGrid grid(extents[21][24]);
  setV(grid, 0, 1, 21, 0, 1, 24, false);
  setV(grid, 7, 7, 21, 0, 1, 24, true);
  setV(grid, 0, 1, 21, 8, 8, 24, true);
  setV(grid, 3, 7, 21, 8, 8, 24, false);
  setV(grid, 7, 7, 21, 4, 8, 24, false);
  
  for (uint r=0; r<21; r++) {
    for (uint c=0; c<24; c++) {
      if (grid[r][c]) cout << "X"; else cout << ".";
    }
    cout << endl;
  }
  

  TopologicalMapPtr m = topologicalMapFromGrid (grid, 1.0, 2, 1, 1, 0, "local");

  cout << *m;

  ofstream str("local/test");
  m->writeToStream(str);
  ofstream str2("local/out.ppm");
  m->writePpm(str2);
  m->connectorCosts(Point2D(1,1), Point2D(10,10));

  m->addOutlet(OutletInfo(1,2,3,4,5,6,7,8, "green"));
  m->addOutlet(OutletInfo(4,7,9,3,3.5,12,7.253,1, "blue"));
  //str = ofstream("local/gui-input.xml");
  // m->writeOutletsAndMap(str);


  ifstream str3("/u/bhaskara/local/top/willow.tmap");
  double dx=-1;
  tf::Transform trans(tf::Quaternion::getIdentity(), tf::Vector3(dx,0,0));
  TopologicalMap m3(str3, 1.0, 1e9, 1e9, trans);

  Point2D p1(1-dx,1), p2(30-dx,30);
  cout << "Nearest outlet to " << p1 << " is " << m3.nearestOutlet(p1) << endl;
  cout << "Nearest outlet to " << p2 << " is " << m3.nearestOutlet(p2) << endl;

  cout << "Outlet 1 blocked is " << m3.outletInfo(1).blocked << endl;
  cout << "Outlet 0 blocked is " << m3.outletInfo(0).blocked << endl;
  m3.observeOutletBlocked(1);
  cout << "Outlet 1 blocked is " << m3.outletInfo(1).blocked << endl;
  cout << "Outlet 0 blocked is " << m3.outletInfo(0).blocked << endl;

  
  
  p1 = Point2D(25-dx,21);
  p2 = Point2D(15-dx,25);
  printConnectorCosts(m3,p1,p2, 0);

  RegionId door1 = m3.containingRegion(Point2D(21.5-dx,20.5));
  RegionId door2 = m3.containingRegion(Point2D(50-dx,30));
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


  tmap::Visualizer v(m3);
  Duration dur(1);
  while (true) {
    dur.sleep();
    v.visualize();
  }

//   std::ifstream str3("local/willow.tmap");
//   tmap::TopologicalMap m2(str3);
//   std::ofstream str4("local/willow2.tmap");
//   m2.writeToStream(str4);
//   std::ofstream str5("local/willow2.ppm");
//   m2.writePpm(str5);
}

  
  
  
  
  
