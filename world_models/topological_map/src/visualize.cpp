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
#include <string>
#include <boost/program_options.hpp>
#include <sysexits.h>
#include <topological_map/topological_map.h>
#include <topological_map/visualization.h>
#include <ros/time.h>
#include <ros/ros.h>
#include <ros/assert.h>

typedef unsigned int uint;
typedef const uint cuint;

using std::vector;
using std::string;
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



namespace po=boost::program_options;

int main (int argc, char* argv[])
{

  string top_map_file("");

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("topological_map,t", po::value<string>(&top_map_file), "Topological map file");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    

  if (vm.count("help") || !vm.count("topological_map")) {
    std::cout << desc;
    return 1;
  }

  ifstream str3(top_map_file.c_str());
  TopologicalMap m3(str3, 1.0, 1e9, 1e9);

  ros::init(argc, argv, "topological_map_visualizer"); 
  tmap::Visualizer v(m3);

  Duration dur(1);
  ros::NodeHandle n;
  while (n.ok()) {
    dur.sleep();
    v.visualize();
  }
}

  
  
  
  
  
