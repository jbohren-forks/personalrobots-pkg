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



/** 
 * \file
 * 
 * \author Bhaskara Marthi
 */


#include <topological_map/topological_map.h>
#include <topological_map/exception.h>
#include <topological_map/GetTopologicalMap.h>
#include <string>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <iostream>
#include <fstream>
#include <sysexits.h>
#include <ros/ros.h>

namespace topological_map
{

bool mapService (const TopologicalMap& m, GetTopologicalMap::Request& req, GetTopologicalMap::Response& resp)
{
  resp.regions.push_back(MapRegion());
  return true;
}

typedef boost::function<bool(GetTopologicalMap::Request&, GetTopologicalMap::Response&)> TopMapCallback;


} // End namespace







namespace tmap=topological_map;
namespace po=boost::program_options;
using std::string;
using std::cout;
using std::ifstream;
using boost::bind;
using boost::ref;



int main (int argc, char** argv)
{

  string top_map_file("");
  
  // Begin command line argument processing

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("topological_map,t", po::value<string>(&top_map_file), "Topological map file");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("topological_map")) {
    cout << desc;
    return 1;
  }

  ifstream top_map_stream(top_map_file.c_str());
  tmap::TopologicalMap m(top_map_stream, 1.0, 1e9, 1e9);

  // End command line processing
  // Create and startup node
  
  ros::init(argc, argv, "topological_map");
  ros::NodeHandle n;

  ros::ServiceServer top_map_service = n.advertiseService("topological_map", tmap::TopMapCallback(bind(tmap::mapService, ref(m), _1, _2)));
  ros::spin();

  return 0;
}
