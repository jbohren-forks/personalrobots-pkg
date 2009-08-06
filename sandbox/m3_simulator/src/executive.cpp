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
 * Executive that moves to a bunch of waypoints
 *
 */

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <pr2_robot_actions/Pose2D.h>

#define foreach BOOST_FOREACH

namespace po=boost::program_options;
using namespace std;
using pr2_robot_actions::Pose2D;

int main (int argc, char** argv)
{
  vector<Pose2D> waypoints;


  // Process command-line arguments
  {
    vector<string> waypoint_strings;
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help,h", "produce help message")
      ("waypoint,w", po::value<vector<string> >(&waypoint_strings), "waypoints");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || !vm.count("waypoint")) {
      cout << desc << "\n";
      return 1;
    }

    else {
      typedef boost::char_separator<char> Separator;
      typedef boost::tokenizer<Separator> Tokenizer;
      Separator sep(" ");

      foreach (string str, waypoint_strings) {
        Tokenizer tok(str, sep);
        Tokenizer::iterator iter=tok.begin();
        Pose2D waypoint;
        waypoint.x = atof((*iter++).c_str());
        waypoint.y = atof((*iter++).c_str());
        waypoint.th = atof((*iter++).c_str());
        waypoints.push_back(waypoint);
      }
    }
  }



  foreach (Pose2D waypoint, waypoints)
    cout << "Waypoint " << waypoint.x << " " << waypoint.y << " " << waypoint.th << endl;
      
  // Start executive
  //ros::init (argc, argv, "m3_waypoint_executive");
  //WaypointExecutive exec;
  //exec.run();

  return 0;
} 
