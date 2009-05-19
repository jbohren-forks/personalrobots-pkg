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

#include "topological_map/topological_map.h"
#include "topological_map/exception.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/foreach.hpp>
#include <robot_actions/action_client.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <boost/assign/std/vector.hpp>
#include <gtest/gtest.h>


#define foreach BOOST_FOREACH

using namespace std;
using namespace boost::assign;

namespace ra=robot_actions;

using std::cout;
using std::endl;
using std::set;
using std::ifstream;
using ros::NodeHandle;
using boost::extents;
using boost::shared_ptr;
using boost::tie;
using std::stringstream;
using topological_map::TopologicalMap;
using topological_map::ConnectorId;
using topological_map::OutletId;
using robot_msgs::PoseStamped;
using nav_robot_actions::MoveBaseState;
using topological_map::Point2D;
using ros::Time;
using ros::Duration;


// Tests

typedef vector<pair<ConnectorId, double> > CCosts;



TEST(TopologicalMap, Simulator)
{
  ifstream str("/u/bhaskara/ros/ros-pkg/world_models/willow_maps/willow.tmap");
  TopologicalMap m(str, 1.0, 1e9, 1e9);

  NodeHandle n;
  ros::Duration timeout(600.0);

  ra::ActionClient<PoseStamped, MoveBaseState, PoseStamped> navigator("move_base");

  vector<OutletId> outlets;
  outlets += 1, 4, 2, 30, 10;

  foreach (OutletId outlet, outlets) {
    Point2D p = m.outletApproachPosition(outlet, 2.0, 0.5);
    cout << "Going to " << p << endl;
    PoseStamped goal, feedback;
    goal.pose.position.x = p.x;
    goal.pose.position.y = p.y;
    goal.pose.position.z = 0;
    goal.pose.orientation.x = 1;
    goal.header.frame_id = "map";
    ra::ResultStatus result = navigator.execute(goal, feedback, timeout);
    EXPECT_EQ (ra::SUCCESS, result);
  }
}

TEST(TopologicalMap, ConnectorCosts)
{
  ifstream str("/u/bhaskara/ros/ros-pkg/world_models/willow_maps/willow.tmap");

  TopologicalMap m(str, 1.0, 1e9, 1e9);

  CCosts cc = m.connectorCosts(Point2D(33.844, 36.379), Point2D(12.7, 22.5), Time(0));

  EXPECT_TRUE (cc.size()<100);
  EXPECT_TRUE (cc.size()>0);
  EXPECT_TRUE (cc[0].second>0.0);
  EXPECT_TRUE (cc[0].second<1000);
}



int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "topological_map_tests"); 
  return RUN_ALL_TESTS();
}
