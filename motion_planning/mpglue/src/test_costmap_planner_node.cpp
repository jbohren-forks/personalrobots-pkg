/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <mpglue/SetIndexTransform.h>
#include <mpglue/SelectPlanner.h>
#include <navfn/SetCostmap.h>
#include <gtest/gtest.h>

#define PREFIX "/costmap_planner_node/"


TEST (costmap_planner_node, set_costmap)
{
  ros::NodeHandle nh;
  ros::ServiceClient
    client(nh.serviceClient<navfn::SetCostmap>(PREFIX "set_costmap"));
  navfn::SetCostmap srv;
  srv.request.width = 4;
  srv.request.height = 3;
  uint16_t const tt(srv.request.width * srv.request.height);
  srv.request.costs.reserve(tt);
  for (uint8_t ii(0); ii < tt; ++ii)
    srv.request.costs.push_back(ii);
  ASSERT_TRUE (client.call(srv));
}


TEST (costmap_planner_node, set_index_transform)
{
  ros::NodeHandle nh;
  ros::ServiceClient
    client(nh.serviceClient<mpglue::SetIndexTransform>(PREFIX "set_index_transform"));
  mpglue::SetIndexTransform srv;
  srv.request.origin_x =    0.3;
  srv.request.origin_y =  -10.5;
  srv.request.origin_th =   1.234;
  srv.request.resolution =  0.05;
  EXPECT_TRUE (client.call(srv));
  srv.request.resolution =  -9876;
  EXPECT_FALSE (client.call(srv));
}


TEST (costmap_planner_node, select_planner)
{
  ros::NodeHandle nh;
  ros::ServiceClient
    client(nh.serviceClient<mpglue::SelectPlanner>(PREFIX "select_planner"));
  mpglue::SelectPlanner srv;

  srv.request.planner_spec = "";
  srv.request.robot_spec = "";
  EXPECT_TRUE (client.call(srv)) << "error_message " << srv.response.error_message;
  EXPECT_EQ (srv.response.ok, 1);

  srv.request.planner_spec = "ad:3dkin";
  srv.request.robot_spec = "pr2:100:150:55:34";
  EXPECT_TRUE (client.call(srv)) << "error_message " << srv.response.error_message;
  EXPECT_EQ (srv.response.ok, 1);
  
  srv.request.planner_spec = "__NoSuchPlanner";
  srv.request.robot_spec = "__NoSuchRobot";
  EXPECT_TRUE (client.call(srv));
  EXPECT_EQ (srv.response.ok, 0);
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_costmap_planner_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
