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
#include <mpglue/plan.h>
#include <mpglue/SetIndexTransform.h>
#include <mpglue/SelectPlanner.h>
#include <navfn/SetCostmap.h>
#include <navfn/MakeNavPlan.h>
#include <costmap_2d/cost_values.h>
#include <gtest/gtest.h>

#define PREFIX "/costmap_planner_node/"

using navfn::SetCostmap;
using mpglue::SetIndexTransform;
using mpglue::SelectPlanner;
using navfn::MakeNavPlan;


namespace mpglue_test {

  class CostmapPlannerNodeTest
    : public testing::Test {
  public:
    CostmapPlannerNodeTest()
      : set_costmap_client(nh.serviceClient<SetCostmap>(PREFIX "set_costmap")),
	set_index_transform_client(nh.serviceClient<SetIndexTransform>(PREFIX "set_index_transform")),
	select_planner_client(nh.serviceClient<SelectPlanner>(PREFIX "select_planner")),
	make_nav_plan_client(nh.serviceClient<MakeNavPlan>(PREFIX "make_nav_plan"))
    {}
  
    ros::NodeHandle nh;
  
    ros::ServiceClient set_costmap_client;
    ros::ServiceClient set_index_transform_client;
    ros::ServiceClient select_planner_client;
    ros::ServiceClient make_nav_plan_client;
  
    SetCostmap set_costmap_srv;
    SetIndexTransform set_index_transform_srv;
    SelectPlanner select_planner_srv;
    MakeNavPlan make_nav_plan_srv;
  };

}

using namespace mpglue_test;

TEST_F (CostmapPlannerNodeTest, set_costmap)
{
  set_costmap_srv.request.width = 4;
  set_costmap_srv.request.height = 3;
  uint16_t const tt(set_costmap_srv.request.width * set_costmap_srv.request.height);
  set_costmap_srv.request.costs.reserve(tt);
  for (uint8_t ii(0); ii < tt; ++ii)
    set_costmap_srv.request.costs.push_back(ii);
  ASSERT_TRUE (set_costmap_client.call(set_costmap_srv));
}


TEST_F (CostmapPlannerNodeTest, set_index_transform)
{
  set_index_transform_srv.request.origin_x =    0.3;
  set_index_transform_srv.request.origin_y =  -10.5;
  set_index_transform_srv.request.origin_th =   1.234;
  set_index_transform_srv.request.resolution =  0.05;
  EXPECT_TRUE (set_index_transform_client.call(set_index_transform_srv));
  set_index_transform_srv.request.resolution =  -9876;
  EXPECT_FALSE (set_index_transform_client.call(set_index_transform_srv));
}


TEST_F (CostmapPlannerNodeTest, select_planner)
{
  select_planner_srv.request.planner_spec = "";
  select_planner_srv.request.robot_spec = "";
  EXPECT_TRUE (select_planner_client.call(select_planner_srv));
  EXPECT_EQ (select_planner_srv.response.ok, 1)
    << "error_message " << select_planner_srv.response.error_message;

  select_planner_srv.request.planner_spec = "ad:3dkin";
  select_planner_srv.request.robot_spec = "pr2:100:150:55:34";
  EXPECT_TRUE (select_planner_client.call(select_planner_srv));
  EXPECT_EQ (select_planner_srv.response.ok, 1)
    << "error_message " << select_planner_srv.response.error_message;
  
  select_planner_srv.request.planner_spec = "__NoSuchPlanner";
  select_planner_srv.request.robot_spec = "__NoSuchRobot";
  EXPECT_TRUE (select_planner_client.call(select_planner_srv));
  EXPECT_EQ (select_planner_srv.response.ok, 0);
}


TEST_F (CostmapPlannerNodeTest, make_nav_plan)
{
  set_costmap_srv.request.width = 0;
  set_costmap_srv.request.height = 0;
  set_costmap_srv.request.costs.clear();
  ASSERT_TRUE (set_costmap_client.call(set_costmap_srv)) << "failed to clear costmap";
  
  set_costmap_srv.request.width = 200;
  set_costmap_srv.request.height = 100;
  uint16_t const tt(set_costmap_srv.request.width * set_costmap_srv.request.height);
  for (uint16_t ii(0); ii < tt; ++ii)
    set_costmap_srv.request.costs.push_back(0);
  ASSERT_TRUE (set_costmap_client.call(set_costmap_srv)) << "failed to initialize costmap";
  
  set_index_transform_srv.request.origin_x =    0;
  set_index_transform_srv.request.origin_y =    0;
  set_index_transform_srv.request.origin_th =   0;
  set_index_transform_srv.request.resolution =  0.05;
  ASSERT_TRUE (set_index_transform_client.call(set_index_transform_srv));
  
  select_planner_srv.request.planner_spec = "ad:2d";
  select_planner_srv.request.robot_spec = "";
  ASSERT_TRUE (select_planner_client.call(select_planner_srv))
    << "failed to call select_planner service";
  ASSERT_EQ (select_planner_srv.response.ok, 1)
    << "select planner error_message " << select_planner_srv.response.error_message;
  
  double const xmax(set_costmap_srv.request.width  * set_index_transform_srv.request.resolution);
  double const ymax(set_costmap_srv.request.height * set_index_transform_srv.request.resolution);
  
  mpglue::waypoint_s start(0.1 * xmax, 0.1 * ymax, M_PI / 4);
  mpglue::waypoint_s goal(0.9 * xmax, 0.9 * ymax, - 3 * M_PI / 4);
  start.toPose(make_nav_plan_srv.request.start.pose);
  goal.toPose(make_nav_plan_srv.request.goal.pose);
  ASSERT_TRUE (make_nav_plan_client.call(make_nav_plan_srv))
    << "failed to call make_nav_plan service";
  ASSERT_EQ (make_nav_plan_srv.response.plan_found, 1)
    << "ADStar should have found a path";
  
  start.x = 1.1 * xmax;
  start.toPose(make_nav_plan_srv.request.start.pose);
  ASSERT_TRUE (make_nav_plan_client.call(make_nav_plan_srv))
    << "failed to call make_nav_plan service";
  ASSERT_NE (make_nav_plan_srv.response.plan_found, 1)
    << "ADStar should have failed to find a path (start outside of costmap)";

  start.x = 0.1 * xmax;
  goal.y = 1.1 * ymax;
  start.toPose(make_nav_plan_srv.request.start.pose);
  goal.toPose(make_nav_plan_srv.request.goal.pose);
  ASSERT_TRUE (make_nav_plan_client.call(make_nav_plan_srv))
    << "failed to call make_nav_plan service";
  ASSERT_NE (make_nav_plan_srv.response.plan_found, 1)
    << "ADStar should have failed to find a path (goal outside of costmap)";
  
  for (uint16_t ii(0); ii < tt; ++ii)
    set_costmap_srv.request.costs[ii] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  ASSERT_TRUE (set_costmap_client.call(set_costmap_srv))
    << "failed to fill costmap with obstacles";
  
  goal.y = 0.9 * ymax;
  goal.toPose(make_nav_plan_srv.request.goal.pose);
  ASSERT_TRUE (make_nav_plan_client.call(make_nav_plan_srv))
    << "failed to call make_nav_plan service";
  ASSERT_NE (make_nav_plan_srv.response.plan_found, 1)
    << "ADStar should have failed to find a path (costmap filled with obstacles)";
  
  select_planner_srv.request.planner_spec = "NavFn";
  select_planner_srv.request.robot_spec = "";
  ASSERT_TRUE (select_planner_client.call(select_planner_srv))
    << "failed to call select_planner service";
  ASSERT_EQ (select_planner_srv.response.ok, 1)
    << "select planner error_message " << select_planner_srv.response.error_message;
  
  ASSERT_TRUE (make_nav_plan_client.call(make_nav_plan_srv))
    << "failed to call make_nav_plan service";
  ASSERT_NE (make_nav_plan_srv.response.plan_found, 1)
    << "NavFn should have failed to find a path (costmap filled with obstacles)";
  
  for (uint16_t ii(0); ii < tt; ++ii)
    set_costmap_srv.request.costs[ii] = 2;
  ASSERT_TRUE (set_costmap_client.call(set_costmap_srv))
    << "failed to fill costmap with low-cost values";
  
  ASSERT_TRUE (make_nav_plan_client.call(make_nav_plan_srv))
    << "failed to call make_nav_plan service";
  ASSERT_EQ (make_nav_plan_srv.response.plan_found, 1)
    << "NavFn should have found a path";
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_costmap_planner_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
