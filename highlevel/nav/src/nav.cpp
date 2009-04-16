/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <ros/node.h>
#include <navfn/navfn.h>
#include <navfn/NavfnPlan.h>
#include <navfn/MapCell.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <robot_actions/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <robot_msgs/Polyline2D.h>

class PlannerTest {
  public:
    PlannerTest(ros::Node& ros_node, tf::TransformListener& tf) : ros_node_(ros_node), tf_(tf),
    map_updater_(NULL) {
      ros_node_.advertise<robot_msgs::Polyline2D>("gui_path", 1);
      ros_node_.subscribe("goal", goal_msg_, &PlannerTest::goalCallback, this, 1);

      //set the parameters we want to use for the costmap
      ros_node_.setParam("~costmap/observation_topics", "base_scan");
      ros_node_.setParam("~costmap/base_scan/observation_persistance", 0.0);
      ros_node_.setParam("~costmap/base_scan/expected_update_rate", 0.2);
      ros_node_.setParam("~costmap/base_scan/data_type", "LaserScan");

      map_updater_ = new costmap_2d::Costmap2DROS(ros_node_, tf_);
    }

    ~PlannerTest(){
      if(map_updater_ != NULL)
        delete map_updater_;
    }

    void goalCallback(){
      //update the working copy of the costmap we'll use for planning
      map_updater_->getCostMapCopy(working_copy_);

      navfn::NavfnPlan::Request plan_req;
      navfn::NavfnPlan::Response plan_res;

      //update the costmap that the planner will use
      const unsigned char* planner_map = working_copy_.getCharMap();


      unsigned int map_size = working_copy_.cellSizeX() * working_copy_.cellSizeY();
      plan_req.map_size_x = working_copy_.cellSizeX();
      plan_req.map_size_y = working_copy_.cellSizeY();
      plan_req.set_cost_map_size(map_size);

      //copy the map over
      for(unsigned int i = 0; i < map_size; ++i)
        plan_req.cost_map[i] = planner_map[i];

      //get the current pose of the robot in map space
      tf::Stamped<tf::Pose> robot_pose, global_pose;
      global_pose.setIdentity();
      robot_pose.setIdentity();
      robot_pose.frame_id_ = "base_link";
      robot_pose.stamp_ = ros::Time();
      try{
        tf_.transformPose("map", robot_pose, global_pose);
      }
      catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
      }
      catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
      }
      catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      }

      double wx = global_pose.getOrigin().x();
      double wy = global_pose.getOrigin().y();

      navfn::MapCell start;
      navfn::MapCell goal;

      unsigned int mx, my;
      working_copy_.worldToMap(wx, wy, mx, my);
      start.x = mx;
      start.y = my;

      working_copy_.worldToMap(goal_msg_.x, goal_msg_.y, mx, my);
      goal.x = mx;
      goal.y = my;

      plan_req.start = start;
      plan_req.goal = goal;

      //for timing info
      struct timeval time_start, time_end;
      double start_t, end_t, t_diff;
      gettimeofday(&time_start, NULL);
      bool success = ros::service::call("navfn_server/planner", plan_req, plan_res);
      gettimeofday(&time_end, NULL);
      start_t = time_start.tv_sec + double(time_start.tv_usec) / 1e6;
      end_t = time_end.tv_sec + double(time_end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_INFO("Planning time: %.9f", t_diff);

      // If good, extract plan and update
      if(success){
        // Extract the plan in world co-ordinates
        robot_msgs::Polyline2D gui_path_msg;
        gui_path_msg.header.frame_id = "map";
        gui_path_msg.set_points_size(plan_res.plan.size());
        for(unsigned int i=0; i < plan_res.plan.size(); i++){
          double wx, wy;
          unsigned int mx = plan_res.plan[i].x;
          unsigned int my = plan_res.plan[i].y;
          working_copy_.mapToWorld(mx, my, wx, wy);
          gui_path_msg.points[i].x = wx;
          gui_path_msg.points[i].y = wy;
        }

        gui_path_msg.color.r = 0;
        gui_path_msg.color.g = 1.0;
        gui_path_msg.color.b = 0;
        gui_path_msg.color.a = 0;

        ros_node_.publish("gui_path", gui_path_msg);

      }
      

    }

  private:
    ros::Node& ros_node_;
    tf::TransformListener& tf_;
    robot_actions::Pose2D goal_msg_;
    costmap_2d::Costmap2DROS* map_updater_;
    costmap_2d::Costmap2D working_copy_;
};

int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node ros_node("planner_test");
  tf::TransformListener tf(ros_node, true, ros::Duration(10));
  PlannerTest test(ros_node, tf);
  ros_node.spin();
  return(0);

}

