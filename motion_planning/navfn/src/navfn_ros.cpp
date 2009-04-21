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
#include <navfn/navfn_ros.h>

namespace navfn {
  NavfnROS::NavfnROS(ros::Node& ros_node, tf::TransformListener& tf, const costmap_2d::Costmap2D& cost_map) : ros_node_(ros_node), tf_(tf), 
  cost_map_(cost_map), planner_(cost_map.cellSizeX(), cost_map.cellSizeY()) {
    //read parameters for the planner
    ros_node_.param("~/navfn/global_frame", global_frame_, std::string("map"));
    ros_node_.param("~/navfn/robot_base_frame", robot_base_frame_, std::string("base_link"));
  }

  double NavfnROS::getPointPotential(const robot_msgs::Point& world_point){
    unsigned int mx, my;
    if(!cost_map_.worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_.nx + mx;
    return planner_.potarr[index];
  }

  bool NavfnROS::computePotential(const robot_msgs::Point& world_point){
    planner_.setCostMap(cost_map_.getCharMap());

    unsigned int mx, my;
    if(!cost_map_.worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_.setStart(map_start);
    planner_.setGoal(map_goal);

    return planner_.calcNavFnDijkstra();
  }

  bool NavfnROS::makePlan(const robot_msgs::PoseStamped& goal, std::vector<robot_msgs::PoseStamped>& plan){
    //get the pose of the robot in the global frame
    tf::Stamped<tf::Pose> robot_pose, global_pose, orig_goal, goal_pose;

    //convert the goal message into tf::Stamped<tf::Pose>
    tf::PoseStampedMsgToTF(goal, orig_goal);

    global_pose.setIdentity();
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time::now();
    try{
      //transform both the goal and pose of the robot to the global frame
      tf_.transformPose(global_frame_, robot_pose, global_pose);
      tf_.transformPose(global_frame_, orig_goal, goal_pose);
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

    planner_.setCostMap(cost_map_.getCharMap());

    unsigned int mx, my;
    if(!cost_map_.worldToMap(wx, wy, mx, my))
      return false;

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal_pose.getOrigin().x();
    wy = goal_pose.getOrigin().y();

    if(!cost_map_.worldToMap(wx, wy, mx, my))
      return false;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_.setStart(map_start);
    planner_.setGoal(map_goal);

    bool success = planner_.calcNavFnAstar();

    if(success){
      //extract the plan
      float *x = planner_.getPathX();
      float *y = planner_.getPathY();
      int len = planner_.getPathLen();
      for(int i = 0; i < len; ++i){
        unsigned int cell_x, cell_y;
        cell_x = (unsigned int) x[i];
        cell_y = (unsigned int) y[i];

        //convert the plan to world coordinates
        double world_x, world_y;
        cost_map_.mapToWorld(cell_x, cell_y, world_x, world_y);

        robot_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        plan.push_back(pose);
      }
    }

    return success;
  }
};
