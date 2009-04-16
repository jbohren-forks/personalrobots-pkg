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
  NavfnROS::NavfnROS(ros::Node& ros_node, tf::TransformListener& tf, const costmap_2d::Costmap2D& cost_map) : ros_node_(ros_node), tf_(tf), cost_map_(cost_map) {
    //read parameters for the planner
    ros_node_.param("~/navfn/global_frame", global_frame_, std::string("map"));
    ros_node_.param("~/navfn/robot_base_frame", robot_base_frame_, std::string("base_link"));
  }

  bool NavfnROS::makePlan(const robot_actions::Pose2D& goal, std::vector<robot_actions::Pose2D>& plan){
    unsigned int size_x = cost_map_.cellSizeX();
    unsigned int size_y = cost_map_.cellSizeY();

    //create a new planner of the appropriate size
    NavFn planner(size_x, size_y);
    
    //get the pose of the robot in the global frame
    tf::Stamped<tf::Pose> robot_pose, global_pose;
    global_pose.setIdentity();
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();
    try{
      tf_.transformPose(global_frame_, robot_pose, global_pose);
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

    planner.setCostMap(cost_map_.getCharMap());

    unsigned int mx, my;
    if(!cost_map_.worldToMap(wx, wy, mx, my))
      return false;

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    if(!cost_map_.worldToMap(goal.x, goal.y, mx, my))
      return false;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner.setStart(map_start);
    planner.setGoal(map_goal);

    bool success = planner.calcNavFnAstar();

    if(success){
      //extract the plan
      float *x = planner.getPathX();
      float *y = planner.getPathY();
      int len = planner.getPathLen();
      for(int i = 0; i < len; ++i){
        unsigned int cell_x, cell_y;
        cell_x = (unsigned int) x[i];
        cell_y = (unsigned int) y[i];

        //convert the plan to world coordinates
        double world_x, world_y;
        cost_map_.mapToWorld(cell_x, cell_y, world_x, world_y);

        robot_actions::Pose2D pose;
        pose.x = world_x;
        pose.y = world_y;
        plan.push_back(pose);
      }
    }

    return success;
  }
};
