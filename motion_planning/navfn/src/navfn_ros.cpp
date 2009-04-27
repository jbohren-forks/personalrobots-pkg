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
  NavfnROS::NavfnROS(ros::Node& ros_node, tf::TransformListener& tf, costmap_2d::Costmap2D& cost_map) : ros_node_(ros_node), tf_(tf), 
  cost_map_(cost_map), planner_(cost_map.cellSizeX(), cost_map.cellSizeY()) {
    //advertise our plan visualization
    ros_node_.advertise<robot_msgs::Polyline>("~navfn/plan", 1);

    //read parameters for the planner
    ros_node_.param("~/navfn/costmap/global_frame", global_frame_, std::string("map"));
    ros_node_.param("~/navfn/costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("~/navfn/transform_tolerance", transform_tolerance_, 0.2);
    
    //we need to make sure that the transform between the robot base frame and the global frame is available
    while(!tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(5.0))){
      ROS_WARN("Waiting on transform from %s to %s to become available before running the planner", robot_base_frame_.c_str(), global_frame_.c_str());
    }

    //we'll get the parameters for the robot radius from the costmap we're associated with
    ros_node_.param("~navfn/costmap/inscribed_radius", inscribed_radius_, 0.325);
    ros_node_.param("~navfn/costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    ros_node_.param("~navfn/costmap/inflation_radius", inflation_radius_, 0.55);
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

  void NavfnROS::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my){
    double useless_pitch, useless_roll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);

    //set the associated costs in the cost map to be free
    cost_map_.setCost(mx, my, costmap_2d::FREE_SPACE);

    double max_inflation_dist = inflation_radius_ + inscribed_radius_;

    //make sure to re-inflate obstacles in the affected region
    cost_map_.reinflateWindow(global_pose.getOrigin().x(), global_pose.getOrigin().y(), max_inflation_dist, max_inflation_dist);

  }

  bool NavfnROS::makePlan(const robot_msgs::PoseStamped& goal, std::vector<robot_msgs::PoseStamped>& plan){
    //get the pose of the robot in the global frame
    tf::Stamped<tf::Pose> robot_pose, global_pose, orig_goal, goal_pose;

    //convert the goal message into tf::Stamped<tf::Pose>
    tf::PoseStampedMsgToTF(goal, orig_goal);

    global_pose.setIdentity();
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later
    robot_pose.stamp_ = ros::Time();
    try{
      //transform both the goal and pose of the robot to the global frame
      tf_.transformPose(global_frame_, robot_pose, global_pose);
      tf_.transformPose(global_frame_, orig_goal, goal_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return false;
    }
    // check global pose timeout
    if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
      ROS_ERROR("Transform timeout. global pose stamp: %f current time: %f",global_pose.stamp_.toSec(),current_time.toSec());
      return false;
    }

    double wx = global_pose.getOrigin().x();
    double wy = global_pose.getOrigin().y();

    unsigned int mx, my;
    if(!cost_map_.worldToMap(wx, wy, mx, my))
      return false;

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(global_pose, mx, my);

    planner_.setCostMap(cost_map_.getCharMap());

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

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return success;
  }

  void NavfnROS::publishPlan(const std::vector<robot_msgs::PoseStamped>& path, double r, double g, double b, double a){
    //given an empty path we won't do anything
    if(path.empty())
      return;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    robot_msgs::Polyline gui_path_msg;
    gui_path_msg.header.frame_id = path[0].header.frame_id;
    gui_path_msg.header.stamp = path[0].header.stamp;
    gui_path_msg.set_points_size(path.size());
    for(unsigned int i=0; i < path.size(); i++){
      gui_path_msg.points[i].x = path[i].pose.position.x;
      gui_path_msg.points[i].y = path[i].pose.position.y;
      gui_path_msg.points[i].z = path[i].pose.position.z;
    }

    gui_path_msg.color.r = r;
    gui_path_msg.color.g = g;
    gui_path_msg.color.b = b;
    gui_path_msg.color.a = a;

    ros_node_.publish("~navfn/plan", gui_path_msg);
  }
};
