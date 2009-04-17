/*********************************************************************
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
 *   * Neither the name of Willow Garage nor the names of its
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

#pragma once

#include <ros/node.h>

//messages
#include <robot_actions/Pose2D.h>
#include <robot_msgs/Vector3.h>
#include <robot_msgs/Point.h>

// For transform support
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>

// door helper functions
#include <door_handle_detector/door_functions.h>

// costmap
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>

//angles
#include <angles/angles.h>

namespace door_reactive_planner
{

 class DoorReactivePlanner
 {
   public:

    DoorReactivePlanner(ros::Node &ros_node, tf::TransformListener& tf, costmap_2d::Costmap2D* cost_map, std::string path_frame_id, std::string costmap_frame_id);

   bool makePlan(const robot_actions::Pose2D &start, std::vector<robot_actions::Pose2D> &best_path);

   ros::Node &node_;
   
   tf::TransformListener &tf_;

   private:

   costmap_2d::Costmap2D *cost_map_;

   base_local_planner::CostmapModel *cost_map_model_;
   
   std::string path_frame_id_;

   std::string costmap_frame_id_;

   bool choose_straight_line_trajectory_;

   double inscribed_radius_;

   double circumscribed_radius_;

   double min_distance_from_obstacles_;

   double dist_waypoints_max_;

   double dist_rot_waypoints_max_;

   double max_explore_distance_;

   double horizontal_explore_distance_;

   double max_explore_delta_angle_;

   double door_goal_distance_;

   int num_explore_paths_;

   double max_inflated_cost_;

   int cell_distance_from_obstacles_;

   robot_msgs::Vector3 vector_along_door_;

   double centerline_angle_;

   robot_actions::Pose2D goal_;

   bool door_information_set_ ;

   std::vector<robot_msgs::Point> footprint_;

   void getParams();

   void setDoor(robot_msgs::Door door_msg_in);

   bool computeOrientedFootprint(const robot_actions::Pose2D &position, const std::vector<robot_msgs::Point>& footprint_spec, std::vector<robot_msgs::Point>& oriented_footprint);

   double distance(const robot_actions::Pose2D &p, const robot_actions::Pose2D &q);

   bool createLinearPath(const robot_actions::Pose2D &cp,const robot_actions::Pose2D &fp, std::vector<robot_actions::Pose2D> &return_path);

   void getFinalPosition(const robot_actions::Pose2D &current_position, const double &delta_angle, const double &distance_to_centerline, robot_actions::Pose2D &end_position);

   bool getPointCost(const robot_msgs::Point &position, double &cost);

   void checkPath(const std::vector<robot_actions::Pose2D> &path, const std::string &path_frame_id, std::vector<robot_actions::Pose2D> &return_path, std::string &costmap_frame_id);

   void transformPath(const std::vector<robot_actions::Pose2D> &path_in, const std::string &frame_in, std::vector<robot_actions::Pose2D> &path_out, const std::string &frame_out);

   void transform2DPose(const robot_actions::Pose2D &path_in, const std::string original_frame_id, robot_actions::Pose2D &path_out, const std::string &transform_frame_id);

   friend class costmap_2d::Costmap2D;
 };
}

