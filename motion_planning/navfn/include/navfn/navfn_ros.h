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
#ifndef NAVFN_NAVFN_ROS_H_
#define NAVFN_NAVFN_ROS_H_

#include <ros/node.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <robot_msgs/PoseStamped.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/Polyline2D.h>
#include <tf/transform_listener.h>
#include <vector>
#include <robot_msgs/Point.h>

namespace navfn {
  class NavfnROS {
    public:
      /**
       * @brief  Constructor for the NavFnROS object
       * @param  ros_node The a reference to the ros node running
       * @param  tf A reference to a TransformListener
       * @param  cos_map A reference to the costmap to use
       */
      NavfnROS(ros::Node& ros_node, tf::TransformListener& tf, costmap_2d::Costmap2D& cost_map);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const robot_msgs::PoseStamped& goal, std::vector<robot_msgs::PoseStamped>& plan);

      /**
       * @brief  Compute the full navigation function for the costmap given a point in the world to start from
       * @param world_point The point to use for seeding the navigation function 
       * @return True if the navigation function was computed successfully, false otherwise
       */
      bool computePotential(const robot_msgs::Point& world_point);

      /**
       * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @return The navigation function's value at that point in the world
       */
      double getPointPotential(const robot_msgs::Point& world_point);

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<robot_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~NavfnROS(){}

    private:
      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
      ros::Node& ros_node_;
      tf::TransformListener& tf_;
      costmap_2d::Costmap2D& cost_map_;
      NavFn planner_;
      std::string global_frame_, robot_base_frame_;
      double transform_tolerance_; // timeout before transform errors
      double inscribed_radius_, circumscribed_radius_, inflation_radius_;
  };
};

#endif
