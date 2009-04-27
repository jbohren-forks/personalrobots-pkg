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
* Author: Sachin Chitta
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <doors_core/door_reactive_planner.h>

#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <pr2_robot_actions/Pose2D.h>
#include <robot_msgs/JointTraj.h>

#include <ros/node.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/DiagnosticMessage.h>

#include <angles/angles.h>
#include <vector>
#include <string>



namespace nav {
  /**
   * @class MoveBaseDoorAction
   * @brief A class adhering to the robot_actions::Action interface that moves the robot base to a goal location.
   */
  class MoveBaseDoorAction : public robot_actions::Action<robot_msgs::Door, robot_msgs::Door> {
    public:
      /**
       * @brief  Constructor for the actions
       * @param ros_node A reference to the ros node used 
       * @param tf A reference to a TransformListener
       * @return 
       */
      MoveBaseDoorAction(ros::Node& ros_node, tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBaseDoorAction();

      /**
       * @brief  Runs whenever a new goal is sent to the move_base
       * @param goal The goal to pursue 
       * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
       * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
       */
      virtual robot_actions::ResultStatus execute(const robot_msgs::Door& goal, robot_msgs::Door& feedback);

    private:
      /**
       * @brief  Sleeps for the remainder of a cycle
       * @param  start The start time of the cycle
       * @param  cycle_time The desired cycle time
       * @return True if the desired cycle time is met, false otherwise
       */
      bool sleepLeftover(ros::Time start, ros::Duration cycle_time);

      /**
       * @brief  Publishes the footprint of the robot for visualization purposes
       */
      void publishFootprint();

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPath(const std::vector<pr2_robot_actions::Pose2D>& path, std::string topic, double r, double g, double b, double a);

      /**
       * @brief  Make a new global plan (the goal is specified using the setDoor() function while start is specified using the 
       * current position of the robot internally
       */
      void makePlan();

      /**
       * @brief  Trim off parts of the global plan that are far enough behind the robot
       */
      void prunePlan();

      /**
       * @brief  Check if the goal orientation has been achieved
       * @return True if achieved, false otherwise
       */
      bool goalOrientationReached();

      /**
       * @brief  Check if the goal position has been achieved
       * @return True if achieved, false otherwise
       */
      bool goalPositionReached();

      /**
       * @brief Check if the robot is inside the doorway
       */
      bool goalReached();

      /**
       * @brief  Compute the distance between two points
       * @param x1 The first x point 
       * @param y1 The first y point 
       * @param x2 The second x point 
       * @param y2 The second y point 
       * @return 
       */
      double distance(double x1, double y1, double x2, double y2);

      /**
       * @brief  Get the current pose of the robot in the global frame and set the global_pose_ variable
       */
      void updateGlobalPose();

      /**
       * @brief  Clear the footprint of the robot in a given cost map
       * @param cost_map The costmap to apply the clearing opertaion on
       */
      void clearRobotFootprint(costmap_2d::Costmap2D& cost_map);

      /**
       * @brief  Resets the costmaps to the static map outside a given window
       */
      void resetCostMaps();

      ros::Node& ros_node_;
      tf::TransformListener& tf_;
      bool run_planner_;
      costmap_2d::Costmap2DROS* planner_cost_map_ros_;
      costmap_2d::Costmap2D planner_cost_map_;

      door_reactive_planner::DoorReactivePlanner* planner_;
      std::vector<pr2_robot_actions::Pose2D> global_plan_;
      std::vector<robot_msgs::Point> footprint_;
      std::string global_frame_, control_frame_, robot_base_frame_;
      bool valid_plan_;
      boost::recursive_mutex lock_;
      robot_msgs::Door door_;
      pr2_robot_actions::Pose2D goal_;

      tf::Stamped<tf::Pose> global_pose_;
      double xy_goal_tolerance_, yaw_goal_tolerance_, min_abs_theta_vel_;
      double inscribed_radius_, circumscribed_radius_, inflation_radius_;
      double controller_frequency_;

      std::vector<pr2_robot_actions::Pose2D> empty_plan_;

      std::string control_topic_name_;

    /**
     * @brief Transform a tf::Stamped<tf::Pose> to pr2_robot_actions::Pose2D
     * @param pose The pose object to be transformed
     * @return Transformed object of type pr2_robot_actions::Pose2D
     */
    pr2_robot_actions::Pose2D getPose2D(const tf::Stamped<tf::Pose> &pose);

    /**
     * @brief dispatch control commands
     * @param plan_in The plan to be dispatched
     */
    void dispatchControl(const std::vector<pr2_robot_actions::Pose2D> &plan_in);

    int plan_size_;

    std::string plan_state_;

    void publishDiagnostics(bool force);

    double current_distance_to_goal_;

    double diagnostics_expected_publish_time_;

    ros::Time last_diagnostics_publish_time_;

  };
};
#endif

