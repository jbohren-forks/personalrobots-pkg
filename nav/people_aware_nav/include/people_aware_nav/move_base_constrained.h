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
 *********************************************************************/
#ifndef PEOPLE_AWARE_NAV_MOVE_BASE_CONSTRAINED_H_
#define PEOPLE_AWARE_NAV_MOVE_BASE_CONSTRAINED_H_
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <people_aware_nav/ConstrainedMoveBaseState.h>
#include <robot_msgs/PoseStamped.h>
#include <people_aware_nav/ConstrainedGoal.h>
#include <ros/node.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <vector>
#include <string>

namespace people_aware_nav {

using robot_msgs::PoseStamped;

/**
 * @class MoveBaseConstrained
 * @brief A class adhering to the robot_actions::Action interface that moves the robot base to a goal location.
 */
class MoveBaseConstrained : public robot_actions::Action<ConstrainedGoal, PoseStamped> {
public:
  /**
   * @brief  Constructor for the actions
   * @param ros_node A reference to the ros node used 
   * @param tf A reference to a TransformListener
   * @return 
   */
  MoveBaseConstrained(ros::Node& ros_node, tf::TransformListener& tf);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~MoveBaseConstrained();

  /**
   * @brief  Runs whenever a new goal is sent to the move_base
   * @param goal The goal to pursue 
   * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
   * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
   */
  virtual robot_actions::ResultStatus execute(const ConstrainedGoal& goal, PoseStamped& feedback);

private:
  /**
   * @brief  Sleeps for the remainder of a cycle
   * @param  start The start time of the cycle
   * @param  cycle_time The desired cycle time
   * @return True if the desired cycle time is met, false otherwise
   */
  bool sleepLeftover(ros::Time start, ros::Duration cycle_time, ros::Duration& actual);


  /**
   * @brief  Make a new global plan
   * @param  goal The goal to plan to
   */
  void makePlan(const ConstrainedGoal& goal);


  /**
   * @brief  Get the current pose of the robot in the specified frame
   * @param  frame The frame to get the pose in
   * @param  pose The pose returned
   */
  void getRobotPose(std::string frame, tf::Stamped<tf::Pose>& pose);


  /**
   * @brief  Trim off parts of the global plan that are far enough behind the robot
   */
  void prunePlan();

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
  void resetCostmaps();

  ros::Node& ros_node_;
  tf::TransformListener& tf_;
  bool run_planner_;
  base_local_planner::TrajectoryPlannerROS* tc_;
  costmap_2d::Costmap2DROS* planner_cost_map_ros_, *controller_cost_map_ros_;
  costmap_2d::Costmap2D planner_cost_map_, controller_cost_map_;

  navfn::NavfnROS* planner_;
  std::vector<robot_msgs::PoseStamped> global_plan_;
  std::vector<robot_msgs::Point> footprint_;
  std::string global_frame_, robot_base_frame_;
  bool valid_plan_, new_plan_;
  boost::recursive_mutex lock_;
  ConstrainedGoal goal_;

  tf::Stamped<tf::Pose> global_pose_;
  double inscribed_radius_, circumscribed_radius_, inflation_radius_;
  double controller_frequency_;

};

};
#endif

