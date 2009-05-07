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

#include <iostream> 

/** ROS **/
#include <ros/node.h>
#include <boost/thread.hpp>

/** TF **/
#include <tf/tf.h>

/** Messages needed for trajectory control and collision map**/
#include <robot_msgs/Pose.h>
#include <robot_msgs/JointTraj.h>
#include <robot_msgs/JointTrajPoint.h>
#include <robot_msgs/CollisionMap.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>

// MPGlue and sbpl headers
#include <mpglue/sbpl_planner.h>
#include <mpglue/sbpl_environment.h>
#include <mpglue/plan.h>
#include <sfl/util/strutil.hpp>
#include <sbpl/headers.h>

#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <pr2_robot_actions/Pose2D.h>
#include <robot_msgs/Door.h>
#include <door_handle_detector/door_functions.h>

typedef struct
{
    robot_msgs::Door door;
    double door_thickness;
    double arm_min_workspace_angle;
    double arm_max_workspace_angle;
    double arm_min_workspace_radius;
    double arm_max_workspace_radius;
    double door_angle_discretization_interval;
    robot_msgs::Point32 shoulder;
}DoorEnvProperties;

class SBPLDoorPlanner : public robot_actions::Action<robot_msgs::Door, robot_msgs::Door>
{
  public:
  SBPLDoorPlanner(ros::Node& ros_node, tf::TransformListener& tf);

  virtual ~SBPLDoorPlanner();
      
  /**
   * @brief  Runs whenever a new goal is sent to the move_base
   * @param goal The goal to pursue 
   * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
   * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
   */
  virtual robot_actions::ResultStatus execute(const robot_msgs::Door& goal, robot_msgs::Door& feedback);

  private:

  ros::Node &ros_node_;
  tf::TransformListener &tf_;
  
  boost::shared_ptr<mpglue::CostmapAccessor> cm_access_;
  boost::shared_ptr<mpglue::IndexTransform> cm_index_;
  boost::shared_ptr<mpglue::SBPLEnvironment> env_;
  boost::shared_ptr<mpglue::SBPLPlannerWrap> pWrap_;

  double planner_time_limit_; /* The amount of time given to the planner to find a plan */
  double allocated_time_;

  bool use_cost_map_;
  bool forward_search_;

  std::string planner_type_;
  std::string plan_stats_file_;
  
  costmap_2d::Costmap2DROS *cost_map_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_;        /**< local copy of the costmap underlying cost_map_ros_ */
  
  friend struct cm_getter;
  struct cm_getter: public mpglue::costmap_2d_getter {
      cm_getter(SBPLDoorPlanner * spn): spn_(spn) {}
      virtual costmap_2d::Costmap2D const * operator () () { return &spn_->cost_map_; }
      SBPLDoorPlanner * spn_;
  };
  cm_getter cm_getter_;	                   /**< for mpglue to get at our costmap instance */
  
  bool initializePlannerAndEnvironment(const robot_msgs::Door &door);
  
  bool makePlan(const pr2_robot_actions::Pose2D &start, const pr2_robot_actions::Pose2D &goal, robot_msgs::JointTraj &path);

  std::vector<robot_msgs::Point> footprint_;

  boost::recursive_mutex lock_; /*!< Lock for access to class members in callbacks */

  DoorEnvProperties door_env_;

  std::string global_frame_, robot_base_frame_;

  bool updateGlobalPose();

  pr2_robot_actions::Pose2D getPose2D(const tf::Stamped<tf::Pose> &pose);

  pr2_robot_actions::Pose2D global_pose_2D_;

  pr2_robot_actions::Pose2D goal_;

  tf::Stamped<tf::Pose> global_pose_;

  bool removeDoor();

  void publishPath(const robot_msgs::JointTraj &path, std::string topic, double r, double g, double b, double a);

  bool computeOrientedFootprint(const pr2_robot_actions::Pose2D &position, const std::vector<robot_msgs::Point>& footprint_spec, std::vector<robot_msgs::Point>& oriented_footprint);

  bool clearRobotFootprint(costmap_2d::Costmap2D& cost_map);

  double inflation_radius_;

  double inscribed_radius_;

  void publishFootprint(const pr2_robot_actions::Pose2D &position);

  void publishDoor(const robot_msgs::Door &door);

  costmap_2d::Costmap2DPublisher* costmap_publisher_;

  bool animate_;
};
