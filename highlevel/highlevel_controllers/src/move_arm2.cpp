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

/* This is a new version of move_arm, which will eventually replace
 * move_arm. - BPG */

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b move_arm is a highlevel controller for moving an arm to a goal joint configuration.
 *
 * This node uses the kinematic path planning service provided in the ROS ompl library where a
 * range of motion planners are available. The control is accomplished by incremental dispatch
 * of joint positions reflecting waypoints in the computed path, until all are accomplished. The current
 * implementation is limited to operatng on left or right arms of a pr2 and is dependent on
 * the kinematic model of that robot.
 *
 * <hr>
 *
 *  @section usage Usage
 *  @verbatim
 *  $ move_arm left|right
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b "mechanism_state"/robot_msgs::MechanismState : The state of the robot joints and actuators
 * - @b "right_arm_goal"/pr2_msgs::MoveArmGoal : The new goal containing a setpoint to achieve for the joint angles
 * - @b "left_arm_goal"/pr2_msgs::MoveArmGoal : The new goal containing a setpoint to achieve for the joint angles
 *
 * Publishes to (name / type):
 * - @b "left_arm_state"/pr2_msgs::MoveArmState : The published state of the controller
 * - @b "left_arm_commands"/pr2_controllers::JointPosCmd : A commanded joint position for the right arm
 *
 *  <hr>
 *
 * @section parameters ROS parameters
 *
 * - None
 **/

#include <highlevel_controllers/highlevel_controller.hh>

// Our interface to the executive
#include <pr2_msgs/MoveArmState.h>
#include <pr2_msgs/MoveArmGoal.h>

#include <robot_srvs/KinematicReplanState.h>
#include <robot_srvs/KinematicReplanLinkPosition.h>
#include <robot_msgs/DisplayKinematicPath.h>
#include <robot_srvs/ValidateKinematicPath.h>
#include <robot_msgs/KinematicPlanStatus.h>

#include <std_msgs/Empty.h>
#include <pr2_mechanism_controllers/JointTraj.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

#include <sstream>
#include <cassert>

#include "tf/transform_listener.h"

class MoveArm : public HighlevelController<pr2_msgs::MoveArmState, 
                                           pr2_msgs::MoveArmGoal>
{

public:

  /**
   * @brief Constructor
   */
  MoveArm(const std::string& node_name, 
          const std::string& state_topic, 
          const std::string& goal_topic,
          const std::string& kinematic_model);

  virtual ~MoveArm() {};

private:
  const std::string kinematic_model_;
  bool have_new_traj_;
  bool replanning_;
  int plan_id_;
  robot_srvs::KinematicReplanState::request active_request_state_;
  robot_srvs::KinematicReplanLinkPosition::request active_request_link_position_;
  robot_msgs::KinematicPlanStatus kps_msg_;

  // HighlevelController interface that we must implement
  virtual void updateGoalMsg();
  virtual bool makePlan();
  virtual bool goalReached();
  virtual bool dispatchCommands();

  // Internal helpers
  void requestStopReplanning(void);
  void sendArmCommand(robot_msgs::KinematicPath &path, 
                      const std::string &model);
  void getTrajectoryMsg(robot_msgs::KinematicPath &path, 
                        pr2_mechanism_controllers::JointTraj &traj);
  void kpsCallback();
};

MoveArm::MoveArm(const std::string& node_name, 
                 const std::string& state_topic, 
                 const std::string& goal_topic,
                 const std::string& kinematic_model)
  : HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal>(node_name, state_topic, goal_topic),
    kinematic_model_(kinematic_model),
    have_new_traj_(false),
    replanning_(false),
    plan_id_(-1)
{
  // TODO: subscribe to relevant topics
  ros::Node::subscribe("kinematic_planning_status",
                       kps_msg_,
                       &MoveArm::kpsCallback,
                       1);
}

void MoveArm::updateGoalMsg()
{
  lock();
  stateMsg.implicit_goal = goalMsg.implicit_goal;
  stateMsg.goal_state = goalMsg.goal_state;
  stateMsg.goal_constraints = goalMsg.goal_constraints;
  unlock();
}

bool MoveArm::makePlan()
{
  // Not clear whether we need to call lock() here...

  if(!goalMsg.implicit_goal)
  {
    robot_srvs::KinematicReplanState::request  req;
    robot_srvs::KinematicReplanState::response  res;

    req.value.params.model_id = kinematic_model_;
    req.value.params.distance_metric = "L2Square";
    req.value.params.planner_id = "IKSBL";
    req.value.threshold = 0.01;
    req.value.interpolate = 1;
    req.value.times = 1;

    // req.start_state is left empty, because we only support replanning, in
    // which case the planner monitors the robot's current state.

    req.value.goal_state = goalMsg.goal_state;

    req.value.allowed_time = 0.5;

    //req.params.volume* are left empty, because we're not planning for the
    //base

    if(replanning_)
      requestStopReplanning();
    replanning_ = true;
    active_request_state_ = req;
    if(!ros::service::call("replan_kinematic_path_state", req, res))
    {
      ROS_WARN("Service call on replan_kinematic_path_state failed");
      return false;
    }
    else
    {
      plan_id_ = res.id;
      ROS_INFO("Issued a replanning request");	    
      return true;
    }
  }
  else
  {
    robot_srvs::KinematicReplanLinkPosition::request req;
    robot_srvs::KinematicReplanLinkPosition::response res;

    req.value.params.model_id = kinematic_model_;
    req.value.params.distance_metric = "L2Square";
    req.value.params.planner_id = "IKSBL";
    req.value.interpolate = 1;
    req.value.times = 1;

    // req.start_state is left empty, because we only support replanning, in
    // which case the planner monitors the robot's current state.

    req.value.goal_constraints = goalMsg.goal_constraints;

    req.value.allowed_time = 0.3;

    //req.params.volume* are left empty, because we're not planning for the
    //base

    if(replanning_)
      requestStopReplanning();
    replanning_ = true;
    active_request_link_position_ = req;
    if(!ros::service::call("replan_kinematic_path_position", req, res))
    {
      ROS_WARN("Service call on replan_kinematic_path_position failed");
      return false;
    }
    else
    {
      plan_id_ = res.id;
      ROS_INFO("Issued a replanning request");	    
      return true;
    }
  }
}

bool MoveArm::goalReached()
{
  kps_msg_.lock();
  bool ret = kps_msg_.done;
  kps_msg_.unlock();
  return ret;
}

bool MoveArm::dispatchCommands()
{
  // To prevent a new one from being written in while we're looking at it
  kps_msg_.lock();
  if(have_new_traj_)
  {
    sendArmCommand(kps_msg_.path, kinematic_model_);
    have_new_traj_ = false;
  }
  kps_msg_.unlock();
  return true;
}

void MoveArm::requestStopReplanning(void)
{
  std_msgs::Empty dummy;
  ros::Node::publish("replan_stop", dummy);
  replanning_ = false;
  ROS_INFO("Issued a request to stop replanning");	
}

void MoveArm::getTrajectoryMsg(robot_msgs::KinematicPath &path, 
                               pr2_mechanism_controllers::JointTraj &traj)
{
  traj.set_points_size(path.get_states_size());

  for (unsigned int i = 0 ; i < path.get_states_size() ; ++i)
  {
    traj.points[i].set_positions_size(path.states[i].get_vals_size());
    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
      traj.points[i].positions[j] = path.states[i].vals[j];
    traj.points[i].time = 0.0;
  }	
}

void MoveArm::sendArmCommand(robot_msgs::KinematicPath &path, 
                             const std::string &model)
{
  pr2_mechanism_controllers::JointTraj traj;
  getTrajectoryMsg(path, traj);
  ros::Node::publish("right_arm_trajectory_command", traj);
  ROS_INFO("Sent trajectory to controller");
}

void MoveArm::kpsCallback()
{
  if(kps_msg_.id >= 0 && (kps_msg_.id == plan_id_))
  {
    if(!kps_msg_.valid)
    {
      //stopArm
    }
    if(!kps_msg_.path.states.empty())
      have_new_traj_ = true;
  }
}

class MoveRightArm: public MoveArm 
{
public:
  MoveRightArm(): MoveArm("rightArmController", 
                          "right_arm_state", 
                          "right_arm_goal", 
                          "pr2::right_arm")
  {
  };

protected:
};

class MoveLeftArm: public MoveArm 
{
public:
  MoveLeftArm(): MoveArm("leftArmController", 
                         "left_arm_state", 
                         "left_arm_goal", 
                         "pr2::left_arm")
  {
  }
};

int
main(int argc, char** argv)
{

  if(argc != 2){
    std::cout << "Usage: ./move_arm left|right";
    return -1;
  }

  ros::init(argc,argv);

  // Extract parameters
  const std::string param = argv[1];

  if(param == "left"){
    MoveLeftArm node;
    node.run();
    ros::fini();
  }
  else if(param == "right"){
    MoveRightArm node;
    node.run();
    ros::fini();
  }
  else {
    std::cout << "Usage: ./move_arm left|right";
    return -1;
  }

  return(0);
}
