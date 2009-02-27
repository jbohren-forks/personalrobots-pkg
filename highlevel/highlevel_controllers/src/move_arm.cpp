/*********************************************************************
*  Software License Agreement (BSD License)
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

// service for (re)planning to a state
#include <robot_srvs/KinematicReplanState.h>
// service for (re)planning to a link position
#include <robot_srvs/KinematicReplanLinkPosition.h>
// service for getting IK solution for an EE pose
#include <robot_srvs/IKService.h>

#include <robot_msgs/DisplayKinematicPath.h>
#include <robot_msgs/KinematicPlanStatus.h>

#include <urdf/URDF.h>

#include <std_msgs/Empty.h>
#include <robot_msgs/JointTraj.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>

#include <robot_msgs/MechanismState.h>

#include <planning_models/kinematic.h>

#include <sstream>
#include <cassert>

#include <tf/transform_listener.h>

class MoveArm : public HighlevelController<pr2_msgs::MoveArmState, 
                                           pr2_msgs::MoveArmGoal>
{

public:

  /**
   * @brief Constructor
   */
  MoveArm(const std::string& side);

  virtual ~MoveArm();

private:
  robot_msgs::MechanismState       mechanism_state_msg_;
  robot_msgs::KinematicPlanStatus  plan_status_;
  const std::string                kinematic_model_;
  const std::string                controller_topic_;
  const std::string                controller_name_;
  planning_models::KinematicModel* robot_model_;

  int                              plan_id_;
  int                              traj_id_;
  bool                             new_goal_;
  bool                             new_trajectory_;
  bool                             plan_valid_;
  robot_msgs::KinematicPath        current_trajectory_;

  // HighlevelController interface that we must implement
  virtual void updateGoalMsg();
  virtual bool makePlan();
  virtual bool goalReached();
  virtual bool dispatchCommands();

  // Internal helpers
  void requestStopReplanning(void);
  void sendArmCommand(robot_msgs::KinematicPath &path, 
                      const std::string &model);
  void stopArm(void);
  void getTrajectoryMsg(robot_msgs::KinematicPath &path, 
                        robot_msgs::JointTraj &traj);
  void printKinematicPath(robot_msgs::KinematicPath &path);
  void mechanismStateCallback();
  void receiveStatus();
  void getCurrentStateAsTraj(robot_msgs::JointTraj& traj);
};

MoveArm::~MoveArm() {
  if (robot_model_) {
    delete robot_model_;
  }
}

MoveArm::MoveArm(const std::string& side)
  : HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal>(side + "_arm_controller", side + "_arm_state", side + "_arm_goal"),
    kinematic_model_("pr2::" + side + "_arm"),
    controller_topic_(side + "_arm_trajectory_command"),
    controller_name_(side + "_arm_trajectory_controller"),
    robot_model_(NULL), plan_id_(-1), traj_id_(-1), 
    new_goal_(false), new_trajectory_(false), 
    plan_valid_(false)
{
  // Adjust logging level
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  // Set the logger for this package to output all statements
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros::console::notifyLoggerLevelsChanged();

  //Create robot model.
  std::string model;
  if (ros::Node::instance()->getParam("robot_description", model))
  {
    robot_desc::URDF file;
    file.loadString(model.c_str());
    robot_model_ = new planning_models::KinematicModel();
    robot_model_->setVerbose(false);
    robot_model_->build(model);
    // make sure we are in the robot's self frame
    robot_model_->reduceToRobotFrame();
    
    // Say that we're up and ready
    initialize();
  }
  else
  {
    ROS_FATAL("Robot model not found! Did you remap robot_description?");
    ROS_BREAK();
  }


  ros::Node::instance()->subscribe("kinematic_planning_status", 
                                   plan_status_, 
                                   &MoveArm::receiveStatus, 
                                   this,
                                   1);
  ros::Node::instance()->subscribe("mechanism_state",
                                   mechanism_state_msg_,
                                   &MoveArm::mechanismStateCallback,
                                   this,
                                   1);
  ros::Node::instance()->advertise<robot_msgs::JointTraj>(controller_topic_, 1);
}

void MoveArm::receiveStatus(void)
{
  ROS_DEBUG("Got status: %d %d %d", 
            plan_id_, plan_status_.id, plan_status_.path.states.empty());
  // HACK: sometimes we get the status for the new plan before we receive
  // the response to the service call that gives us the new plan id.  It
  // happens that the planner increments the id by 1 with each request.
  if(((plan_id_ == plan_status_.id) || 
      (plan_id_ == plan_status_.id + 1)) && 
     !plan_status_.path.states.empty())
  {
    current_trajectory_ = plan_status_.path;
    ROS_DEBUG("Got new trajectory from planner");
    new_trajectory_ = true;
  }
}

void MoveArm::updateGoalMsg()
{
  lock();
  stateMsg.goal = goalMsg.goal_configuration;
  ROS_DEBUG("Received new goal");
  new_goal_ = true;
  unlock();
}

void MoveArm::mechanismStateCallback() 
{
  stateMsg.lock();
  unsigned int len = robot_model_->getGroupDimension(robot_model_->getGroupID(kinematic_model_));
  stateMsg.set_configuration_size(len);

  std::vector<std::string> joints;
  robot_model_->getJointsInGroup(joints, robot_model_->getGroupID(kinematic_model_));

  unsigned int stateInd = 0;
  for (unsigned int i = 0; i < joints.size(); i++)
  {
    ROS_ASSERT(stateInd < len);
    if (robot_model_->getJoint(joints[i])->usedParams == 1) {
      unsigned int ind = 0;
      while (mechanism_state_msg_.joint_states[ind].name != joints[i] && ind < mechanism_state_msg_.get_joint_states_size())
      	ind++;
      
      if (ind == mechanism_state_msg_.get_joint_states_size())
      {
	ROS_ERROR("No joint '%s' in mechanism data.", joints[i].c_str());
	stateMsg.configuration[stateInd].name = joints[i];
	stateMsg.configuration[stateInd].position = 0;
      }
      else
      {
	stateMsg.configuration[stateInd].name = joints[i];
	stateMsg.configuration[stateInd].position = mechanism_state_msg_.joint_states[ind].position;
      }
      stateInd++;
    } else if (robot_model_->getJoint(joints[i])->usedParams != 0) {
      ROS_ERROR("Joints with more than one param are not supported in movearm.");
    }
  }
  stateMsg.unlock();
}

void MoveArm::getCurrentStateAsTraj(robot_msgs::JointTraj& traj)
{
  mechanism_state_msg_.lock();

  std::vector<std::string> joints;
  robot_model_->getJointsInGroup(joints, robot_model_->getGroupID(kinematic_model_));

  traj.set_points_size(1);
  traj.points[0].set_positions_size(joints.size());

  for (unsigned int i = 0; i < joints.size(); i++)
  {
    unsigned int j;
    for (j = 0; j < mechanism_state_msg_.joint_states.size(); j++)
    {
      if(mechanism_state_msg_.joint_states[j].name == joints[i])
        break;
    }
    ROS_ASSERT(j<mechanism_state_msg_.joint_states.size());

    traj.points[0].positions[i] = mechanism_state_msg_.joint_states[j].position;
  }

  mechanism_state_msg_.unlock();
}

void stateParamsToMsg(planning_models::KinematicModel::StateParams *state, 
		      planning_models::KinematicModel* model, 
		      std::string group, std::vector<double>& conf) {
  conf.clear();
  //Copy the stateparams in to the req.
  state->copyParamsGroup(conf, group);
}


bool MoveArm::makePlan()
{
  if(!new_goal_)
    return true;

  new_goal_ = false;

  ROS_DEBUG("Starting to plan...");
  
  goalMsg.lock();

  robot_srvs::KinematicReplanState::Request  req;
  robot_srvs::KinematicReplanState::Response  res;

  req.value.params.model_id = kinematic_model_;
  req.value.params.distance_metric = "L2Square";
  //req.value.params.planner_id = "SBL";
  req.value.params.planner_id = "KPIECE";
  req.value.threshold = 0.1;
  req.value.interpolate = 1;
  req.value.times = 1;

  //Copies in the state.
  //First create stateparams for the group of intrest.
  planning_models::KinematicModel::StateParams *state = robot_model_->newStateParams();
  ROS_ASSERT(state);

  // Skip setting the start state, so the planner will use the mech state.

  stateParamsToMsg(state, robot_model_, kinematic_model_, req.value.goal_state.vals);

  //Set the goal state.
  std::vector<robot_msgs::JointState> goal_configuration;
  // If it's an implicit goal, first call out to IK to get the
  // configuration
  if(goalMsg.implicit_goal)
  {
    robot_srvs::IKService::Request  ikreq;
    robot_srvs::IKService::Response ikres;

    ikreq.header = goalMsg.header;
    // Timeout is how long we're willing to wait for the transform from
    // our frame to the ik_node's hardcoded working frame.  0.0 is forever
    // (dangerous!).
    ikreq.timeout.fromSec(0.1);

    // Where does the 7 come from?
    ikreq.joint_pos.set_positions_size(7);

    // Replace with latest mechanism_state data
    ikreq.joint_pos.positions[0] = 0.0;
    ikreq.joint_pos.positions[1] = 0.0;
    ikreq.joint_pos.positions[2] = 0.0;
    ikreq.joint_pos.positions[3] = 0.0;
    ikreq.joint_pos.positions[4] = 0.0;
    ikreq.joint_pos.positions[5] = 0.0;
    ikreq.joint_pos.positions[6] = 0.0;

    if(ros::service::call("perform_pr2_ik_closest", ikreq, ikres))
    {
      for(int j=0; j < (int) ikres.traj.points[0].positions.size(); j++)
      {
        // HACK: we're assuming that the ordering of the angles returned by
        // the IKService is the same as that expected by the planner
        goal_configuration[j].position = ikres.traj.points[0].positions[j];
      }
    }
    else
    {
      ROS_WARN("service call failed");
    }  
  }
  else
  {
    goal_configuration = goalMsg.goal_configuration;
  }

  for (unsigned int i = 0; i < goal_configuration.size(); i++) 
  {
    planning_models::KinematicModel::Joint* j = robot_model_->getJoint(goal_configuration[i].name);
    ROS_ASSERT(j);
    unsigned int axes = j->usedParams;
    ROS_ASSERT(axes == 1);
    double* param = new double[axes];
    param[0] = goalMsg.goal_configuration[i].position;
    state->setParamsJoint(param, goalMsg.goal_configuration[i].name);
    delete[] param;
  }

  //ROS_DEBUG("Going for:");
  //state->print();


  //FIXME: should be something?
  req.value.allowed_time = 0.5;

  //req.params.volume* are left empty, because we're not planning for the base.

  bool ret = ros::service::call("replan_kinematic_path_state", req, res);
  if (!ret)
  {
    ROS_ERROR("Service call on plan_kinematic_path_state failed");
    plan_id_ = -1;
  }
  else
  {
    plan_id_ = res.id;
    ROS_DEBUG("Requested plan, got id %d\n", plan_id_);
    plan_status_.lock();
    plan_status_.unlock();
  }

  goalMsg.unlock();

  return true;
}

bool MoveArm::goalReached()
{
  return getDone();
}

bool MoveArm::dispatchCommands()
{
  // Force replanning if we have a new goal
  if(new_goal_)
    return false;

  plan_status_.lock();

  // HACK: sometimes we get the status for the new plan before we receive
  // the response to the service call that gives us the new plan id.  It
  // happens that the planner increments the id by 1 with each request.
  if(((plan_status_.id != plan_id_) &&
      (plan_status_.id != plan_id_ + 1)) ||
     (traj_id_ < 0 && !new_trajectory_))
  {
    // NOOP
    ROS_DEBUG("Idle");
  }
  else if(plan_status_.done || (traj_id_ >= 0 && !new_trajectory_))
  {
    bool done = false;
    if(!plan_status_.done)
    {
      ROS_DEBUG("Checking for trajectory completion");
      pr2_mechanism_controllers::TrajectoryQuery::Request req;
      pr2_mechanism_controllers::TrajectoryQuery::Response res;
      req.trajectoryid = traj_id_;
      if(!ros::service::call(controller_name_ + "/TrajectoryQuery", req, res))
        ROS_ERROR("Failed to query trajectory completion");
      else if(res.done == res.State_Done)
        done = true;
    }
    else
      done = true;

    if(done)
    {
      ROS_DEBUG("Plan completed.");
      plan_id_ = -1;
      traj_id_ = -1;
      ROS_DEBUG("Execution is complete");
      stateMsg.status = pr2_msgs::MoveArmState::INACTIVE;
      plan_valid_ = true;
    }
  }
  else if(plan_status_.valid && 
          !plan_status_.unsafe &&
          !current_trajectory_.states.empty())
  {
    ROS_DEBUG("sending new trajectory");
    sendArmCommand(current_trajectory_, kinematic_model_);
    plan_valid_ = true;
    new_trajectory_ = false;
  }
  else
  {
    ROS_DEBUG("Plan invalid %d %d %d %d %d %d\n",
             plan_id_ >= 0,
             plan_status_.id == plan_id_,
             plan_status_.valid,
             !plan_status_.unsafe,
             !current_trajectory_.states.empty(),
             new_trajectory_ == true);
    stopArm();
    plan_valid_ = false;
  }
  plan_status_.unlock();

  return plan_valid_;
}

void MoveArm::printKinematicPath(robot_msgs::KinematicPath &path)
{
  unsigned int nstates = path.get_states_size();    
  ROS_DEBUG("Obtained solution path with %u states", nstates);    
  std::stringstream ss;
  ss << std::endl;
  for (unsigned int i = 0 ; i < nstates ; ++i)
  {
    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
      ss <<  path.states[i].vals[j] << " ";	
    ss << std::endl;	
  }    
  ROS_DEBUG(ss.str().c_str());
}

void MoveArm::getTrajectoryMsg(robot_msgs::KinematicPath &path, 
                               robot_msgs::JointTraj &traj)
{
  traj.set_points_size(path.get_states_size());
  ROS_DEBUG("ARM:");
  for (unsigned int i = 0 ; i < path.get_states_size() ; ++i)
  {
    traj.points[i].set_positions_size(path.states[i].get_vals_size());
    ROS_DEBUG("STEP:");
    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
    {
      traj.points[i].positions[j] = path.states[i].vals[j];
      ROS_DEBUG("%f ", path.states[i].vals[j]);
    }
    traj.points[i].time = 0.0;
  }	
}

void MoveArm::sendArmCommand(robot_msgs::KinematicPath &path, 
                             const std::string &model)
{
  stopArm();
  ROS_DEBUG("Sending %d-step trajectory\n", path.states.size());
  pr2_mechanism_controllers::TrajectoryStart::Request req;
  pr2_mechanism_controllers::TrajectoryStart::Response res;
  getTrajectoryMsg(path, req.traj);
  req.hastiming = 0;
  req.requesttiming = 0;
  //ros::Node::instance()->publish(controller_topic_, traj);
  int new_traj_id_ = -1;
  if(!ros::service::call(controller_name_ + "/TrajectoryStart", req, res))
    ROS_ERROR("Failed to start trajectory");
  else
    new_traj_id_ = res.trajectoryid;

  traj_id_ = new_traj_id_;
  ROS_DEBUG("Got new traj_id: %d\n", traj_id_);
  new_trajectory_ = false;
}

void MoveArm::stopArm()
{
  ROS_DEBUG("Stopping arm.");
  /*
  if(!trajectory_stopped_)
  {
    // Send an empty trajectory
    robot_msgs::JointTraj traj;
    ros::Node::instance()->publish(controller_topic_, traj);
    trajectory_stopped_ = true;
  }
  */

  // Cancel any pre-existing trajectory
  if(traj_id_ >= 0)
  {
    ROS_DEBUG("Cancelling trajectory %d\n", traj_id_);
    pr2_mechanism_controllers::TrajectoryCancel::Request cancel;
    pr2_mechanism_controllers::TrajectoryCancel::Response dummy;
    cancel.trajectoryid = traj_id_;
    if(!ros::service::call(controller_name_ + "/TrajectoryCancel", cancel, dummy))
    {
      ROS_ERROR("Failed to cancel trajectory %d\n", traj_id_);
    }
    traj_id_ = -1;
  }
}

class MoveRightArm: public MoveArm 
{
public:
  MoveRightArm(): MoveArm("right")
  {
  };

protected:
};

class MoveLeftArm: public MoveArm 
{
public:
  MoveLeftArm(): MoveArm("left")
  {
  }
};

void usage(void)
{
    std::cout << "Usage: ./move_arm left|right [standard ROS arguments]" << std::endl;
}

int
main(int argc, char** argv)
{

  if(argc < 2){
    usage();
    return -1;
  }

  ros::init(argc,argv);

  // Extract parameters
  const std::string param = argv[1];

  if(param == "left"){
    ros::Node rosnode("left_arm_controller");
    MoveLeftArm node;
    node.run();
    
  }
  else if(param == "right"){
    ros::Node rosnode("right_arm_controller");
    MoveRightArm node;
    node.run();
    
  }
  else {
    usage();      
    return -1;
  }

  return(0);
}
