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
 * - @b "mechanism_state"/mechanism_control::MechanismState : The state of the robot joints and actuators
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

#include <HighlevelController.hh>
#include <pr2_mechanism_controllers/JointPosCmd.h>
#include <mechanism_control/MechanismState.h>
#include <pr2_msgs/MoveArmState.h>
#include <pr2_msgs/MoveArmGoal.h>
#include <robot_msgs/DisplayKinematicPath.h>
#include <robot_srvs/NamedKinematicPlanState.h>
#include <robot_srvs/PlanNames.h>

static const unsigned int RIGHT_ARM_JOINTS_BASE_INDEX = 11;
static const unsigned int LEFT_ARM_JOINTS_BASE_INDEX = 12;

class MoveArm : public HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal> {

public:

  /**
   * @brief Constructor
   */
  MoveArm(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
	  const std::string& _armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel);

  virtual ~MoveArm();

private:

  /**
   * @brief Helper Method to obtain the joint value by name
   * @return true if the joint is present, otherwise false
   */
  bool readJointValue(const mechanism_control::MechanismState& mechanismStateMsg, const std::string& name, double& value);

  void handleArmConfigurationCallback();
  void updateGoalMsg();
  bool makePlan();
  bool goalReached();
  bool dispatchCommands();
  bool withinBounds(unsigned waypointIndex);

  void setStartState(robot_srvs::NamedKinematicPlanState::request& req);
  void setCommandParameters(pr2_mechanism_controllers::JointPosCmd& cmd);

  const std::string armCmdTopic;
  const std::string kinematicModel;
  mechanism_control::MechanismState mechanismState;
  robot_srvs::NamedKinematicPlanState::response plan;
  unsigned int currentWaypoint; /*!< The waypoint in the plan that we are targetting */

protected:
  std::vector<std::string> jointNames_; /*< The collection of joint names of interest. Instantiate in the  derived class.*/
};


bool MoveArm::readJointValue(const mechanism_control::MechanismState& mechanismStateMsg, const std::string& name, double& value){
  for(unsigned int i = 0; i < mechanismStateMsg.get_joint_states_size(); i++){
    const std::string& jointName = mechanismStateMsg.joint_states[i].name;
    if(name == jointName){
      value = mechanismStateMsg.joint_states[i].position;
      return true;
    }
  }

  return false;
}

static const double L1_JOINT_DIFF_MAX = .05;

MoveArm::MoveArm(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
		 const std::string& armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel)
  : HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal>(nodeName, stateTopic, goalTopic),
    armCmdTopic(_armCmdTopic), kinematicModel(_kinematicModel), currentWaypoint(0){


  // Advertise for messages to command the arm
  advertise<pr2_mechanism_controllers::JointPosCmd>(armCmdTopic, QUEUE_MAX());

  // Advertise the display.
  advertise<robot_msgs::DisplayKinematicPath>("display_kinematic_path", 1);

  // Subscribe to arm configuration messages
  subscribe(armPosTopic, mechanismState, &MoveArm::handleArmConfigurationCallback, QUEUE_MAX());
}

MoveArm::~MoveArm(){}

void MoveArm::handleArmConfigurationCallback(){
  initialize();

  // Read available name value pairs
  std::vector<std::pair< std::string, double> > nameValuePairs;
  for(std::vector< std::string >::const_iterator it = jointNames_.begin(); it != jointNames_.end(); ++it){
    double value;
    const std::string& name = *it;
    if(readJointValue(mechanismState, name, value))
      nameValuePairs.push_back(std::pair<std::string, double>(name, value));
  }

  lock();

  // Now set the state msg up for publication
  stateMsg.set_configuration_size(nameValuePairs.size());
  for(unsigned int i = 0; i < nameValuePairs.size(); i++){
    stateMsg.configuration[i].name = nameValuePairs[i].first;
    stateMsg.configuration[i].position = nameValuePairs[i].second;
  }

  unlock();
}

void MoveArm::updateGoalMsg(){
  lock();
  stateMsg.goal = goalMsg.configuration;
  unlock();
}

bool MoveArm::makePlan(){
  std::cout << "Invoking Kinematic Planner\n";


  robot_srvs::PlanNames::request namesReq;
  robot_srvs::PlanNames::response names;
  ros::service::call("plan_joint_state_names", namesReq, names);

  //unsigned int needparams = 0;
  //for (unsigned int i = 0 ; i < names.get_names_size() && i < names.get_num_values_size() ; ++i) {
  //std::cout << names.names[i] << " (" << names.num_values[i] << ")\n";
  // needparams += names.num_values[i];
  //}
  //std::cout << "Need " << needparams << " Params\n";



  robot_srvs::NamedKinematicPlanState::request req;
    
  req.params.model_id = kinematicModel;
  req.params.distance_metric = "L2Square";
  req.threshold = 10e-06;
  req.interpolate = 0;
  req.times = 1;


  //initializing full value state
  req.start_state.set_names_size(names.get_names_size());
  req.start_state.set_joints_size(names.get_names_size());
  for (unsigned int i = 0 ; i < req.start_state.get_joints_size() ; ++i) {
    req.start_state.names[i] = names.names[i];
    //std::cout << req.start_state.names[i] << ": " << names.num_values[i] << std::endl;
    req.start_state.joints[i].set_vals_size(names.num_values[i]);
    for (int k = 0 ; k < names.num_values[i]; k++) {
      req.start_state.joints[i].vals[k] = 0; //FIXME: should be the value from mechanism state.
    }
  }

  // TODO: Adjust based on parameters for left vs right arms

  // Fill out the start state from current arm configuration
  //mechanismState.lock();
  //setStartState(req);
  //mechanismState.unlock();

  // Filling out goal state from data in the goal message
  goalMsg.lock();
  req.goal_state.set_names_size(goalMsg.get_configuration_size());
  req.goal_state.set_joints_size(goalMsg.get_configuration_size());
  for(unsigned int i = 0; i<goalMsg.get_configuration_size(); i++) {
    req.goal_state.names[i] = goalMsg.configuration[i].name;
    req.goal_state.joints[i].set_vals_size(1); //FIXME: multi-part joints?
    req.goal_state.joints[i].vals[0] = goalMsg.configuration[i].position;
  }

  goalMsg.unlock();

  req.allowed_time = 10.0;
  req.params.volumeMin.x = -1.0; req.params.volumeMin.y = -1.0; req.params.volumeMin.z = -1.0;
  req.params.volumeMax.x = -1.0; req.params.volumeMax.y = -1.0; req.params.volumeMax.z = -1.0;

  // Invoke kinematic motion planner
  ros::service::call("plan_kinematic_path_named", req, plan);
  unsigned int nstates = plan.path.get_states_size();

  // If all well, then there will be at least 2 states
  bool foundPlan = (nstates > 0);

  if (foundPlan) {
    currentWaypoint = 0;
    std::cout << "Obtained solution path with " << nstates << " states\n";
  } else {
    std::cout << "Service 'plan_kinematic_path_named' failed\n";
  }

  /*robot_msgs::DisplayKinematicPath dpath;
  dpath.frame_id = "robot";
  dpath.model_name = req.params.model_id;
  dpath.start_state = req.start_state;
  dpath.path = plan.path;
  publish("display_kinematic_path", dpath);*/


  return foundPlan;
}

/**
 * If the current waypoint has been reached then advance to the next.
 * If no new waypoint then return true, otherwise false.
 */
bool MoveArm::goalReached(){

  for(unsigned int i = currentWaypoint; i < plan.path.get_states_size(); i++){
    if(withinBounds(i)){
      std::cout << "Accomplished waypoint " << i << std::endl;
      currentWaypoint++;
    }
    else {
      return false;
    }
  }

  std::cout << "Goal Reached" << std::endl;
  return true;
}

/**
 * @brief Iterate over all published joint values we match on
 * @todo Does not handle joints with multiple axes.
 */
bool MoveArm::withinBounds(unsigned waypointIndex){
  double sum_joint_diff = 0.0;
  mechanismState.lock();
  for(unsigned int i=0; i<plan.path.states[waypointIndex].get_joints_size(); i++){
    double value;
    if(readJointValue(mechanismState, plan.path.states[waypointIndex].names[i], value))
       sum_joint_diff += fabs(value - plan.path.states[waypointIndex].joints[i].vals[0]);
  }
  mechanismState.unlock();

  if(L1_JOINT_DIFF_MAX > sum_joint_diff)
    return true;

  return false;
}

/**
 * @brief In principle we could be checking the feasibility of the plan here based on a local computation
 */
bool MoveArm::dispatchCommands(){
  if(currentWaypoint >= plan.path.get_states_size()){
    std::cout << "SetStateGoalFromPlan:: trying to set state greater than number of states in path.\n";
    return false;
  }

  pr2_mechanism_controllers::JointPosCmd armCommand;

  setCommandParameters(armCommand);
  
  std::cout << "Dispatching state for waypoint [" << currentWaypoint << "]: ";
  
  for(unsigned int i=0; i<plan.path.states[currentWaypoint].get_joints_size(); i++){
    std::cout << plan.path.states[currentWaypoint].names[i] << " (";
    for (unsigned int k=0; k<plan.path.states[currentWaypoint].joints[i].get_vals_size(); k++) {
      std::cout << plan.path.states[currentWaypoint].joints[i].vals[k] << ",";
    }
    std::cout << ") ";
  }

  std::cout << std::endl;

  publish(armCmdTopic, armCommand);

  return true;
}

/**
 * @todo Multi-axis joints.
 */
void MoveArm::setCommandParameters(pr2_mechanism_controllers::JointPosCmd& armCommand){
    static const double TOLERANCE(0.25);

    // Set up message size
    armCommand.set_names_size(jointNames_.size());
    armCommand.set_positions_size(jointNames_.size());
    armCommand.set_margins_size(jointNames_.size());

    for(unsigned int i = 0; i < jointNames_.size(); i++){
      armCommand.names[i] = plan.path.states[currentWaypoint].names[i];
      armCommand.positions[i] = plan.path.states[currentWaypoint].joints[i].vals[0];
      armCommand.margins[i] = TOLERANCE;
    }
}

void MoveArm::setStartState(robot_srvs::NamedKinematicPlanState::request& req){
    /*
    req.start_state.vals[33] = mechanismState.joint_states[base + 12].position;
    req.start_state.vals[34] = mechanismState.joint_states[base + 10].position;
    req.start_state.vals[35] = mechanismState.joint_states[base + 8].position;
    req.start_state.vals[36] = mechanismState.joint_states[base + 6].position;
    req.start_state.vals[37] = mechanismState.joint_states[base + 4].position;
    req.start_state.vals[38] = mechanismState.joint_states[base + 2].position;
    req.start_state.vals[39] = mechanismState.joint_states[base + 0].position;
    */
}

class MoveRightArm: public MoveArm {
public:
  MoveRightArm(): MoveArm("rightArmController", "right_arm_state", "right_arm_goal", "mechanism_state", "right_arm_commands", "pr2::right_arm"){
    jointNames_.push_back("shoulder_pan_right_joint");
    jointNames_.push_back("shoulder_pitch_right_joint");
    jointNames_.push_back("upperarm_roll_right_joint");
    jointNames_.push_back("elbow_flex_right_joint");
    jointNames_.push_back("forearm_roll_right_joint");
    jointNames_.push_back("wrist_flex_right_joint");
    jointNames_.push_back("gripper_roll_right_joint");
  };

protected:
};

class MoveLeftArm: public MoveArm {
public:
  MoveLeftArm(): MoveArm("leftArmController", "left_arm_state", "left_arm_goal", "mechanism_state", "left_arm_commands", "pr2::left_arm"){
    // Instantiate joint vector
    jointNames_.push_back("shoulder_pan_left_joint");
    jointNames_.push_back("shoulder_pitch_left_joint");
    jointNames_.push_back("upperarm_roll_left_joint");
    jointNames_.push_back("elbow_flex_left_joint");
    jointNames_.push_back("forearm_roll_left_joint");
    jointNames_.push_back("wrist_flex_left_joint");
    jointNames_.push_back("gripper_roll_left_joint");
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
