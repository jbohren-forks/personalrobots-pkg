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
 * @b move_end_effector is a highlevel controller for moving the end effector of the arm to a given point in space.
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
 *  $ move_end_effector left|right
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b "mechanism_state"/robot_msgs::MechanismState : The state of the robot joints and actuators
 * - @b "left/right_end_effector_goal"/pr2_msgs::MoveEndEffectorGoal: Where to move end effector
 *
 * Publishes to (name / type):
 * - @b "left_end_effector_state"/pr2_msgs::MoveEndEffectorState : Am I Active or 
 *
 *  <hr>
 *
 * @section parameters ROS parameters
 *
 * - None
 **/

#include <HighlevelController.hh>
#include <pr2_mechanism_controllers/JointPosCmd.h>
#include <robot_msgs/MechanismState.h>
#include <pr2_msgs/MoveEndEffectorState.h>
#include <pr2_msgs/MoveEndEffectorGoal.h>
#include <robot_srvs/KinematicPlanState.h>

static const unsigned int RIGHT_ARM_JOINTS_BASE_INDEX = 11;
static const unsigned int LEFT_ARM_JOINTS_BASE_INDEX = 12;

class MoveEndEffector : public HighlevelController<pr2_msgs::MoveEndEffectorState, pr2_msgs::MoveEndEffectorGoal> {

public:

  /**
   * @brief Constructor
   */
  MoveEndEffector(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
		     const std::string& _armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel);

  virtual ~MoveEndEffector();

private:

  /**
   * @brief Helper Method to obtain the joint value by name
   * @return true if the joint is present, otherwise false
   */
  bool readJointValue(const robot_msgs::MechanismState& mechanismStateMsg, const std::string& name, double& value);

  void handleArmConfigurationCallback();
  void updateGoalMsg();
  bool makePlan();
  bool goalReached();
  bool dispatchCommands();
  bool withinBounds(unsigned waypointIndex);

  void setStartState(robot_srvs::KinematicPlanState::request& req);
  void setCommandParameters(pr2_mechanism_controllers::JointPosCmd& cmd);

  const std::string armCmdTopic;
  const std::string kinematicModel;
  robot_msgs::MechanismState mechanismState;
  robot_srvs::KinematicPlanState::response plan;
  unsigned int currentWaypoint; /*!< The waypoint in the plan that we are targetting */

protected:
  std::vector<std::string> jointNames_; /*< The collection of joint names of interest. Instantiate in the  derived class.*/
};


bool MoveEndEffector::readJointValue(const robot_msgs::MechanismState& mechanismStateMsg, const std::string& name, double& value){
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

MoveEndEffector::MoveEndEffector(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
		 const std::string& armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel)
  : HighlevelController<pr2_msgs::MoveEndEffectorState, pr2_msgs::MoveEndEffectorGoal>(nodeName, stateTopic, goalTopic),
    armCmdTopic(_armCmdTopic), kinematicModel(_kinematicModel), currentWaypoint(0){

  // Subscribe to arm configuration messages
  subscribe(armPosTopic, mechanismState, &MoveEndEffector::handleArmConfigurationCallback, QUEUE_MAX());

  // Advertise for messages to command the arm
  advertise<pr2_mechanism_controllers::JointPosCmd>(armCmdTopic, QUEUE_MAX());
}

MoveEndEffector::~MoveEndEffector(){}

void MoveEndEffector::handleArmConfigurationCallback(){
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

void MoveEndEffector::updateGoalMsg(){
  lock();
  stateMsg.goal = goalMsg.configuration;
  unlock();
}

bool MoveEndEffector::makePlan(){
  std::cout << "Invoking Kinematic Planner\n";

  robot_srvs::KinematicPlanState::request req;
    
  req.params.model_id = kinematicModel;
  req.params.distance_metric = "L2Square";
  req.threshold = 10e-06;
  req.interpolate = 0;
  req.times = 1;
  req.start_state.set_vals_size(45);
    
  //initializing full value state
  for (unsigned int i = 0 ; i < req.start_state.get_vals_size() ; ++i) {
    req.start_state.vals[i] = 0.0;
  }

  // TODO: Adjust based on parameters for left vs right arms

  // Fill out the start state from current arm configuration
  //mechanismState.lock();
  //setStartState(req);
  //mechanismState.unlock();

  // Filling out goal state from data in the goal message
  goalMsg.lock();
  req.goal_state.set_vals_size(jointNames_.size());
  for(unsigned int i = 0; i<jointNames_.size(); i++)
    req.goal_state.vals[i] = goalMsg.configuration[i].position;

  goalMsg.unlock();

  req.allowed_time = 10.0;
  req.params.volumeMin.x = -1.0; req.params.volumeMin.y = -1.0; req.params.volumeMin.z = -1.0;
  req.params.volumeMax.x = -1.0; req.params.volumeMax.y = -1.0; req.params.volumeMax.z = -1.0;

  // Invoke kinematic motion planner
  ros::service::call("plan_kinematic_path_state", req, plan);
  unsigned int nstates = plan.path.get_states_size();

  // If all well, then there will be at least 2 states
  bool foundPlan = (nstates > 0);

  if (foundPlan) {
    currentWaypoint = 0;
    std::cout << "Obtained solution path with " << nstates << " states\n";
  } else {
    std::cout << "Service 'plan_kinematic_path_state' failed\n";
  }

  return foundPlan;
}

/**
 * If the current waypoint has been reached then advance to the next.
 * If no new waypoint then return true, otherwise false.
 */
bool MoveEndEffector::goalReached(){

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
 */
bool MoveEndEffector::withinBounds(unsigned waypointIndex){
  double sum_joint_diff = 0.0;
  mechanismState.lock();
  for(unsigned int i=0; i<jointNames_.size(); i++){
    double value;
    if(readJointValue(mechanismState, jointNames_[i], value))
       sum_joint_diff += fabs(value - plan.path.states[waypointIndex].vals[i]);
  }
  mechanismState.unlock();

  if(L1_JOINT_DIFF_MAX > sum_joint_diff)
    return true;

  return false;
}

/**
 * @brief In principle we could be checking the feasibility of the plan here based on a local computation
 */
bool MoveEndEffector::dispatchCommands(){
  if(currentWaypoint >= plan.path.get_states_size()){
    std::cout << "SetStateGoalFromPlan:: trying to set state greater than number of states in path.\n";
    return false;
  }

  pr2_mechanism_controllers::JointPosCmd armCommand;

  setCommandParameters(armCommand);
  
  std::cout << "Dispatching state for waypoint [" << currentWaypoint << "]: " 
	    << plan.path.states[currentWaypoint].vals[0] << " "
	    << plan.path.states[currentWaypoint].vals[1] << " "
	    << plan.path.states[currentWaypoint].vals[2] << " "
	    << plan.path.states[currentWaypoint].vals[3] << " "
	    << plan.path.states[currentWaypoint].vals[4] << " "
	    << plan.path.states[currentWaypoint].vals[5] << " "
	    << plan.path.states[currentWaypoint].vals[6] << std::endl;

  publish(armCmdTopic, armCommand);

  return true;
}


void MoveEndEffector::setCommandParameters(pr2_mechanism_controllers::JointPosCmd& armCommand){
    static const double TOLERANCE(0.25);

    // Set up message size
    armCommand.set_names_size(jointNames_.size());
    armCommand.set_positions_size(jointNames_.size());
    armCommand.set_margins_size(jointNames_.size());

    for(unsigned int i = 0; i < jointNames_.size(); i++){
      armCommand.names[i] = jointNames_[i];
      armCommand.positions[i] = plan.path.states[currentWaypoint].vals[i];
      armCommand.margins[i] = TOLERANCE;
    }
}

void MoveEndEffector::setStartState(robot_srvs::KinematicPlanState::request& req){
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

class MoveRightEndEffector: public MoveEndEffector {
public:
  MoveRightEndEffector(): MoveEndEffector("rightEndEffectorController", "right_end_effector_state", "right_end_effector_goal", "mechanism_state", "right_arm_commands", "pr2::right_arm"){
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

class MoveLeftEndEffector: public MoveEndEffector {
public:
  MoveLeftEndEffector(): MoveEndEffector("leftEndEffectorController", "left_end_effector_state", "left_end_effector_goal", "mechanism_state", "left_arm_commands", "pr2::left_arm"){
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
    std::cout << "Usage: ./move_end_effector left|right";
    return -1;
  }

  ros::init(argc,argv);

  // Extract parameters
  const std::string param = argv[1];

  if(param == "left"){
    MoveLeftEndEffector node;
    node.run();
    ros::fini();
  }
  else if(param == "right"){
    MoveRightEndEffector node;
    node.run();
    ros::fini();
  }
  else {
    std::cout << "Usage: ./move_end_effector left|right";
    return -1;
  }

  return(0);
}
