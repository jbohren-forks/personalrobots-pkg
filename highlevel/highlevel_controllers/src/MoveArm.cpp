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

#include <HighlevelController.hh>
#include <pr2_controllers/JointPosCmd.h>
#include <mechanism_control/MechanismState.h>
#include <pr2_msgs/MoveArmState.h>
#include <pr2_msgs/MoveArmGoal.h>
#include <robot_srvs/KinematicPlanState.h>

static const unsigned int RIGHT_ARM_JOINTS_BASE_INDEX = 11;
static const unsigned int LEFT_ARM_JOINTS_BASE_INDEX = 12;

class MoveArm : public HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal> {

public:

  /**
   * @brief Constructor
   */
  MoveArm(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
	  const std::string& _armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel, unsigned int base_);

  virtual ~MoveArm();

protected:

  void handleArmConfigurationCallback();
  void updateStateMsg();
  bool makePlan();
  bool goalReached();
  bool dispatchCommands();
  bool withinBounds(unsigned waypointIndex);

  virtual void setStartState(robot_srvs::KinematicPlanState::request& req) = 0;
  virtual void setCommandParameters(pr2_controllers::JointPosCmd& cmd) = 0;

  const std::string armCmdTopic;
  const std::string kinematicModel;
  mechanism_control::MechanismState mechanismState;
  robot_srvs::KinematicPlanState::response plan;
  unsigned int currentWaypoint; /*!< The waypoint in the plan that we are targetting */
  unsigned int base; /*!< Base from which offsets in mechanism state are obtained */
};


static const double L1_JOINT_DIFF_MAX = .05;

MoveArm::MoveArm(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
		 const std::string& armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel, unsigned int base_)
  : HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal>(nodeName, stateTopic, goalTopic),
    armCmdTopic(_armCmdTopic), kinematicModel(_kinematicModel), currentWaypoint(0), base(base_){

  // Subscribe to arm configuration messages
  subscribe(armPosTopic, mechanismState, &MoveArm::handleArmConfigurationCallback, QUEUE_MAX());

  // Advertise for messages to command the arm
  advertise<pr2_controllers::JointPosCmd>(armCmdTopic, QUEUE_MAX());
}

MoveArm::~MoveArm(){}

void MoveArm::handleArmConfigurationCallback(){
  initialize();
}

void MoveArm::updateStateMsg(){
  mechanismState.lock();
  stateMsg.configuration.shoulder_pan = mechanismState.joint_states[base + 12].position;
  stateMsg.configuration.shoulder_pitch = mechanismState.joint_states[base + 10].position;
  stateMsg.configuration.upperarm_roll = mechanismState.joint_states[base + 8].position;
  stateMsg.configuration.elbow_flex = mechanismState.joint_states[base + 6].position;
  stateMsg.configuration.forearm_roll = mechanismState.joint_states[base + 4].position;
  stateMsg.configuration.wrist_flex = mechanismState.joint_states[base + 2].position;
  stateMsg.configuration.gripper_roll = mechanismState.joint_states[base + 0].position;
  mechanismState.unlock();

  if(isActive()){
    goalMsg.lock();
    stateMsg.goal = goalMsg.configuration;
    goalMsg.unlock();
  }
  else {
    stateMsg.goal = stateMsg.configuration;
  }
}

bool MoveArm::makePlan(){
  std::cout << "Invoking Kinematic Planner\n";

  robot_srvs::KinematicPlanState::request req;
    
  req.params.model_id = kinematicModel;
  req.params.distance_metric = "L2Square";
  req.threshold = 10e-06;
  req.interpolate = 0;
  req.times = 1;
  req.start_state.set_vals_size(45);
    
  //initializing full value state
  for (unsigned int i = 0 ; i < req.start_state.vals_size ; ++i) {
    req.start_state.vals[i] = 0.0;
  }

  // TODO: Adjust based on parameters for left vs right arms

  // Fill out the start state from current arm configuration
  mechanismState.lock();
  setStartState(req);
  mechanismState.unlock();

  // Filling out goal state from data in the goal message
  goalMsg.lock();
  req.goal_state.set_vals_size(7);
  req.goal_state.vals[0] = goalMsg.configuration.shoulder_pan;
  req.goal_state.vals[1] = goalMsg.configuration.shoulder_pitch;
  req.goal_state.vals[2] = goalMsg.configuration.upperarm_roll;
  req.goal_state.vals[3] = goalMsg.configuration.elbow_flex;
  req.goal_state.vals[4] = goalMsg.configuration.forearm_roll;
  req.goal_state.vals[5] = goalMsg.configuration.wrist_flex;
  req.goal_state.vals[6] = goalMsg.configuration.gripper_roll;
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

bool MoveArm::withinBounds(unsigned waypointIndex){
  double sum_joint_diff = 0.0;
  mechanismState.lock();
  sum_joint_diff += fabs(mechanismState.joint_states[base + 12].position - plan.path.states[waypointIndex].vals[0]);
  sum_joint_diff += fabs(mechanismState.joint_states[base + 10].position - plan.path.states[waypointIndex].vals[1]);
  sum_joint_diff += fabs(mechanismState.joint_states[base + 8].position - plan.path.states[waypointIndex].vals[2]);
  sum_joint_diff += fabs(mechanismState.joint_states[base + 6].position - plan.path.states[waypointIndex].vals[3]);
  sum_joint_diff += fabs(mechanismState.joint_states[base + 4].position - plan.path.states[waypointIndex].vals[4]);
  sum_joint_diff += fabs(mechanismState.joint_states[base + 2].position - plan.path.states[waypointIndex].vals[5]);
  sum_joint_diff += fabs(mechanismState.joint_states[base + 0].position - plan.path.states[waypointIndex].vals[6]);
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
    printf("SetStateGoalFromPlan:: trying to set state greater than number of states in path.\n");
    return false;
  }

  pr2_controllers::JointPosCmd armCommand;

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

class MoveRightArm: public MoveArm {
public:
  MoveRightArm(): MoveArm("rightArmController", "right_arm_state", "right_arm_goal", "mechanism_state", "right_arm_commands", "pr2::right_arm", RIGHT_ARM_JOINTS_BASE_INDEX){}

protected:
  void setStartState(robot_srvs::KinematicPlanState::request& req){
    req.start_state.vals[33] = mechanismState.joint_states[base + 12].position;
    req.start_state.vals[34] = mechanismState.joint_states[base + 10].position;
    req.start_state.vals[35] = mechanismState.joint_states[base + 8].position;
    req.start_state.vals[36] = mechanismState.joint_states[base + 6].position;
    req.start_state.vals[37] = mechanismState.joint_states[base + 4].position;
    req.start_state.vals[38] = mechanismState.joint_states[base + 2].position;
    req.start_state.vals[39] = mechanismState.joint_states[base + 0].position;
  }

  void setCommandParameters(pr2_controllers::JointPosCmd& armCommand){
    static const double TOLERANCE(0.25);
    armCommand.set_names_size(7);
    armCommand.set_positions_size(7);
    armCommand.set_margins_size(7);

    armCommand.names[0] = "shoulder_pan_right_joint";
    armCommand.positions[0] = plan.path.states[currentWaypoint].vals[0];
    armCommand.margins[0] = TOLERANCE;

    armCommand.names[1] = "shoulder_pitch_right_joint";
    armCommand.positions[1] = plan.path.states[currentWaypoint].vals[1];
    armCommand.margins[1] = TOLERANCE;

    armCommand.names[2] = "upperarm_roll_right_joint";
    armCommand.positions[2] = plan.path.states[currentWaypoint].vals[2];
    armCommand.margins[2] = TOLERANCE;

    armCommand.names[3] = "elbow_flex_right_joint";
    armCommand.positions[3] = plan.path.states[currentWaypoint].vals[3];
    armCommand.margins[3] = TOLERANCE;

    armCommand.names[4] = "forearm_roll_right_joint";
    armCommand.positions[4] = plan.path.states[currentWaypoint].vals[4];
    armCommand.margins[4] = TOLERANCE;

    armCommand.names[5] = "wrist_flex_right_joint";
    armCommand.positions[5] = plan.path.states[currentWaypoint].vals[5];
    armCommand.margins[5] = TOLERANCE;

    armCommand.names[6] = "gripper_roll_right_joint";
    armCommand.positions[6] = plan.path.states[currentWaypoint].vals[6];
    armCommand.margins[6] = TOLERANCE;
  }
};

class MoveLeftArm: public MoveArm {
public:
  MoveLeftArm(): MoveArm("leftArmController", "left_arm_state", "left_arm_goal", "mechanism_state", "left_arm_commands", "pr2::left_arm", LEFT_ARM_JOINTS_BASE_INDEX){}

protected:
  void setStartState(robot_srvs::KinematicPlanState::request& req){
    req.start_state.vals[22] = mechanismState.joint_states[base + 12].position;
    req.start_state.vals[23] = mechanismState.joint_states[base + 10].position;
    req.start_state.vals[24] = mechanismState.joint_states[base + 8].position;
    req.start_state.vals[25] = mechanismState.joint_states[base + 6].position;
    req.start_state.vals[26] = mechanismState.joint_states[base + 4].position;
    req.start_state.vals[27] = mechanismState.joint_states[base + 2].position;
    req.start_state.vals[28] = mechanismState.joint_states[base + 0].position;
  }

  void setCommandParameters(pr2_controllers::JointPosCmd& armCommand){
    static const double TOLERANCE(0.25);
    armCommand.set_names_size(7);
    armCommand.set_positions_size(7);
    armCommand.set_margins_size(7);

    armCommand.names[0] = "shoulder_pan_left_joint";
    armCommand.positions[0] = plan.path.states[currentWaypoint].vals[0];
    armCommand.margins[0] = TOLERANCE;

    armCommand.names[1] = "shoulder_pitch_left_joint";
    armCommand.positions[1] = plan.path.states[currentWaypoint].vals[1];
    armCommand.margins[1] = TOLERANCE;

    armCommand.names[2] = "upperarm_roll_left_joint";
    armCommand.positions[2] = plan.path.states[currentWaypoint].vals[2];
    armCommand.margins[2] = TOLERANCE;

    armCommand.names[3] = "elbow_flex_left_joint";
    armCommand.positions[3] = plan.path.states[currentWaypoint].vals[3];
    armCommand.margins[3] = TOLERANCE;

    armCommand.names[4] = "forearm_roll_left_joint";
    armCommand.positions[4] = plan.path.states[currentWaypoint].vals[4];
    armCommand.margins[4] = TOLERANCE;

    armCommand.names[5] = "wrist_flex_left_joint";
    armCommand.positions[5] = plan.path.states[currentWaypoint].vals[5];
    armCommand.margins[5] = TOLERANCE;

    armCommand.names[6] = "gripper_roll_left_joint";
    armCommand.positions[6] = plan.path.states[currentWaypoint].vals[6];
    armCommand.margins[6] = TOLERANCE;
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
