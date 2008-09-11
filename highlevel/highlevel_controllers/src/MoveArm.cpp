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
#include <std_msgs/PR2Arm.h>
#include <HighlevelController.hh>
#include <pr2_msgs/MoveArmState.h>
#include <pr2_msgs/MoveArmGoal.h>
#include <robot_srvs/KinematicPlanState.h>

class MoveArm : public HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal> {

public:

  /**
   * @brief Constructor
   */
  MoveArm(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
	  const std::string& _armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel);

  virtual ~MoveArm();

protected:

  void handleArmConfigurationCallback();
  void updateStateMsg();
  bool makePlan();
  bool goalReached();
  bool dispatchCommands();
  bool withinBounds(unsigned waypointIndex);

  virtual void setStartState(robot_srvs::KinematicPlanState::request& req) = 0;

  const std::string armCmdTopic;
  const std::string kinematicModel;
  std_msgs::PR2Arm armConfiguration;
  robot_srvs::KinematicPlanState::response plan;
  unsigned int currentWaypoint; /*!< The waypoint in the plan that we are targetting */
};


static const double L1_JOINT_DIFF_MAX = .05;
static const double L1_GRIP_FORCE_DIFF_MAX = .01;
static const double L1_GRIP_GAP_DIFF_MAX = .135;

MoveArm::MoveArm(const std::string& nodeName, const std::string& stateTopic, const std::string& goalTopic,
		 const std::string& armPosTopic, const std::string& _armCmdTopic, const std::string& _kinematicModel)
  : HighlevelController<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal>(nodeName, stateTopic, goalTopic),
    armCmdTopic(_armCmdTopic), kinematicModel(_kinematicModel), currentWaypoint(0){

  // Subscribe to arm configuration messages
  subscribe(armPosTopic, armConfiguration, &MoveArm::handleArmConfigurationCallback, QUEUE_MAX());

  // Advertise for messages to command the arm
  advertise<std_msgs::PR2Arm>(armCmdTopic, QUEUE_MAX());
}

MoveArm::~MoveArm(){}

void MoveArm::handleArmConfigurationCallback(){
  initialize();
}

void MoveArm::updateStateMsg(){
  armConfiguration.lock();
  stateMsg.configuration.turretAngle = armConfiguration.turretAngle;
  stateMsg.configuration.shoulderLiftAngle = armConfiguration.shoulderLiftAngle;
  stateMsg.configuration.upperarmRollAngle = armConfiguration.upperarmRollAngle;
  stateMsg.configuration.elbowAngle  = armConfiguration.elbowAngle;
  stateMsg.configuration.forearmRollAngle  = armConfiguration.forearmRollAngle;
  stateMsg.configuration.wristPitchAngle  = armConfiguration.wristPitchAngle;
  stateMsg.configuration.wristRollAngle = armConfiguration.wristRollAngle;
  stateMsg.configuration.gripperForceCmd = armConfiguration.gripperForceCmd;
  stateMsg.configuration.gripperGapCmd = armConfiguration.gripperGapCmd;
  armConfiguration.unlock();

  if(isActive()){
    goalMsg.lock();
    stateMsg.goal.turretAngle = goalMsg.configuration.turretAngle;
    stateMsg.goal.shoulderLiftAngle = goalMsg.configuration.shoulderLiftAngle;
    stateMsg.goal.upperarmRollAngle = goalMsg.configuration.upperarmRollAngle;
    stateMsg.goal.elbowAngle  = goalMsg.configuration.elbowAngle;
    stateMsg.goal.forearmRollAngle  = goalMsg.configuration.forearmRollAngle;
    stateMsg.goal.wristPitchAngle  = goalMsg.configuration.wristPitchAngle;
    stateMsg.goal.wristRollAngle = goalMsg.configuration.wristRollAngle;
    stateMsg.goal.gripperForceCmd = goalMsg.configuration.gripperForceCmd;
    stateMsg.goal.gripperGapCmd = goalMsg.configuration.gripperGapCmd;
    goalMsg.unlock();
  }
  else {
    stateMsg.goal.turretAngle = stateMsg.configuration.turretAngle;
    stateMsg.goal.shoulderLiftAngle = stateMsg.configuration.shoulderLiftAngle;
    stateMsg.goal.upperarmRollAngle = stateMsg.configuration.upperarmRollAngle;
    stateMsg.goal.elbowAngle  = stateMsg.configuration.elbowAngle;
    stateMsg.goal.forearmRollAngle  = stateMsg.configuration.forearmRollAngle;
    stateMsg.goal.wristPitchAngle  = stateMsg.configuration.wristPitchAngle;
    stateMsg.goal.wristRollAngle = stateMsg.configuration.wristRollAngle;
    stateMsg.goal.gripperForceCmd = stateMsg.configuration.gripperForceCmd;
    stateMsg.goal.gripperGapCmd = stateMsg.configuration.gripperGapCmd;
  }
}

bool MoveArm::makePlan(){
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
  armConfiguration.lock();
  setStartState(req);
  armConfiguration.unlock();

  // Filling out goal state from data in the goal message
  goalMsg.lock();
  req.goal_state.set_vals_size(7);
  req.goal_state.vals[0] = goalMsg.configuration.turretAngle;
  req.goal_state.vals[1] = goalMsg.configuration.shoulderLiftAngle;
  req.goal_state.vals[2] = goalMsg.configuration.upperarmRollAngle;
  req.goal_state.vals[3] = goalMsg.configuration.elbowAngle;
  req.goal_state.vals[4] = goalMsg.configuration.forearmRollAngle;
  req.goal_state.vals[5] = goalMsg.configuration.wristPitchAngle;
  req.goal_state.vals[6] = goalMsg.configuration.wristRollAngle;
  goalMsg.unlock();

  req.allowed_time = 10.0;
  req.params.volumeMin.x = -1.0; req.params.volumeMin.y = -1.0; req.params.volumeMin.z = -1.0;
  req.params.volumeMax.x = -1.0; req.params.volumeMax.y = -1.0; req.params.volumeMax.z = -1.0;

  // Invoke kinematic motion planner
  bool foundPlan = ros::service::call("plan_kinematic_path_state", req, plan);
  if (foundPlan) {
    unsigned int nstates = plan.path.get_states_size();
    currentWaypoint = 0;
    printf("Obtained solution path with %u states\n", nstates);
  } else {
    fprintf(stderr, "Service 'plan_kinematic_path_state' failed\n"); 	
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

  return true;
}

bool MoveArm::withinBounds(unsigned waypointIndex){
  double sum_joint_diff = 0.0;
  armConfiguration.lock();
  sum_joint_diff += fabs(armConfiguration.turretAngle-plan.path.states[waypointIndex].vals[0]);
  sum_joint_diff += fabs(armConfiguration.shoulderLiftAngle-plan.path.states[waypointIndex].vals[1]);
  sum_joint_diff += fabs(armConfiguration.upperarmRollAngle-plan.path.states[waypointIndex].vals[2]);
  sum_joint_diff += fabs(armConfiguration.elbowAngle-plan.path.states[waypointIndex].vals[3]);
  sum_joint_diff += fabs(armConfiguration.forearmRollAngle-plan.path.states[waypointIndex].vals[4]);
  sum_joint_diff += fabs(armConfiguration.wristPitchAngle-plan.path.states[waypointIndex].vals[5]);
  sum_joint_diff += fabs(armConfiguration.wristRollAngle-plan.path.states[waypointIndex].vals[6]);

  double force_diff = fabs(armConfiguration.gripperForceCmd);
  double gap_diff = fabs(armConfiguration.gripperGapCmd-goalMsg.configuration.gripperGapCmd);
  armConfiguration.unlock();

  if(L1_JOINT_DIFF_MAX > sum_joint_diff && 
     L1_GRIP_GAP_DIFF_MAX > gap_diff &&
     L1_GRIP_FORCE_DIFF_MAX > force_diff)
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

  std_msgs::PR2Arm armCommand;
  armCommand.turretAngle = plan.path.states[currentWaypoint].vals[0];
  armCommand.shoulderLiftAngle = plan.path.states[currentWaypoint].vals[1];
  armCommand.upperarmRollAngle = plan.path.states[currentWaypoint].vals[2];
  armCommand.elbowAngle = plan.path.states[currentWaypoint].vals[3];
  armCommand.forearmRollAngle = plan.path.states[currentWaypoint].vals[4];
  armCommand.wristPitchAngle = plan.path.states[currentWaypoint].vals[5];
  armCommand.wristRollAngle = plan.path.states[currentWaypoint].vals[6];  
  armCommand.gripperForceCmd = goalMsg.configuration.gripperForceCmd;
  armCommand.gripperGapCmd = goalMsg.configuration.gripperGapCmd;
  
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
  MoveRightArm(): MoveArm("rightArmController", "right_arm_state", "right_arm_goal", "right_pr2arm_pos", "cmd_rightarmconfig", "pr2::right_arm"){}

protected:
  void setStartState(robot_srvs::KinematicPlanState::request& req){
    req.start_state.vals[33] = armConfiguration.turretAngle;
    req.start_state.vals[34] = armConfiguration.shoulderLiftAngle;
    req.start_state.vals[35] = armConfiguration.upperarmRollAngle;
    req.start_state.vals[36] = armConfiguration.elbowAngle;
    req.start_state.vals[37] = armConfiguration.forearmRollAngle;
    req.start_state.vals[38] = armConfiguration.wristPitchAngle;
    req.start_state.vals[39] = armConfiguration.wristRollAngle;
  }
};

class MoveLeftArm: public MoveArm {
public:
  MoveLeftArm(): MoveArm("leftArmController", "left_arm_state", "left_arm_goal", "left_pr2arm_pos", "cmd_leftarmconfig", "pr2::left_arm"){}

protected:
  void setStartState(robot_srvs::KinematicPlanState::request& req){
    req.start_state.vals[22] = armConfiguration.turretAngle;
    req.start_state.vals[23] = armConfiguration.shoulderLiftAngle;
    req.start_state.vals[24] = armConfiguration.upperarmRollAngle;
    req.start_state.vals[25] = armConfiguration.elbowAngle;
    req.start_state.vals[26] = armConfiguration.forearmRollAngle;
    req.start_state.vals[27] = armConfiguration.wristPitchAngle;
    req.start_state.vals[28] = armConfiguration.wristRollAngle;
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
