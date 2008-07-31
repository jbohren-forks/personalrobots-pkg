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

#include <ros/node.h>
#include <std_msgs/PR2Arm.h>
#include <robot_srvs/KinematicMotionPlan.h>

static const double L1JointDiffMax = .01;
static const double L1GripDiffMax = .02;

class IssueKinematicPlan : public ros::node {

public:

  IssueKinematicPlan(void) : ros::node("issue_kinematic_plan") {
    advertise<std_msgs::PR2Arm>("cmd_leftarmconfig");
    advertise<std_msgs::PR2Arm>("cmd_rightarmconfig");
    subscribe("left_pr2arm_pos",  leftArmPos,  &IssueKinematicPlan::currentLeftArmPos);
    subscribe("right_pr2arm_pos", rightArmPos, &IssueKinematicPlan::currentRightArmPos);
    
    subscribe("right_pr2arm_set_position", rightArmGoal, &IssueKinematicPlan::moveRightArm);	
    subscribe("left_pr2arm_set_position", leftArmGoal, &IssueKinematicPlan::moveLeftArm);

    _leftArmExecutingMove = false;
    _leftArmMoveStates = 0;
    _leftArmStateSent = 0;

    _rightArmExecutingMove = false;
    _rightArmMoveStates = 0;
    _rightArmStateSent = 0;
  }

private:

  void currentLeftArmPos(void) {

    if(_leftArmExecutingMove) {

      //l1 norm for now

      double sum_joint_diff = 0.0;
      sum_joint_diff += abs(leftArmPos.turretAngle-_leftArmCommand.turretAngle);
      sum_joint_diff += abs(leftArmPos.shoulderLiftAngle-_leftArmCommand.shoulderLiftAngle);
      sum_joint_diff += abs(leftArmPos.upperarmRollAngle-_leftArmCommand.upperarmRollAngle);
      sum_joint_diff += abs(leftArmPos.elbowAngle-_leftArmCommand.elbowAngle);
      sum_joint_diff += abs(leftArmPos.forearmRollAngle-_leftArmCommand.forearmRollAngle);
      sum_joint_diff += abs(leftArmPos.wristPitchAngle-_leftArmCommand.wristPitchAngle);
      sum_joint_diff += abs(leftArmPos.wristRollAngle-_leftArmCommand.wristRollAngle);

      double gap_diff = abs(leftArmPos.gripperGapCmd-_leftArmCommand.gripperGapCmd);

      if(L1JointDiffMax < sum_joint_diff && L1GripDiffMax < gap_diff) {
	_leftArmStateSent++;
	if(_leftArmStateSent == _leftArmMoveStates) {
	  //we're done
	  std::cout << "Successfully completed left arm path.\n";
	  _leftArmExecutingMove = false;
	  _leftArmMoveStates = 0;
	  _leftArmStateSent = 0;
	  return;
	} else {
	  SetStateGoalFromPlan(_leftArmStateSent, _leftPlanRes, _leftArmCommand);
	  publish("cmd_leftarmconfig", _leftArmCommand);
	}
      }
      //otherwise we just chill out
    }

  }
  
  void currentRightArmPos(void) {
    if(_rightArmExecutingMove) {
      
      //l1 norm for now
      
      double sum_joint_diff = 0.0;
      sum_joint_diff += abs(rightArmPos.turretAngle-_rightArmCommand.turretAngle);
      sum_joint_diff += abs(rightArmPos.shoulderLiftAngle-_rightArmCommand.shoulderLiftAngle);
      sum_joint_diff += abs(rightArmPos.upperarmRollAngle-_rightArmCommand.upperarmRollAngle);
      sum_joint_diff += abs(rightArmPos.elbowAngle-_rightArmCommand.elbowAngle);
      sum_joint_diff += abs(rightArmPos.forearmRollAngle-_rightArmCommand.forearmRollAngle);
      sum_joint_diff += abs(rightArmPos.wristPitchAngle-_rightArmCommand.wristPitchAngle);
      sum_joint_diff += abs(rightArmPos.wristRollAngle-_rightArmCommand.wristRollAngle);
      
      double gap_diff = abs(rightArmPos.gripperGapCmd-_rightArmCommand.gripperGapCmd);
      
      if(L1JointDiffMax < sum_joint_diff && L1GripDiffMax < gap_diff) {
	_rightArmStateSent++;
	if(_rightArmStateSent == _rightArmMoveStates) {
	  //we're done
	  std::cout << "Successfully completed right arm path.\n";
	  _rightArmExecutingMove = false;
	  _rightArmMoveStates = 0;
	  _rightArmStateSent = 0;
	  return;
	} else {
	  SetStateGoalFromPlan(_rightArmStateSent, _rightPlanRes, _rightArmCommand);
	  publish("cmd_rightarmconfig", _rightArmCommand);
	}
      }
      //otherwise we just chill out
    }
  }
  
  void moveLeftArm() {
    
    robot_srvs::KinematicMotionPlan::request req;
    
    req.model_id = "pr2::leftArm";
    req.start_state.set_vals_size(32);
    
    //initializing full value state
    for (unsigned int i = 0 ; i < req.start_state.vals_size ; ++i) {
      req.start_state.vals[i] = 0.0;
    }

    req.start_state.vals[18] = leftArmPos.turretAngle;
    req.start_state.vals[19] = leftArmPos.shoulderLiftAngle;
    req.start_state.vals[20] = leftArmPos.upperarmRollAngle;
    req.start_state.vals[21] = leftArmPos.elbowAngle;
    req.start_state.vals[22] = leftArmPos.forearmRollAngle;
    req.start_state.vals[23] = leftArmPos.wristPitchAngle;
    req.start_state.vals[24] = leftArmPos.wristRollAngle;
    //req.start_state.vals[25] = leftArmPos.gripperForceCmd;
    //req.start_state.vals[26] = leftArmPos.gripperGapCmd;
    
    //filling out goal state
    req.goal_state.set_vals_size(7);
    req.goal_state.vals[0] = leftArmGoal.turretAngle;
    req.goal_state.vals[1] = leftArmGoal.shoulderLiftAngle;
    req.goal_state.vals[2] = leftArmGoal.upperarmRollAngle;
    req.goal_state.vals[3] = leftArmGoal.elbowAngle;
    req.goal_state.vals[4] = leftArmGoal.forearmRollAngle;
    req.goal_state.vals[5] = leftArmGoal.wristPitchAngle;
    req.goal_state.vals[6] = leftArmGoal.wristRollAngle;
    
    req.allowed_time = 2.0;
    
    req.volumeMin.x = -1.0; req.volumeMin.y = -1.0; req.volumeMin.z = -1.0;
    req.volumeMax.x = -1.0; req.volumeMax.y = -1.0; req.volumeMax.z = -1.0;
   
    unsigned int nstates = 0;
    if (ros::service::call("plan_kinematic_path", req, _leftPlanRes)) {
      nstates = _leftPlanRes.path.get_states_size();
      printf("Obtained solution path with %u states\n", nstates);
    } else {
      fprintf(stderr, "Service 'plan_kinematic_path' failed\n");	
    }

    //now we have to execute the plan

    if(nstates > 0) {
      _leftArmExecutingMove = true;
      _leftArmMoveStates = nstates;
      _leftArmStateSent = 0;
      SetStateGoalFromPlan(0, _leftPlanRes, _leftArmCommand);
      //setting gripper stuff for all commands
      _leftArmCommand.gripperForceCmd = leftArmGoal.gripperForceCmd;
      _leftArmCommand.gripperGapCmd = leftArmGoal.gripperGapCmd;
      publish("cmd_leftarmconfig", _leftArmCommand);
    }
  }
  
  void moveRightArm() {
    
    robot_srvs::KinematicMotionPlan::request req;
    
    req.model_id = "pr2::rightArm";
    req.start_state.set_vals_size(32);
    
    //initializing full value state
    for (unsigned int i = 0 ; i < req.start_state.vals_size ; ++i) {
      req.start_state.vals[i] = 0.0;
    }

    req.start_state.vals[25] = rightArmPos.turretAngle;
    req.start_state.vals[26] = rightArmPos.shoulderLiftAngle;
    req.start_state.vals[27] = rightArmPos.upperarmRollAngle;
    req.start_state.vals[28] = rightArmPos.elbowAngle;
    req.start_state.vals[29] = rightArmPos.forearmRollAngle;
    req.start_state.vals[30] = rightArmPos.wristPitchAngle;
    req.start_state.vals[31] = rightArmPos.wristRollAngle;
    //req.start_state.vals[25] = rightArmPos.gripperForceCmd;
    //req.start_state.vals[26] = rightArmPos.gripperGapCmd;
    
    //filling out goal state
    req.goal_state.set_vals_size(7);
    req.goal_state.vals[0] = rightArmGoal.turretAngle;
    req.goal_state.vals[1] = rightArmGoal.shoulderLiftAngle;
    req.goal_state.vals[2] = rightArmGoal.upperarmRollAngle;
    req.goal_state.vals[3] = rightArmGoal.elbowAngle;
    req.goal_state.vals[4] = rightArmGoal.forearmRollAngle;
    req.goal_state.vals[5] = rightArmGoal.wristPitchAngle;
    req.goal_state.vals[6] = rightArmGoal.wristRollAngle;
    
    req.allowed_time = 2.0;
    
    req.volumeMin.x = -1.0; req.volumeMin.y = -1.0; req.volumeMin.z = -1.0;
    req.volumeMax.x = -1.0; req.volumeMax.y = -1.0; req.volumeMax.z = -1.0;
   
    unsigned int nstates = 0;
    if (ros::service::call("plan_kinematic_path", req, _rightPlanRes)) {
      nstates = _rightPlanRes.path.get_states_size();
      printf("Obtained solution path with %u states\n", nstates);
    } else {
      fprintf(stderr, "Service 'plan_kinematic_path' failed\n");	
    }

    //now we have to execute the plan

    if(nstates > 0) {
      _rightArmExecutingMove = true;
      _rightArmMoveStates = nstates;
      _rightArmStateSent = 0;
      SetStateGoalFromPlan(0, _rightPlanRes, _rightArmCommand);
      //setting gripper stuff for all commands
      _rightArmCommand.gripperForceCmd = rightArmGoal.gripperForceCmd;
      _rightArmCommand.gripperGapCmd = rightArmGoal.gripperGapCmd;
      publish("cmd_rightarmconfig", _rightArmCommand);
    }
  }

private:

  void SetStateGoalFromPlan(unsigned int stateNum, const robot_srvs::KinematicMotionPlan::response& res,
			    std_msgs::PR2Arm& armCom) {
    if(stateNum >= res.path.get_states_size()) {
      printf("SetStateGoalFromPlan:: trying to set state greater than number of states in path.\n");
      return;
    }
    armCom.turretAngle = res.path.states[stateNum].vals[0];
    armCom.shoulderLiftAngle = res.path.states[stateNum].vals[1];
    armCom.upperarmRollAngle = res.path.states[stateNum].vals[2];
    armCom.elbowAngle = res.path.states[stateNum].vals[3];
    armCom.forearmRollAngle = res.path.states[stateNum].vals[4];
    armCom.wristPitchAngle = res.path.states[stateNum].vals[5];
    armCom.wristRollAngle = res.path.states[stateNum].vals[6];  
  }

private:

  robot_srvs::KinematicMotionPlan::response _leftPlanRes;
  robot_srvs::KinematicMotionPlan::response _rightPlanRes;
  
  std_msgs::PR2Arm _leftArmCommand;
  std_msgs::PR2Arm _rightArmCommand;
  
  std_msgs::PR2Arm leftArmPos;
  std_msgs::PR2Arm rightArmPos;
  std_msgs::PR2Arm leftArmGoal;
  std_msgs::PR2Arm rightArmGoal;

  bool _leftArmExecutingMove;
  bool _rightArmExecutingMove;

  unsigned int _leftArmMoveStates;
  unsigned int _rightArmMoveStates;

  unsigned int _leftArmStateSent;
  unsigned int _rightArmStateSent;

};

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    IssueKinematicPlan issue;
    sleep(1);

    issue.spin();
        
    issue.shutdown();
    
    return 0;    
}
