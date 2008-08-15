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
#include <pr2_msgs/MoveArmGoal.h>
#include <pr2_msgs/MoveArmState.h>
#include <robot_srvs/KinematicMotionPlan.h>

static const double L1_JOINT_DIFF_MAX = .01;
static const double L1_GRIP_DIFF_MAX = .01;

class IssueKinematicPlan : public ros::node {

public:

  IssueKinematicPlan(void) : ros::node("issue_kinematic_plan") {

    left_arm_executing_move_ = false;
    left_arm_move_states_ = 0;
    left_arm_state_sent_ = 0;

    right_arm_executing_move_ = false;
    right_arm_move_states_ = 0;
    right_arm_state_sent_ = 0;

    //for commanding the arm joints
    advertise<std_msgs::PR2Arm>("cmd_leftarmconfig");
    advertise<std_msgs::PR2Arm>("cmd_rightarmconfig");
    
    //for returning goal state information
    advertise<pr2_msgs::MoveArmState>("left_movearmstate");
    advertise<pr2_msgs::MoveArmState>("right_movearmstate");

    //for gettting current arm information
    subscribe("left_pr2arm_pos",  left_arm_pos_,  &IssueKinematicPlan::currentLeftArmPos);
    subscribe("right_pr2arm_pos", right_arm_pos_, &IssueKinematicPlan::currentRightArmPos);
    
    //for getting arm goals
    subscribe("right_movearmgoal", right_arm_goal_, &IssueKinematicPlan::moveRightArm);	
    subscribe("left_movearmgoal", left_arm_goal_, &IssueKinematicPlan::moveLeftArm);
  }

private:

  void currentLeftArmPos(void) {

    if(left_arm_executing_move_) {

      //l1 norm for now

      double sum_joint_diff = 0.0;
      sum_joint_diff += fabs(left_arm_pos_.turretAngle-left_arm_command_.turretAngle);
      sum_joint_diff += fabs(left_arm_pos_.shoulderLiftAngle-left_arm_command_.shoulderLiftAngle);
      sum_joint_diff += fabs(left_arm_pos_.upperarmRollAngle-left_arm_command_.upperarmRollAngle);
      sum_joint_diff += fabs(left_arm_pos_.elbowAngle-left_arm_command_.elbowAngle);
      sum_joint_diff += fabs(left_arm_pos_.forearmRollAngle-left_arm_command_.forearmRollAngle);
      sum_joint_diff += fabs(left_arm_pos_.wristPitchAngle-left_arm_command_.wristPitchAngle);
      sum_joint_diff += fabs(left_arm_pos_.wristRollAngle-left_arm_command_.wristRollAngle);

      double gap_diff = fabs(left_arm_pos_.gripperForceCmd);//-left_arm_command_.gripperGapCmd);

      if(L1_JOINT_DIFF_MAX < sum_joint_diff && L1_GRIP_DIFF_MAX < gap_diff) {
	left_arm_state_sent_++;
	if(left_arm_state_sent_ == left_arm_move_states_) {
	  //we're done
	  std::cout << "Successfully completed left arm path.\n";
	  left_arm_executing_move_ = false;
	  left_arm_move_states_ = 0;
	  left_arm_state_sent_ = 0;
          pr2_msgs::MoveArmState move_arm_state;
          move_arm_state.set_desired_arm_state_size(left_arm_goal_.get_move_arm_goal_size());
          for(size_t t = 0; t < left_arm_goal_.get_move_arm_goal_size(); t++)
          {
            move_arm_state.desired_arm_state[t] = left_arm_goal_.move_arm_goal[t];
          }
          move_arm_state.set_actual_arm_state_size(9);
          move_arm_state.actual_arm_state[0] = left_arm_pos_.turretAngle;
          move_arm_state.actual_arm_state[1] = left_arm_pos_.shoulderLiftAngle;
          move_arm_state.actual_arm_state[2] = left_arm_pos_.upperarmRollAngle;
          move_arm_state.actual_arm_state[3] = left_arm_pos_.elbowAngle;
          move_arm_state.actual_arm_state[4] = left_arm_pos_.forearmRollAngle;
          move_arm_state.actual_arm_state[5] = left_arm_pos_.wristPitchAngle;
          move_arm_state.actual_arm_state[6] = left_arm_pos_.wristRollAngle;
          move_arm_state.actual_arm_state[7] = left_arm_pos_.gripperForceCmd;
          move_arm_state.actual_arm_state[8] = left_arm_pos_.gripperGapCmd;
          move_arm_state.active = 0;
          move_arm_state.valid = 1;
          move_arm_state.success = 1;  
	  return;
	} else {
	  setStateGoalFromPlan(left_arm_state_sent_, left_plan_res_, left_arm_command_);
	  publish("cmd_leftarmconfig", left_arm_command_);
	}
      }
      //otherwise we just chill out
    }

  }
  
  void currentRightArmPos(void) {
    if(right_arm_executing_move_) {
      
      //std::cout << "Right " << _right_arm_executing_move_ << std::endl;
      
      //l1 norm for now
      
      //std::cout << "Right arm "
      //         << right_arm_pos_.turretAngle << " versus " << right_arm_command_.turretAngle << std::endl;
 
      double sum_joint_diff = 0.0;
      sum_joint_diff += fabs(right_arm_pos_.turretAngle-right_arm_command_.turretAngle);
      sum_joint_diff += fabs(right_arm_pos_.shoulderLiftAngle-right_arm_command_.shoulderLiftAngle);
      sum_joint_diff += fabs(right_arm_pos_.upperarmRollAngle-right_arm_command_.upperarmRollAngle);
      sum_joint_diff += fabs(right_arm_pos_.elbowAngle-right_arm_command_.elbowAngle);
      sum_joint_diff += fabs(right_arm_pos_.forearmRollAngle-right_arm_command_.forearmRollAngle);
      sum_joint_diff += fabs(right_arm_pos_.wristPitchAngle-right_arm_command_.wristPitchAngle);
      sum_joint_diff += fabs(right_arm_pos_.wristRollAngle-right_arm_command_.wristRollAngle);
      
      //std::cout << "Sum joint diff " << sum_joint_diff << std::endl;
      
      double gap_diff = fabs(right_arm_pos_.gripperForceCmd);//-right_arm_command_.gripperGapCmd);
      
      //std::cout << "Gap diff " << gap_diff << std::endl;

      if(L1_JOINT_DIFF_MAX > sum_joint_diff && L1_GRIP_DIFF_MAX > gap_diff) {
	right_arm_state_sent_++;
	if(right_arm_state_sent_ == right_arm_move_states_) {
	  //we're done
	  std::cout << "Successfully completed right arm path.\n";
	  right_arm_executing_move_ = false;
	  right_arm_move_states_ = 0;
	  right_arm_state_sent_ = 0;
          pr2_msgs::MoveArmState move_arm_state;
          move_arm_state.set_desired_arm_state_size(right_arm_goal_.get_move_arm_goal_size());
          for(size_t t = 0; t < right_arm_goal_.get_move_arm_goal_size(); t++)
          {
            move_arm_state.desired_arm_state[t] = right_arm_goal_.move_arm_goal[t];
          }
          move_arm_state.set_actual_arm_state_size(9);
          move_arm_state.actual_arm_state[0] = right_arm_pos_.turretAngle;
          move_arm_state.actual_arm_state[1] = right_arm_pos_.shoulderLiftAngle;
          move_arm_state.actual_arm_state[2] = right_arm_pos_.upperarmRollAngle;
          move_arm_state.actual_arm_state[3] = right_arm_pos_.elbowAngle;
          move_arm_state.actual_arm_state[4] = right_arm_pos_.forearmRollAngle;
          move_arm_state.actual_arm_state[5] = right_arm_pos_.wristPitchAngle;
          move_arm_state.actual_arm_state[6] = right_arm_pos_.wristRollAngle;
          move_arm_state.actual_arm_state[7] = right_arm_pos_.gripperForceCmd;
          move_arm_state.actual_arm_state[8] = right_arm_pos_.gripperGapCmd;
          move_arm_state.active = 0;
          move_arm_state.valid = 1;
          move_arm_state.success = 1;  
          publish("right_movearmstate", move_arm_state);
	  return;
	} else {
	  setStateGoalFromPlan(right_arm_state_sent_, right_plan_res_, right_arm_command_);
	  publish("cmd_rightarmconfig", right_arm_command_);
	}
      }
      //otherwise we just chill out
    }
  }
  
  void moveLeftArm() {
    
    robot_srvs::KinematicMotionPlan::request req;
    
    req.model_id = "pr2::leftArm";
    req.threshold = 10e-06;
    req.start_state.set_vals_size(45);
    
    //initializing full value state
    for (unsigned int i = 0 ; i < req.start_state.vals_size ; ++i) {
      req.start_state.vals[i] = 0.0;
    }

    req.start_state.vals[22] = left_arm_pos_.turretAngle;
    req.start_state.vals[23] = left_arm_pos_.shoulderLiftAngle;
    req.start_state.vals[24] = left_arm_pos_.upperarmRollAngle;
    req.start_state.vals[25] = left_arm_pos_.elbowAngle;
    req.start_state.vals[26] = left_arm_pos_.forearmRollAngle;
    req.start_state.vals[27] = left_arm_pos_.wristPitchAngle;
    req.start_state.vals[28] = left_arm_pos_.wristRollAngle;
    //req.start_state.vals[25] = left_arm_pos_.gripperForceCmd;
    //req.start_state.vals[26] = left_arm_pos_.gripperGapCmd;
    
    //filling out goal state
    req.goal_state.set_vals_size(7);
    req.goal_state.vals[0] = left_arm_goal_.move_arm_goal[0];
    req.goal_state.vals[1] = left_arm_goal_.move_arm_goal[1];
    req.goal_state.vals[2] = left_arm_goal_.move_arm_goal[2];
    req.goal_state.vals[3] = left_arm_goal_.move_arm_goal[3];
    req.goal_state.vals[4] = left_arm_goal_.move_arm_goal[4];
    req.goal_state.vals[5] = left_arm_goal_.move_arm_goal[5];
    req.goal_state.vals[6] = left_arm_goal_.move_arm_goal[6];
  
    req.allowed_time = 2.0;
    
    req.volumeMin.x = -1.0; req.volumeMin.y = -1.0; req.volumeMin.z = -1.0;
    req.volumeMax.x = -1.0; req.volumeMax.y = -1.0; req.volumeMax.z = -1.0;
   
    pr2_msgs::MoveArmState move_arm_state;
    move_arm_state.set_desired_arm_state_size(left_arm_goal_.get_move_arm_goal_size());
    for(size_t t = 0; t < left_arm_goal_.get_move_arm_goal_size(); t++)
    {
      move_arm_state.desired_arm_state[t] = left_arm_goal_.move_arm_goal[t];
    }
    move_arm_state.set_actual_arm_state_size(9);
    move_arm_state.actual_arm_state[0] = left_arm_pos_.turretAngle;
    move_arm_state.actual_arm_state[1] = left_arm_pos_.shoulderLiftAngle;
    move_arm_state.actual_arm_state[2] = left_arm_pos_.upperarmRollAngle;
    move_arm_state.actual_arm_state[3] = left_arm_pos_.elbowAngle;
    move_arm_state.actual_arm_state[4] = left_arm_pos_.forearmRollAngle;
    move_arm_state.actual_arm_state[5] = left_arm_pos_.wristPitchAngle;
    move_arm_state.actual_arm_state[6] = left_arm_pos_.wristRollAngle;
    move_arm_state.actual_arm_state[7] = left_arm_pos_.gripperForceCmd;
    move_arm_state.actual_arm_state[8] = left_arm_pos_.gripperGapCmd;

    unsigned int nstates = 0;
    if (ros::service::call("plan_kinematic_path", req, left_plan_res_)) {
      nstates = left_plan_res_.path.get_states_size();
      printf("Obtained solution path with %u states\n", nstates);
    } else {
      fprintf(stderr, "Service 'plan_kinematic_path' failed\n");
      move_arm_state.active = 0;
      move_arm_state.valid = 0;
      move_arm_state.success = 0;  	
    }

    //now we have to execute the plan

    if(nstates > 0) {
      left_arm_executing_move_ = true;
      left_arm_move_states_ = nstates;
      left_arm_state_sent_ = 0;
      setStateGoalFromPlan(0, left_plan_res_, left_arm_command_);
      //setting gripper stuff for all commands
      left_arm_command_.gripperForceCmd = left_arm_goal_.move_arm_goal[7];
      left_arm_command_.gripperGapCmd = left_arm_goal_.move_arm_goal[8];
      publish("cmd_leftarmconfig", left_arm_command_);
      move_arm_state.active = 1;
      move_arm_state.valid = 1;
      move_arm_state.success = 0;  
    } else {
      //no states would seem to mean failure as well
      move_arm_state.active = 0;
      move_arm_state.valid = 0;
      move_arm_state.success = 0;
    }  
    publish("left_movearmstate", move_arm_state);
  }
  
  void moveRightArm() {
    
    robot_srvs::KinematicMotionPlan::request req;
    
    req.model_id = "pr2::rightArm";
    req.threshold = 10e-04;
    req.start_state.set_vals_size(45);
    
    //initializing full value state
    for (unsigned int i = 0 ; i < req.start_state.vals_size ; ++i) {
      req.start_state.vals[i] = 0.0;
    }

    req.start_state.vals[33] = right_arm_pos_.turretAngle;
    req.start_state.vals[34] = right_arm_pos_.shoulderLiftAngle;
    req.start_state.vals[35] = right_arm_pos_.upperarmRollAngle;
    req.start_state.vals[36] = right_arm_pos_.elbowAngle;
    req.start_state.vals[37] = right_arm_pos_.forearmRollAngle;
    req.start_state.vals[38] = right_arm_pos_.wristPitchAngle;
    req.start_state.vals[39] = right_arm_pos_.wristRollAngle;
    //req.start_state.vals[25] = right_arm_pos_.gripperForceCmd;
    //req.start_state.vals[26] = right_arm_pos_.gripperGapCmd;
    
    //filling out goal state
    req.goal_state.set_vals_size(7);
    req.goal_state.vals[0] = right_arm_goal_.move_arm_goal[0];
    req.goal_state.vals[1] = right_arm_goal_.move_arm_goal[1];
    req.goal_state.vals[2] = right_arm_goal_.move_arm_goal[2];
    req.goal_state.vals[3] = right_arm_goal_.move_arm_goal[3];
    req.goal_state.vals[4] = right_arm_goal_.move_arm_goal[4];
    req.goal_state.vals[5] = right_arm_goal_.move_arm_goal[5];
    req.goal_state.vals[6] = right_arm_goal_.move_arm_goal[6];
    
//     std::cout << "Right goal "
//               << req.goal_state.vals[0] << " "
//               << req.goal_state.vals[1] << " "
//               << req.goal_state.vals[2] << " "
//               << req.goal_state.vals[3] << " "
//               << req.goal_state.vals[4] << " "
//               << req.goal_state.vals[5] << " "
//               << req.goal_state.vals[6] << std::endl;

    req.allowed_time = 2.0;
    
    req.volumeMin.x = -1.0; req.volumeMin.y = -1.0; req.volumeMin.z = -1.0;
    req.volumeMax.x = -1.0; req.volumeMax.y = -1.0; req.volumeMax.z = -1.0;
   
    unsigned int nstates = 0;
    pr2_msgs::MoveArmState move_arm_state;
    move_arm_state.set_desired_arm_state_size(right_arm_goal_.get_move_arm_goal_size());
    for(size_t t = 0; t < right_arm_goal_.get_move_arm_goal_size(); t++)
    {
      move_arm_state.desired_arm_state[t] = right_arm_goal_.move_arm_goal[t];
    }
    move_arm_state.set_actual_arm_state_size(9);
    move_arm_state.actual_arm_state[0] = right_arm_pos_.turretAngle;
    move_arm_state.actual_arm_state[1] = right_arm_pos_.shoulderLiftAngle;
    move_arm_state.actual_arm_state[2] = right_arm_pos_.upperarmRollAngle;
    move_arm_state.actual_arm_state[3] = right_arm_pos_.elbowAngle;
    move_arm_state.actual_arm_state[4] = right_arm_pos_.forearmRollAngle;
    move_arm_state.actual_arm_state[5] = right_arm_pos_.wristPitchAngle;
    move_arm_state.actual_arm_state[6] = right_arm_pos_.wristRollAngle;
    move_arm_state.actual_arm_state[7] = right_arm_pos_.gripperForceCmd;
    move_arm_state.actual_arm_state[8] = right_arm_pos_.gripperGapCmd;
    if (ros::service::call("plan_kinematic_path", req, right_plan_res_)) {
      nstates = right_plan_res_.path.get_states_size();
      printf("Obtained solution path with %u states\n", nstates);
      std::cout << "Distance is " << right_plan_res_.distance << std::endl;
    } else {
      fprintf(stderr, "Service 'plan_kinematic_path' failed\n");
      move_arm_state.active = 0;
      move_arm_state.valid = 0;
      move_arm_state.success = 0;  
    }

    //now we have to execute the plan

    if(nstates > 0) {
      //std::cout << "Number of vals in zero-ith state " << right_plan_res_.path.states[0].get_vals_size() << std::endl;
      right_arm_executing_move_ = true;
      right_arm_move_states_ = nstates;
      right_arm_state_sent_ = 0;
      setStateGoalFromPlan(0, right_plan_res_, right_arm_command_);
      //setting gripper stuff for all commands
      right_arm_command_.gripperForceCmd = right_arm_goal_.move_arm_goal[7];
      right_arm_command_.gripperGapCmd = right_arm_goal_.move_arm_goal[8];
      publish("cmd_rightarmconfig", right_arm_command_);
      move_arm_state.active = 1;
      move_arm_state.valid = 1;
      move_arm_state.success = 0;  
    } else {
      //no states would seem to mean failure as well
      move_arm_state.active = 0;
      move_arm_state.valid = 0;
      move_arm_state.success = 0;  
    }
    publish("right_movearmstate", move_arm_state);
  }

private:

  void setStateGoalFromPlan(unsigned int state_num, const robot_srvs::KinematicMotionPlan::response& res,
			    std_msgs::PR2Arm& arm_com) {
    if(state_num >= res.path.get_states_size()) {
      printf("SetStateGoalFromPlan:: trying to set state greater than number of states in path.\n");
      return;
    }
    arm_com.turretAngle = res.path.states[state_num].vals[0];
    arm_com.shoulderLiftAngle = res.path.states[state_num].vals[1];
    arm_com.upperarmRollAngle = res.path.states[state_num].vals[2];
    arm_com.elbowAngle = res.path.states[state_num].vals[3];
    arm_com.forearmRollAngle = res.path.states[state_num].vals[4];
    arm_com.wristPitchAngle = res.path.states[state_num].vals[5];
    arm_com.wristRollAngle = res.path.states[state_num].vals[6];  
  
    std::cout << "Setting state " << state_num << " " 
              << res.path.states[state_num].vals[0] << " "
              << res.path.states[state_num].vals[1] << " "
              << res.path.states[state_num].vals[2] << " "
              << res.path.states[state_num].vals[3] << " "
              << res.path.states[state_num].vals[4] << " "
              << res.path.states[state_num].vals[5] << " "
              << res.path.states[state_num].vals[6] << std::endl;
  }

private:

  robot_srvs::KinematicMotionPlan::response left_plan_res_;
  robot_srvs::KinematicMotionPlan::response right_plan_res_;
  
  std_msgs::PR2Arm left_arm_command_;
  std_msgs::PR2Arm right_arm_command_;
  
  std_msgs::PR2Arm left_arm_pos_;
  std_msgs::PR2Arm right_arm_pos_;

  pr2_msgs::MoveArmGoal left_arm_goal_;
  pr2_msgs::MoveArmGoal right_arm_goal_;

  bool left_arm_executing_move_;
  bool right_arm_executing_move_;

  unsigned int left_arm_move_states_;
  unsigned int right_arm_move_states_;

  unsigned int left_arm_state_sent_;
  unsigned int right_arm_state_sent_;

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
