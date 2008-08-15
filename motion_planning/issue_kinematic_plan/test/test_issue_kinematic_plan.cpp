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
#include <pr2_msgs/MoveArmState.h>
#include <pr2_msgs/MoveArmGoal.h>

class TestIssueKinematicPlan : public ros::node
{
public:

  TestIssueKinematicPlan(void) : ros::node("test_issue_kinematic_plan")
  {
    advertise<pr2_msgs::MoveArmGoal>("right_movearmgoal");
    advertise<pr2_msgs::MoveArmGoal>("left_movearmgoal");
    
    subscribe("right_movearmstate", right_move_arm_state_, &TestIssueKinematicPlan::recMoveRightArmState);
    subscribe("left_movearmstate", left_move_arm_state_, &TestIssueKinematicPlan::recMoveLeftArmState);
  }    
  
  void recMoveRightArmState(void) 
  {
    std::cout << "Got right arm state message " 
              << " active " << (right_move_arm_state_.active==1) << " "
              << " valid " << (right_move_arm_state_.valid==1) << " "
              << " success " << (right_move_arm_state_.success==1) << std::endl
              << " arm location " 
              << right_move_arm_state_.actual_arm_state[0] << " "
              << right_move_arm_state_.actual_arm_state[1] << " "
              << right_move_arm_state_.actual_arm_state[2] << " "
              << right_move_arm_state_.actual_arm_state[3] << " "
              << right_move_arm_state_.actual_arm_state[4] << " "
              << right_move_arm_state_.actual_arm_state[5] << " "
              << right_move_arm_state_.actual_arm_state[6] << " "
              << right_move_arm_state_.actual_arm_state[7] << " "
              << right_move_arm_state_.actual_arm_state[8] << " "
              << std::endl
              << " desired location "
              <<  right_move_arm_state_.desired_arm_state[0] << " "
              <<  right_move_arm_state_.desired_arm_state[1] << " "
              <<  right_move_arm_state_.desired_arm_state[2] << " "
              <<  right_move_arm_state_.desired_arm_state[3] << " "
              <<  right_move_arm_state_.desired_arm_state[4] << " "
              <<  right_move_arm_state_.desired_arm_state[5] << " "
              <<  right_move_arm_state_.desired_arm_state[6] << " "
              <<  right_move_arm_state_.desired_arm_state[7] << " "
              <<  right_move_arm_state_.desired_arm_state[8] << std::endl;
  }

  void recMoveLeftArmState(void) 
  {
  }
  pr2_msgs::MoveArmState right_move_arm_state_;
  pr2_msgs::MoveArmState left_move_arm_state_;

};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    TestIssueKinematicPlan test;

    sleep(3);

    //for(int i = 0; i < 5; i++) {
    pr2_msgs::MoveArmGoal arm_goal;
    arm_goal.set_move_arm_goal_size(9);
    arm_goal.move_arm_goal[0] = -1.0;
    arm_goal.move_arm_goal[1] = 0;
    arm_goal.move_arm_goal[2] = 0;
    arm_goal.move_arm_goal[3] = 0;
    arm_goal.move_arm_goal[4] = 0; 
    arm_goal.move_arm_goal[5] = 0;
    arm_goal.move_arm_goal[6] = 0;
    arm_goal.move_arm_goal[7] = 100;
    arm_goal.move_arm_goal[8] = .2;
    test.publish("right_movearmgoal",arm_goal);
    //sleep(1);
    //test.publish("left_pr2arm_set_position",arm_goal);
    test.spin();
        
    test.shutdown();
    
    return 0;    
}
