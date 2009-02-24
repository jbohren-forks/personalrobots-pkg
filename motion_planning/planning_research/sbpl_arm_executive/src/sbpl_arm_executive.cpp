/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *W
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sbpl_arm_executive/pr2_arm_node.h>

using namespace pr2_arm_node;

class ExecNode : public PR2ArmNode
{
  public:

    int state_;
    enum {INITIALIZED, GET_GOAL, PLAN, EXECUTE, DONE, MAX_STATES};
    ExecNode(std::string node_name, std::string arm_name, std::string gripper_name):PR2ArmNode(node_name,arm_name,gripper_name),state_(INITIALIZED){};

    robot_msgs::JointTrajPoint current_joint_positions_;
    robot_msgs::JointTraj planned_path_;
    std::vector<robot_msgs::Pose> goal_;

    void spin()
    {
      int goal_id_ = 1;
      double hp[7] = {0,0.2,0.0,-1.25,0,0,0};
      std::vector<double>home_position(hp,hp + sizeof(hp)/sizeof(*hp));
      goal_.resize(1);
 
      while (ok())
      {
	switch (state_)
        {	  
          case INITIALIZED:
          {
	    ROS_INFO("Initializing");
	    //            goHome(home_position);
            state_ = GET_GOAL;
            break;
          }
          case GET_GOAL:
          {
	    ROS_INFO("Get goal");
            if(goal_id_ == 1)
            {
              goal_[0] = RPYToTransform(0.0,0.0,0.0,0.2, -0.8, 0.8);
              ROS_INFO("goal: %1.2f %1.2f %1.2f", goal_[0].position.x,goal_[0].position.y,goal_[0].position.z);
              goal_id_ = 2;
            }
            else
            {
              goal_[0] = RPYToTransform(0.0,0.0,0.0,0.2, -0.8, 0.8);
              goal_id_ = 1;
            }
            state_ = PLAN;
            break;
          }
          case PLAN:
          {
	    ROS_INFO("Planning");
	    getCurrentPosition(current_joint_positions_);
            if(planSBPLPath(current_joint_positions_,goal_,planned_path_))
              state_ = EXECUTE;
            else
              ROS_INFO("Planning failed: Retry");
            break;
          }
          case EXECUTE:
          {
            if(sendTrajectory(arm_name_,planned_path_))
            {
              ROS_INFO("Executed trajectory successfully");
              state_ = GET_GOAL;
            }
            else
            {
              ROS_INFO("Trajectory execution unsuccessful");
              state_ = PLAN;
            }
            break;
          }
	}
// 	usleep(10000);
        sleep(2);
      }
    }
};

int main(int argc, char *argv[])
{
  ros::init(argc,argv); 

  ExecNode node("sbpl_executive","right_arm","right_gripper");

  try 
  {
    node.spin();
  }
  catch(char const* e)
  {
    std::cout << e << std::endl;
  }  
  return(0);
}
