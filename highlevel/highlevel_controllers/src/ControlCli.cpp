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

// Author Tony Pratkanis (pratkanis@willowgarage.com)


#include <ros/node.h>
#include <pr2_msgs/MoveArmGoal.h>
#include <mechanism_control/MechanismState.h>

using namespace std;

class ControlCLI : public ros::node {
public:
  ControlCLI() : ros::node("CommandLineInterface") {
    dead = false;
    advertise<pr2_msgs::MoveArmGoal>("left_arm_goal", 1);
    advertise<pr2_msgs::MoveArmGoal>("right_arm_goal", 1);
    advertise<mechanism_control::MechanismState>("mechanism_state", 1);
    runCLI();
  }
  bool alive() { return !dead; }
private:
  bool dead;


  void fillNamesLeftArm(std::vector<std::string> &names) {
    names.push_back("shoulder_pan_left_joint");
    names.push_back("shoulder_pitch_left_joint");
    names.push_back("upperarm_roll_left_joint");
    names.push_back("elbow_flex_left_joint");
    names.push_back("forearm_roll_left_joint");
    names.push_back("wrist_flex_left_joint");
    names.push_back("gripper_roll_left_joint"); 
  }
  
  void fillNamesRightArm(std::vector<std::string> &names) {
    names.push_back("shoulder_pan_right_joint");
    names.push_back("shoulder_pitch_right_joint");
    names.push_back("upperarm_roll_right_joint");
    names.push_back("elbow_flex_right_joint");
    names.push_back("forearm_roll_right_joint");
    names.push_back("wrist_flex_right_joint");
    names.push_back("gripper_roll_right_joint");
  }

  void runCLI() {
    printf("Type:\nI\tInitialize States\nQ\tQuit\nS\tHand Wave\nL\tLeft Arm\nR\tRight Arm\n");
    char c = '\n';

    while (c == '\n' || c == '\r') {
      c = getchar();
    }

    if (c == 'I') {
      mechanism_control::MechanismState mechanismState;
      std::vector<std::string> names;
      
      fillNamesLeftArm(names);
      fillNamesRightArm(names);

      mechanismState.set_joint_states_size(names.size());
      for (unsigned int i = 0; i < names.size(); i++) {
        mechanismState.joint_states[i].position = 0;
        mechanismState.joint_states[i].name = names[i];
      }
      printf("Publishing states.\n");
      publish<mechanism_control::MechanismState>("mechanism_state", mechanismState);
      
    } else if (c == 'S') {
      std::vector<std::string> names;
      
      fillNamesRightArm(names);

      pr2_msgs::MoveArmGoal goalMsg;
      goalMsg.enable = 1;
      goalMsg.set_configuration_size(names.size());

      for (unsigned int i = 0; i < names.size(); i++) {
	goalMsg.configuration[i].name = names[i];
	goalMsg.configuration[i].position = 0;
      }

      goalMsg.configuration[0].position = -1;


      printf("Publishing:");
      for (unsigned int i = 0; i < names.size(); i++) {
        printf(" %f", goalMsg.configuration[i].position);
      }
      printf("\n");

      publish<pr2_msgs::MoveArmGoal>("right_arm_goal", goalMsg);

    } else if (c == 'L' || c == 'R') {
      printf("%s Arm\n", c == 'L' ? "Left" : "Right");

      std::vector<std::string> names;
      
      if (c == 'L') {
        fillNamesLeftArm(names);
      } else {
        fillNamesRightArm(names);
      }

      pr2_msgs::MoveArmGoal goalMsg;
      goalMsg.enable = 1;
      goalMsg.set_configuration_size(names.size());

      for (unsigned int i = 0; i < names.size(); i++) {
        printf("Enter the angle for joint %s in radians:\n",
               names[i].c_str());
        float value = -1000000;
        char buff[256];
      RETRY:
        memset(buff, 0, 256);
        value = -1000000;
        fgets(buff, 256, stdin);
        if (buff[0] == '\n') { goto RETRY; }
        sscanf(buff, "%f", &value);
        if (value < -M_PI * 2 || value > M_PI * 2) {
          printf("Please enter a value in the range [-2PI, 2PI]\n");
          goto RETRY;
        }
	goalMsg.configuration[i].name = names[i];
	goalMsg.configuration[i].position = value;
        
      }
      printf("Publishing:");
      for (unsigned int i = 0; i < names.size(); i++) {
        printf(" %f", goalMsg.configuration[i].position);
      }
      printf("\n");

      publish<pr2_msgs::MoveArmGoal>(c == 'L' ? "left_arm_goal" : "right_arm_goal", goalMsg);
      
    } else if (c == 'Q') {
      dead = true;
      return;
    } else {
      printf("Rejected command.\n");
    }
    runCLI();
  }
};




int
main(int argc, char** argv)
{

  ros::init(argc,argv);
  
  ControlCLI node;
  while (node.ok() && node.alive()) {
    usleep(100);
  }
  ros::fini();

  return(0);
}










