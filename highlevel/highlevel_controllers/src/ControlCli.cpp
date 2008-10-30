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
#include <robot_msgs/MechanismState.h>
#include <highlevel_controllers/RechargeGoal.h>
#include <robot_srvs/PlanNames.h>
#include <robot_msgs/BatteryState.h>
#include <rosthread/member_thread.h>


using namespace std;

class ControlCLI : public ros::node {
public:
  ControlCLI() : ros::node("CommandLineInterface") {
    dead = false;
    advertise<pr2_msgs::MoveArmGoal>("left_arm_goal", 1);
    advertise<pr2_msgs::MoveArmGoal>("right_arm_goal", 1);
    advertise<pr2_msgs::MoveArmGoal>("right_arm_goal", 1);
    advertise<highlevel_controllers::RechargeGoal>("recharge_goal", 1);
    advertise<robot_msgs::BatteryState>("bogus_battery_state", 1);
    advertise<robot_msgs::MechanismState>("mechanism_state", 1);

    // Set default values such that the robot is connected and fully charged
    batteryState_.power_consumption = -800;
    batteryState_.energy_remaining = 1000;
    batteryState_.energy_capacity = 1000;

    // Start a thread to publish batteryState at 10 Hz
    ros::thread::member_thread::startMemberFunctionThread(this, &ControlCLI::publishingLoop);  

    // Start the console
    runCLI();
  }
  bool alive() { return !dead; }
private:
  bool dead;

  void publishingLoop(){
    while (true) {
      publish<robot_msgs::BatteryState>("bogus_battery_state", batteryState_);
      usleep(100000);
    }
  }

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

  float enterValue(float min, float max) {
    char buff[256];
    while(1) {
      memset(buff, 0, 256);
      float value = min - 1;
      fgets(buff, 256, stdin);
      if (buff[0] != '\n') {
	if (sscanf(buff, "%f", &value) && value < min || value > max) {
	  printf("Please enter a value in the range [%f, %f]\n", min, max);
	} else {
	  return value;
	}
      }
    }
    return 0;
  }

  void runCLI() {
    printf("Type:\nI\tInitialize States\nP\tActivate Recharge.\nB\tSet Battery State\nN\tPrint Joint Names\nQ\tQuit\nS\tHand Wave\nL\tLeft Arm\nR\tRight Arm\n");
    char c = '\n';

    while (c == '\n' || c == '\r') {
      c = getchar();
    }

    if (c == 'B') {
      double consumption(0.0), remaining(0.0), capacity(0.0);
      printf("Enter Power Consumption:\n");
      consumption = enterValue(-10000, 10000);
      printf("Enter Remaining Energy:\n");
      remaining = enterValue(-10000, 10000);
      printf("Enter Capacity:\n");
      capacity = enterValue(0, 10000);
      printf("Setting battery state to:<%f, %f, %f>.\n", consumption, remaining, capacity);
      batteryState_.lock();
      batteryState_.power_consumption = consumption;
      batteryState_.energy_remaining = remaining;
      batteryState_.energy_capacity = capacity;
      batteryState_.unlock();
    } else if (c == 'P') {
      double recharge_level(0.0);
      printf("Enter recharge level as a percentage of maximum capacity:\n");
      recharge_level = enterValue(0, 1);
      recharge_level = std::max(0.0, std::min(recharge_level, 1.0));
      printf("Sending recharge goal:%f\n", recharge_level);
      highlevel_controllers::RechargeGoal goal;
      goal.enable = 1;
      goal.recharge_level = recharge_level;
      publish<highlevel_controllers::RechargeGoal>("recharge_goal", goal);
    } else if (c == 'N') {
      printf("Joint names (number of parameters):\n");
      
      robot_srvs::PlanNames::request namesReq;
      robot_srvs::PlanNames::response names;
      if (ros::service::call("plan_joint_state_names", namesReq, names)) {
        for (unsigned int i = 0; i < names.get_names_size(); i++) {
          printf("%s: %d\n", names.names[i].c_str(), names.num_values[i]);
        }
      } else {
        printf("Error in service call.\n");
      }

      
    } else if (c == 'I') {
      robot_msgs::MechanismState mechanismState;
      std::vector<std::string> names;
      
      fillNamesLeftArm(names);
      fillNamesRightArm(names);

      mechanismState.set_joint_states_size(names.size());
      for (unsigned int i = 0; i < names.size(); i++) {
        mechanismState.joint_states[i].position = 0;
        mechanismState.joint_states[i].name = names[i];
      }
      printf("Publishing states.\n");
      publish<robot_msgs::MechanismState>("mechanism_state", mechanismState);
      
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
	goalMsg.configuration[i].name = names[i];
	goalMsg.configuration[i].position = enterValue(-2 * M_PI, 2 * M_PI);
        
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

  robot_msgs::BatteryState batteryState_;
  robot_msgs::BatteryState batteryMsg_;;
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










