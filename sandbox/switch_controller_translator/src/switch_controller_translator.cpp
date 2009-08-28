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

#include "ros/ros.h"

#include <std_msgs/Empty.h>
#include <pr2_robot_actions/SwitchControllers.h>
#include <pr2_robot_actions/SwitchControllersState.h>

#include <pr2_mechanism_msgs/SwitchControllerAction.h>
#include <actionlib/client/action_client.h>
#include <action_translator/action_translator.h>
#include <robot_actions/action_runner.h>
#include <boost/thread.hpp>


pr2_mechanism_msgs::SwitchControllerGoal fromOldGoal(const pr2_robot_actions::SwitchControllers& old_goal)
{
  pr2_mechanism_msgs::SwitchControllerGoal new_goal;
  new_goal.start_controllers = old_goal.start_controllers;
  new_goal.stop_controllers = old_goal.stop_controllers;
  return new_goal;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_action_translator");

  ros::NodeHandle n;
  action_translator::ActionTranslator<pr2_mechanism_msgs::SwitchControllerAction, pr2_robot_actions::SwitchControllers, std_msgs::Empty>
    translator("switch_controller", "switch_controllers", &fromOldGoal, NULL, NULL);

  robot_actions::ActionRunner runner(10.0);
  runner.connect<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState, std_msgs::Empty>(translator);

  runner.run();

  ros::MultiThreadedSpinner m_spinner(2);
  m_spinner.spin();

  return 0;
}
