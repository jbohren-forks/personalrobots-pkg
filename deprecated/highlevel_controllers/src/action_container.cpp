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

// @author Conor McGann

#include <robot_actions/action_runner.h>
#include <robot_actions/action.h>
#include <robot_actions/ShellCommandState.h>
#include <std_msgs/String.h>
#include <cstdlib>

/**
 * @note This file is temporary. Normally we would not declare and define all these actions together. Just
 * working with it until we converge on action design model.
 */
namespace highlevel_controllers {

  class ShellCommand: public robot_actions::Action<std_msgs::String, std_msgs::String> {
  public:

    ShellCommand(): robot_actions::Action<std_msgs::String, std_msgs::String>("shell_command") {}

  private:

    // Activation does all the real work
    virtual robot_actions::ResultStatus execute(const std_msgs::String& command_str, std_msgs::String& feedback_str){
      ROS_DEBUG("Trying to execute %s.", command_str.data.c_str());

      // Will now block the goal callback thread, which is OK
      int result = system(command_str.data.c_str());
      if( result == 0 ){
	feedback_str = command_str;
	return robot_actions::SUCCESS;
      }


      std::stringstream feedback;
      feedback << "Aborted " << command_str.data << " with return code: " << result;
      ROS_DEBUG("Failed to execute %s. Result Code = %d", command_str.data.c_str(), result);
      std_msgs::String response;
      feedback_str.data = feedback.str();
      return robot_actions::ABORTED;
    }
  };
}

int main(int argc, char** argv){ 
  ros::init(argc, argv);
  ros::Node node("highlevel_controller/action_container");

  // Allocate an action runnwe with an update rate of 10 Hz
  robot_actions::ActionRunner runner(10.0);

  highlevel_controllers::ShellCommand shell_command;
  runner.connect<std_msgs::String, robot_actions::ShellCommandState, std_msgs::String>(shell_command);

  runner.run();

  node.spin();

  return 0;
}

