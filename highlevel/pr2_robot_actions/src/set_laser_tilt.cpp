/*********************************************************************
*
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <pr2_msgs/LaserTrajCmd.h>
#include <pr2_srvs/SetLaserTrajCmd.h>
#include <pr2_robot_actions/SetLaserTiltState.h>
#include <std_msgs/Empty.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <string>
#include <ros/ros.h>

#include <pr2_robot_actions/set_hokuyo_mode.h>

namespace pr2_robot_actions {
  class SetLaserTilt : public robot_actions::Action<std_msgs::Empty, std_msgs::Empty> {
  public:
    SetLaserTilt(std::string laser_controller): 
      robot_actions::Action<std_msgs::Empty, std_msgs::Empty>("set_laser_tilt"),
      laser_controller_(laser_controller){};

    robot_actions::ResultStatus execute(const std_msgs::Empty& empty, std_msgs::Empty& feedback){
      //first... set the hokuyo node to the desired frequency for navigation
      pr2_robot_actions::setHokuyoMode("tilt_hokuyo_node", "navigate");

      pr2_srvs::SetLaserTrajCmd::Request req_laser;
      pr2_srvs::SetLaserTrajCmd::Response res_laser;
      req_laser.command.profile = "linear";
      req_laser.command.max_rate = 5;
      req_laser.command.max_accel = 5;
      req_laser.command.pos.push_back(1.0);      req_laser.command.pos.push_back(-0.7);      req_laser.command.pos.push_back(1.0);
      req_laser.command.time.push_back(0.0);     req_laser.command.time.push_back(1.8);      req_laser.command.time.push_back(2.025);
      if(!ros::service::call(laser_controller_ + "/set_traj_cmd", req_laser, res_laser)){
        ROS_ERROR("Failed to start laser.");
        return robot_actions::ABORTED;
      }

      return robot_actions::SUCCESS;

    }
    
  private:
    std::string laser_controller_;
  };
  
};

int main(int argc, char** argv){

  ros::init(argc, argv);
  ros::Node node("set_tilt_controller_action");


  pr2_robot_actions::SetLaserTilt setter("laser_tilt_controller");

  robot_actions::ActionRunner runner(20.0);
  runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(setter);
  runner.run();

  ros::spin();

  return(0);
}

