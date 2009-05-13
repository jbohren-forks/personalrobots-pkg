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
#include <pr2_srvs/SetPeriodicCmd.h>
#include <pr2_msgs/PeriodicCmd.h>
#include <pr2_robot_actions/SetLaserTiltState.h>
#include <string>
#include <ros/ros.h>
namespace pr2_robot_actions {
  class SetLaserTilt : public robot_actions::Action<pr2_msgs::PeriodicCmd, pr2_msgs::PeriodicCmd> {
    public:
      SetLaserTilt(std::string laser_controller): robot_actions::Action<pr2_msgs::PeriodicCmd, pr2_msgs::PeriodicCmd>("set_laser_tilt"),
      laser_controller_(laser_controller){}

      robot_actions::ResultStatus execute(const pr2_msgs::PeriodicCmd& goal, pr2_msgs::PeriodicCmd& feedback){
        pr2_srvs::SetPeriodicCmd::Request req_laser;
        pr2_srvs::SetPeriodicCmd::Response res_laser;
        req_laser.command = goal;

        if(!ros::service::call(laser_controller_ + "/set_periodic_cmd", req_laser, res_laser)){
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
  ros::init(argc, argv, "set_laser_tilt");
  pr2_robot_actions::SetLaserTilt setter("laser_tilt_controller");
  robot_actions::ActionRunner runner(20.0);
  runner.connect<pr2_msgs::PeriodicCmd, pr2_robot_actions::SetLaserTiltState, pr2_msgs::PeriodicCmd>(setter);
  runner.run();

  ros::spin();

  return(0);
}

