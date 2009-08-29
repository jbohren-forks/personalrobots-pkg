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
 *   * Neither the name of Willow Garage nor the names of its
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


#include "pr2_mechanism_control/action_mechanism_control.h"


using namespace ros;
using namespace std;
using namespace pr2_mechanism_msgs;


namespace mechanism_control{

ActionMechanismControl::ActionMechanismControl():
  action_server_(node_, "switch_controller", boost::bind(&ActionMechanismControl::execute, this, _1))
{
  switch_controller_srv_ = node_.serviceClient<pr2_mechanism_msgs::SwitchController>("switch_controller");
};


ActionMechanismControl::~ActionMechanismControl(){};


void ActionMechanismControl::execute(const SwitchControllerGoalConstPtr& switch_goal)
{
  ROS_DEBUG("ActionMechanismControl: received request to start %i controllers and stop %i controllers",
           switch_goal->start_controllers.size(), switch_goal->stop_controllers.size());
  for (unsigned int i=0; i<switch_goal->start_controllers.size(); i++)
    ROS_DEBUG("ActionMechanismControl:   - starting controller %s", switch_goal->start_controllers[i].c_str());
  for (unsigned int i=0; i<switch_goal->stop_controllers.size(); i++)
    ROS_DEBUG("ActionMechanismControl:   - stopping controller %s", switch_goal->stop_controllers[i].c_str());
  pr2_mechanism_msgs::SwitchController::Request req;
  pr2_mechanism_msgs::SwitchController::Response resp;
  req.start_controllers = switch_goal->start_controllers;
  req.stop_controllers  = switch_goal->stop_controllers;
  if (!switch_controller_srv_.call(req, resp)){
    ROS_ERROR("ActionMechanismControl: failed to communicate with mechanism control to switch controllers");
    action_server_.setAborted();
  }
  else{
    if (resp.ok){
      ROS_DEBUG("ActionMechanismControl: controlers switched succesfully");
      action_server_.setSucceeded();
    }
    else{
      ROS_ERROR("ActionMechanismControl: failed to switch controllers");
      action_server_.setAborted();
    }
  }
}

}



int main(int argc, char** argv)
{
  ros::init(argc,argv,"pr2_mechanism_control_action_container");

  mechanism_control::ActionMechanismControl act;

  ros::spin();
  return 0;
}
