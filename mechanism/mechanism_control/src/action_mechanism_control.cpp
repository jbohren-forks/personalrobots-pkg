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

#include <ros/node.h>
#include <robot_srvs/SwitchController.h>
#include <mechanism_control/SwitchController.h>
#include <mechanism_control/SwitchControllerState.h>
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>


using namespace ros;
using namespace std;

class ActionMechanismControl: public robot_actions::Action<mechanism_control::SwitchController, mechanism_control::SwitchController>{

public: 
  ActionMechanismControl(Node& node): 
    robot_actions::Action<mechanism_control::SwitchController, mechanism_control::SwitchController>("door_detector")
  {};
  
  
  ~ActionMechanismControl(){};
  
  
  void handleActivate(const mechanism_control::SwitchController& c)
  {
    robot_srvs::SwitchController::Request req;
    robot_srvs::SwitchController::Response resp;
    req.start_controllers = c.start_controllers;
    req.stop_controllers  = c.stop_controllers;
    if (!ros::service::call("switch_controller", req, resp)){
      ROS_ERROR("ActionMechanismControl: failed to switch controllers");
      notifyAborted(c);
    }
    else
      notifySucceeded(c);
  }
  
  
  // cannot be preempted
  void handlePreempt()
  {};

};// class









// ----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv);

  ros::Node node("name");
  ActionMechanismControl act(node);

  robot_actions::ActionRunner runner(1.0);
  runner.connect<mechanism_control::SwitchController, mechanism_control::SwitchControllerState, mechanism_control::SwitchController>(act);
  runner.run();

  node.spin();
  return 0;
}
