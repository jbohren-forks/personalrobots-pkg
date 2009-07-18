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

#ifndef JOINT_EFFORT_CONTROLLER_H
#define JOINT_EFFORT_CONTROLLER_H

/***************************************************/
/*! \class controller::JointEffortController
    \brief Joint Torque Controller

    This class basically passes the commanded effort
    down through the transmissions and safety code.

    Example config:<br>

    <controller type="JointEffortController" name="controller_name"><br>
      <joint name="joint_to_control" /><br>
    </controller><br>

*/
/***************************************************/

#include <ros/node.h>
#include <mechanism_model/controller.h>
#include "misc_utils/advertised_service_guard.h"
#include "misc_utils/subscription_guard.h"

// Services
#include <std_msgs/Float64.h>
#include <robot_srvs/GetValue.h>


namespace controller
{

class JointEffortController : public Controller
{
public:

  JointEffortController();
  ~JointEffortController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  bool init(mechanism::RobotState *robot, const std::string &joint_name);

  virtual bool starting() { command_ = 0.0; return true; }
  virtual void update();

  mechanism::JointState *joint_state_;

  double command_;

private:
  mechanism::RobotState *robot_;
};

/***************************************************/
/*! \class controller::JointEffortControllerNode
    \brief Joint Torque Controller ROS Node

    This class basically passes the commanded effort
    down through the transmissions and safety code.

*/
/***************************************************/

class JointEffortControllerNode : public Controller
{
public:

  JointEffortControllerNode();
  ~JointEffortControllerNode();

  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  virtual bool starting() { return c_.starting(); }
  virtual void update();

  void command();

private:
 //node stuff
  std::string name_;
  ros::Node *node_;

  //msgs
  std_msgs::Float64 command_msg_;

  //controller
  JointEffortController c_;

};
}

#endif
