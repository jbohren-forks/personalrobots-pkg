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

#pragma once

/***************************************************/
/*! \class controller::JointPositionController
    \brief Joint Position Controller

    This class closes the loop around positon using
    a pid loop.

    Example config:<br>

    <controller type="JointPositionController" name="controller_name" topic="a_topic"><br>
      <joint name="head_tilt_joint"><br>
        <pid p="1.0" i="0.0" d="3.0" iClamp="0.0" /><br>
      </joint><br>
    </controller><br>
*/
/***************************************************/

#include <ros/node.h>

#include <mechanism_model/controller.h>
#include <control_toolbox/pid.h>
#include "misc_utils/advertised_service_guard.h"
#include "misc_utils/subscription_guard.h"

// Services
#include <std_msgs/Float64.h>
#include <robot_srvs/GetJointCmd.h>

namespace controller
{

class JointPositionController : public Controller
{
public:

  JointPositionController();
  ~JointPositionController();

 /*!
   * \brief Functional way to initialize limits and gains.
   * \param pid Pid gain values.
   * \param joint_name Name of joint we want to control.
   * \param *robot The robot.
   */
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  bool init(mechanism::RobotState *robot, const std::string &joint_name,const control_toolbox::Pid &pid);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double command);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
   void getCommand(robot_msgs::JointCmd & cmd);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

  std::string getJointName();
  mechanism::JointState *joint_state_;        /**< Joint we're controlling. */

private:

  mechanism::RobotState *robot_;              /**< Pointer to robot structure. */
  control_toolbox::Pid pid_controller_;       /**< Internal PID controller. */
  double last_time_;                          /**< Last time stamp of update. */
  double command_;                            /**< Last commanded position. */

};

/***************************************************/
/*! \class controller::JointPositionControllerNode
    \brief Joint Position Controller ROS Node

    This class closes the loop around positon using
    a pid loop.
*/
/***************************************************/

class JointPositionControllerNode : public Controller
{
public:

  JointPositionControllerNode();
  ~JointPositionControllerNode();

  void update();
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  // Topics
  void setCommand();
  //Sevices
  bool getCommand(robot_srvs::GetJointCmd::request &req,
		  robot_srvs::GetJointCmd::response &resp);


private:

  //node stuff
  std::string service_prefix_;                 /**< The name of the controller. */
  ros::node *node_;
  AdvertisedServiceGuard guard_get_command_;   /**< Makes sure the advertise goes down neatly. */
  SubscriptionGuard guard_set_command_;        /**< Makes sure the subscription goes down neatly. */

  //msgs
  std_msgs::Float64 cmd_;                      /**< The command from the subscription. */

  //controller
  JointPositionController *c_;                 /**< The controller. */

};
}

