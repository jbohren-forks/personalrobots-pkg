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

    Example config:

    <controller type="JointPositionController" name="controller_name" topic="a_topic">
      <joint name="head_tilt_joint">
        <pid p="1.0" i="0.0" d="3.0" iClamp="0.0" />
      </joint>
    </controller>

*/
/***************************************************/

#include <ros/node.h>

#include <mechanism_model/controller.h>
#include <control_toolbox/pid.h>
#include "misc_utils/advertised_service_guard.h"

// Services
#include <robot_mechanism_controllers/SetCommand.h>
#include <robot_mechanism_controllers/GetActual.h>

#include <robot_mechanism_controllers/SingleJointPosCmd.h>

namespace controller
{

class JointPositionController : public Controller
{
public:
  /*!
   * \brief Default Constructor of the JointPositionController class.
   *
   */
  JointPositionController();

  /*!
   * \brief Destructor of the JointPositionController class.
   */
  ~JointPositionController();

  /*!
   * \brief Functional way to initialize limits and gains.
   * \param p_gain Proportional gain.
   * \param i_gain Integral gain.
   * \param d_gain Derivative gain.
   * \param windup Intergral limit.
   * \param time The current hardware time.
   * \param *joint The joint that is being controlled.
   */
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double command);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
  double getCommand();

  /*!
   * \brief Read the torque of the motor
   */
  double getMeasuredPosition();

  /*!
   * \brief Get latest time..
   */
  double getTime();


  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

  std::string getJointName();
  mechanism::JointState *joint_state_;  /**< Joint we're controlling. */
private:
  
  mechanism::RobotState *robot_;  /**< Pointer to robot structure. */
  control_toolbox::Pid pid_controller_;       /**< Internal PID controller. */
  double last_time_;         /**< Last time stamp of update. */
  double command_;           /**< Last commanded position. */

  double smoothed_error_; /** */
  double smoothing_factor_;

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
  /*!
   * \brief Default Constructor
   *
   */
  JointPositionControllerNode();

  /*!
   * \brief Destructor
   */
  ~JointPositionControllerNode();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  // Services
  bool setCommand(robot_mechanism_controllers::SetCommand::request &req,
                  robot_mechanism_controllers::SetCommand::response &resp);
  void setCommand(double command);

  double getCommand();

  double getMeasuredPosition();

  bool getActual(robot_mechanism_controllers::GetActual::request &req,
                 robot_mechanism_controllers::GetActual::response &resp);

  /*!
   * \brief ROS topic callback
   */
  void setJointPosSingle();

private:
  robot_mechanism_controllers::SingleJointPosCmd msg_;   //The message used by the ROS callback
  JointPositionController *c_;

  AdvertisedServiceGuard guard_set_command_, guard_get_actual_;
  
  /*!
   * \brief service prefix
   */
  std::string service_prefix_;

  /*!
   * \brief publish topic name
   */
  std::string topic_name_;
  
  /*!
   * \brief xml pointer to ros topic name
   */
  TiXmlElement * ros_cb_;

  /*!
   * \brief A pointer to ros node
   */
  ros::node *node_;
};
}

