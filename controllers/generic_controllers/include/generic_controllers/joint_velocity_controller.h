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

#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H

#include <ros/node.h>

#include <generic_controllers/controller.h>
#include <generic_controllers/pid.h>

#include <generic_controllers/SetVelocity.h>

namespace controller
{

class JointVelocityController : public Controller
{
public:
  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  JointVelocityController();

  /*!
   * \brief Destructor of the JointController class.
   */
  ~JointVelocityController();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  void init(double p_gain, double i_gain, double d_gain, double windup, double time, mechanism::Joint *joint);
  void initXml(mechanism::Robot *robot, TiXmlElement *config);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param double pos Velocity command to issue
   */
  void setCommand(double command);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
  double getCommand();

  /*!
   * \brief Read the torque of the motor
   */
  double getActual();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();

private:
  /*!
   * \brief Actually issue torque set command of the joint motor.
   */
  void setJointEffort(double torque);

  mechanism::Joint* joint_; /*!< Joint we're controlling>*/
  Pid pid_controller_; /*!< Internal PID controller>*/
  double last_time_; /*!< Last time stamp of update> */
  double command_; /*!< Last commanded position> */
  mechanism::Robot *robot_; /*!< Pointer to robot structure>*/
};

class JointVelocityControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  JointVelocityControllerNode();

  /*!
   * \brief Destructor
   */
  ~JointVelocityControllerNode();

  void update();

  void initXml(mechanism::Robot *robot, TiXmlElement *config);

  bool setVelocity(generic_controllers::SetVelocity::request &req,
                   generic_controllers::SetVelocity::response &resp);

private:
  JointVelocityController *c_;
};
}

#endif /* JOINT_VELOCITY_CONTROLLER_H */
