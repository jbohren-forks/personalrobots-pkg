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
/*! \class controller::JointVelocityController
    \brief Joint Velocity Controller

    This class closes the loop around velocity using
    a pid loop.

*/
/***************************************************/

#include <ros/node.h>

#include <generic_controllers/controller.h>
#include <generic_controllers/pid.h>

// Services
#include <generic_controllers/SetCommand.h>
#include <generic_controllers/GetActual.h>

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
   * \param p_gain Proportional gain.
   * \param i_gain Integral gain.
   * \param d_gain Derivative gain.
   * \param windup Intergral limit.
   * \param time The current hardware time.
   * \param *joint The joint that is being controlled.
   */

  void init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::Robot *robot);
  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

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
   * \brief Get latest time..
   */
  double getTime();

  /*!
   * \brief Read the torque of the motor
   */
  double getMeasuredVelocity();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();

private:
  mechanism::Joint* joint_; /**< Joint we're controlling. */
  mechanism::Robot *robot_; /**< Pointer to robot structure. */
  Pid pid_controller_;      /**< Internal PID controller. */
  double last_time_;        /**< Last time stamp of update. */
  double command_;          /**< Last commanded position. */

};

/***************************************************/
/*! \class controller::JointVelocityControllerNode
    \brief Joint Velocity Controller ROS Node

    This class closes the loop around velocity using
    a pid loop.

    Example config:

    <controller type="JointVelocityControllerNode" topic="some_topic_name">
      <joint name="joint_to_control">
        <pid p="1.0" i="2.0" d="3.0" iClamp="4.0" />
      </joint>
    </controller>
*/
/***************************************************/

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

  void init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::Robot *robot);
  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

  // Services
  bool setCommand(generic_controllers::SetCommand::request &req,
                  generic_controllers::SetCommand::response &resp);

  bool getActual(generic_controllers::GetActual::request &req,
                  generic_controllers::GetActual::response &resp);

private:
  JointVelocityController *c_;
};
}

