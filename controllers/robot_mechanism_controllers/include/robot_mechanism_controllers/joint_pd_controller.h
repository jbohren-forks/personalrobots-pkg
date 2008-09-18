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
/*! \class controller::JointPDController
  \brief Joint Velocity Controller

  This class closes the loop around velocity using
  a pid loop.

  Example config:

  <controller type="JointPDController" name="controller_name">
    <joint name="joint_to_control">
      <pid p="1.0" i="2.0" d="3.0" iClamp="4.0" />
    </joint>
  </controller>
*/
/***************************************************/

#include <ros/node.h>

#include <mechanism_model/controller.h>
#include <control_toolbox/pid.h>

// Services
#include <robot_mechanism_controllers/SetPDCommand.h>
#include <robot_mechanism_controllers/GetPDActual.h>
#include <robot_mechanism_controllers/GetPDCommand.h>

namespace controller
{

  class JointPDController : public Controller
  {
    public:
    /*!
     * \brief Default Constructor of the JointController class.
     *
     */
    JointPDController();

    /*!
     * \brief Destructor of the JointController class.
     */
    ~JointPDController();

    /*!
     * \brief Functional way to initialize limits and gains.
     * \param p_gain Proportional gain.
     * \param i_gain Integral gain.
     * \param d_gain Derivative gain.
     * \param windup Intergral limit.
     * \param time The current hardware time.
     * \param *joint The joint that is being controlled.
     */

    void init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::RobotState *robot);
    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param double pos Velocity command to issue
     */
    void setPDCommand(double command, double command_dot);

    /*!
     * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
     */
    void getPDCommand(double &command, double &command_dot);

    /*!
     * \brief Get latest time..
     */
    double getTime();

    /*!
     * \brief Read the torque of the motor
     */
    double getMeasuredVelocity();

    /*!
     * \brief Read the torque of the motor
     */
    double getMeasuredPosition();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

    void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);


    std::string getJointName();

  private:

    mechanism::JointState* joint_; /**< Joint we're controlling. */
    mechanism::RobotState *robot_; /**< Pointer to robot structure. */
    control_toolbox::Pid pid_controller_;      /**< Internal PID controller. */
    double last_time_;        /**< Last time stamp of update. */
    double command_;          /**< Last commanded position. */
    double command_dot_;
    double command_t_;          /**< Last commanded position. */
    double command_dot_t_;

    /*!
     * \brief mutex lock for setting and getting commands
     */
    pthread_mutex_t joint_pd_controller_lock_;

  };

/***************************************************/
/*! \class controller::JointPDControllerNode
  \brief Joint Velocity Controller ROS Node

  This class closes the loop around velocity using
  a pid loop.

  The xml config is the same as for JointPDController except
  the addition of a "topic" attribute, which determines the
  namespace over which messages are published and services are
  offered.
*/
/***************************************************/

  class JointPDControllerNode : public Controller
  {
    public:
    /*!
     * \brief Default Constructor
     *
     */
    JointPDControllerNode();

    /*!
     * \brief Destructor
     */
    ~JointPDControllerNode();

    void update();

    void init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::RobotState *robot);
    bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

    // Services
    bool setPDCommand(robot_mechanism_controllers::SetPDCommand::request &req,
                    robot_mechanism_controllers::SetPDCommand::response &resp);

    // Services
    bool getPDCommand(robot_mechanism_controllers::GetPDCommand::request &req,
                    robot_mechanism_controllers::GetPDCommand::response &resp);

    bool getPDActual(robot_mechanism_controllers::GetPDActual::request &req,
                   robot_mechanism_controllers::GetPDActual::response &resp);

    private:
    JointPDController *c_;
  };
}
