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
/*! \class controller::JointAutotuner
    \brief Joint Position Controller

    This class closes the loop around positon using
    a pid loop.

*/
/***************************************************/

#include <ros/node.h>
#include <vector>
#include <mechanism_model/controller.h>
#include <control_toolbox/pid.h>

// Services
#include <robot_mechanism_controllers/SetCommand.h>
#include <robot_mechanism_controllers/GetActual.h>

namespace controller
{

class JointAutotuner : public Controller
{
 enum AutoControlState
  {
    START,POSITIVE_PEAK,NEGATIVE_PEAK, DONE, MANUAL
  };
public:
  /*!
   * \brief Default Constructor of the JointAutotuner class.
   *
   */
  JointAutotuner();

  /*!
   * \brief Destructor of the JointAutotuner class.
   */
  ~JointAutotuner();

  /*!
   * \brief Functional way to initialize limits and gains.
   * \param p_gain Proportional gain.
   * \param i_gain Integral gain.
   * \param d_gain Derivative gain.
   * \param windup Intergral limit.
   * \param time The current hardware time.
   * \param *joint The joint that is being controlled.
   */
  void init(double p_gain, double i_gain, double d_gain, double windup, double time,mechanism::Robot *robot, mechanism::Joint *joint);
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
  double getMeasuredState();

  /*!
   * \brief Get latest time..
   */
  double getTime();


  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();

  double p_gain_;
  double i_gain_;
  double d_gain_;

  AutoControlState current_state_;

private:
  bool tune_velocity_; /**<If true, uses velocity to tune. Otherwise uses position>*/
  mechanism::Joint* joint_;  /**< Joint we're controlling.> */
  mechanism::JointState* joint_state_;  /**< Joint we're controlling.> */
  control_toolbox::Pid pid_controller_;       /**< Internal PID controller.> */
  double last_time_;         /**< Last time stamp of update.> */
  double command_;           /**< Last commanded position.> */
  mechanism::Robot *robot_;  /**< Pointer to robot structure.> */
  mechanism::RobotState *robot_state_;  /**< Pointer to robot structure.> */
  const char* file_path_; /**<Filename and location to write results. >*/
  void writeGainValues(double period, double amplitude, double relay_height); /**<Calculate and write gain values> */

  double amplitude_; /**< Current amplitude of relay cycle> */
  double last_amplitude_;/**< Last amplitude of relay cycle> */
  double period_;/**< Current period of relay cycle> */
  double last_period_;/**< Last period of relay cycle> */

  double positive_peak_;/**< Positive peak reached in cycle> */
  double negative_peak_;/**< Negative peak reached in cycle> */

  double relay_height_;/**< Amount of relay input> */
  int successful_cycles_;/**< Number of matching cycles > */
  double crossing_point_;/**< Location of crossover point for relay test> */
  double cycle_start_time_;/**< Mark time of cycle start> */

  int num_cycles_; /*!<Number of cycles that need to match for autotuner to read as stable>!*/
  double amplitude_tolerance_; /*!<% variation amplitude allowed between successful cycles>!*/
  double period_tolerance_; /*!<% variation period allowed between successful cycles>!*/
  double relay_effort_percent_; /*!<% of effort limit to use in relay test>!*/

};

/***************************************************/
/*! \class controller::JointAutotunerNode
    \brief Joint Position Controller ROS Node

   This class performs an autotuning routine using the relay method.


*/
/***************************************************/

class JointAutotunerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  JointAutotunerNode();

  /*!
   * \brief Destructor
   */
  ~JointAutotunerNode();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  // Services
  bool setCommand(robot_mechanism_controllers::SetCommand::Request &req,
                  robot_mechanism_controllers::SetCommand::Response &resp);

  bool getActual(robot_mechanism_controllers::GetActual::Request &req,
                  robot_mechanism_controllers::GetActual::Response &resp);

private:
  JointAutotuner *c_;
};
}

