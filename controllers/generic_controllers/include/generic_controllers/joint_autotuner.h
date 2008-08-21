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
#include <generic_controllers/controller.h>
#include <generic_controllers/pid.h>

// Services
#include <generic_controllers/SetCommand.h>
#include <generic_controllers/GetActual.h>

namespace controller
{

class JointAutotuner : public Controller
{

  static const double NUMCYCLES = 5; /*!<Number of cycles that need to match for autotuner to read as stable>!*/
  static const double AMPLITUDETOLERANCE = 0.05; //Allow 5% variance 
  static const double PERIODTOLERANCE = 0.05; //Allow 5% variance 
  static const double RELAYFRACTION = 0.05; //Use 5% of effort
 enum AutoControlState
  { 
    POSITIVE_PEAK,NEGATIVE_PEAK, DONE
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
  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

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
  double amplitude_;
  double last_amplitude_;
  double period_;
  double last_period_;

  double positive_peak_;
  double negative_peak_;

  double relay_height_;
  int successful_cycles_;

  double cycle_start_time_;

private:
  mechanism::Joint* joint_;  /**< Joint we're controlling. */
  Pid pid_controller_;       /**< Internal PID controller. */
  double last_time_;         /**< Last time stamp of update. */
  double command_;           /**< Last commanded position. */
  mechanism::Robot *robot_;  /**< Pointer to robot structure. */
};

/***************************************************/
/*! \class controller::JointAutotunerNode
    \brief Joint Position Controller ROS Node
    
    This class closes the loop around positon using
    a pid loop. 


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

  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

  // Services
  bool setCommand(generic_controllers::SetCommand::request &req,
                  generic_controllers::SetCommand::response &resp);

  bool getActual(generic_controllers::GetActual::request &req,
                  generic_controllers::GetActual::response &resp);

private:
  JointAutotuner *c_;
};
}

