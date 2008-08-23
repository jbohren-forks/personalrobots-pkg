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
/*! \class controller::SineSweepController
    \brief Sine Sweep Controller

    This class basically gives a sine sweep to an
    acuator.

*/
/***************************************************/


#include <ros/node.h>
#include <generic_controllers/controller.h>

// Services
#include <generic_controllers/SetCommand.h>
#include <generic_controllers/GetActual.h>

namespace controller
{

class SineSweepController : public Controller
{
public:
  /*!
   * \brief Default Constructor of the SineSweepController class.
   *
   */
  SineSweepController();

  /*!
   * \brief Destructor of the SineSweepController class.
   */
  ~SineSweepController();

  /*!
   * \brief Functional way to initialize.
   * \param start_freq The start value of the sweep.
   * \param end_freq  The end value of the sweep.
   * \param sample_freq  The update frequency of the sweep.
   * \param amplitude The amplitude of the sweep.
   * \param duration The duration in seconds from start to finish.
   * \param time The current hardware time.
   * \param *robot The robot that is being controlled.
   */
  void init(double start_freq, double end_freq, double duration, double amplitude, double time,std::string name,mechanism::Robot *robot);

  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
  double getCommand();

  /*!
   * \brief Read the effort of the joint
   */
  double getMeasuredEffort();


  /*!
   * \brief Get latest time..
   */
  double getTime();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();

private:
  mechanism::Joint* joint_;     /**< Joint we're controlling. */
  mechanism::Robot *robot_;     /**< Pointer to robot structure. */
  double start_freq_;           /**< Begining of the sweep. */
  double end_freq_;             /**< End of the sweep. */
  double amplitude_;            /**< Amplitude of the sweep. */
  double duration_;             /**< Duration of the sweep. */
  double initial_time_;         /**< Start time of the sweep. */
  double start_angular_freq_;   /**< Start time of the sweep. */
  double end_angular_freq_;    /**< Start time of the sweep. */
  double K_factor_ ;            /**< Start time of the sweep. */
  double L_factor_ ;            /**< Start time of the sweep. */

};

/***************************************************/
/*! \class controller::SineSweepControllerNode
    \brief Ram Input Controller ROS Node

    This class basically gives a ramp input to an
    acuator.

*/
/***************************************************/

class SineSweepControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  SineSweepControllerNode();

  /*!
   * \brief Destructor
   */
  ~SineSweepControllerNode();

  void update();

  void init(double start_freq, double end_freq, double duration, double amplitude, double time,std::string name,mechanism::Robot *robot);
  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

private:
  SineSweepController *c_;
};
}


