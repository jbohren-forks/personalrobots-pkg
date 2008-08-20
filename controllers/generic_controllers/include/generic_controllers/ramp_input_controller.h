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
/*! \class controller::RampInputController
    \brief Ramp Input Controller

    This class basically gives a ramp input to an
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

class RampInputController : public Controller
{
public:
  /*!
   * \brief Default Constructor of the RampInputController class.
   *
   */
  RampInputController();

  /*!
   * \brief Destructor of the RampInputController class.
   */
  ~RampInputController();

  /*!
   * \brief Functional way to initialize.
   * \param input_start The start value of the ramp.
   * \param input_end The end value of the ramp.
   * \param duration The duration in seconds from start to finish.
   * \param time The current hardware time.
   * \param *joint The joint that is being controlled.
   */
  void init(double input_start, double input_end, double duration, double time,std::string name,mechanism::Robot *robot);

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
   * \brief Read the velocity of the joint
   */
  double getVelocity();

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
  double input_start_;          /**< Begining of the ramp. */
  double input_end_;            /**< End of the ramp. */
  double duration_;             /**< Duration of the ramp. */
  double initial_time_;         /**< Start time of the ramp. */

};

/***************************************************/
/*! \class controller::RampInputControllerNode
    \brief Ram Input Controller ROS Node

    This class basically gives a ramp input to an
    acuator.

*/
/***************************************************/

class RampInputControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  RampInputControllerNode();

  /*!
   * \brief Destructor
   */
  ~RampInputControllerNode();

  void update();

  void init(double input_start, double input_end, double duration, double time,std::string name,mechanism::Robot *robot);
  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

  bool getActual(generic_controllers::GetActual::request &req,
                  generic_controllers::GetActual::response &resp);

private:
  RampInputController *c_;
};
}


