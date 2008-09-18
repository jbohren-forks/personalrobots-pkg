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
/*! \class controller::RampEffortController
    \brief Ramp Input Controller

    This class basically gives a ramp input to an
    acuator.

*/
/***************************************************/


#include <ros/node.h>
#include <mechanism_model/controller.h>

// Services
#include <robot_mechanism_controllers/SetCommand.h>
#include <robot_mechanism_controllers/GetActual.h>

namespace controller
{

class RampEffortController : public Controller
{
public:
  /*!
   * \brief Default Constructor of the RampEffortController class.
   *
   */
  RampEffortController();

  /*!
   * \brief Destructor of the RampEffortController class.
   */
  ~RampEffortController();

  void init(double input_start, double input_end, double duration, double time,std::string name,mechanism::RobotState *robot);
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

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
  mechanism::JointState *joint_state_;     /**< Joint we're controlling. */
  mechanism::RobotState *robot_;     /**< Pointer to robot structure. */
  double input_start_;          /**< Begining of the ramp. */
  double input_end_;            /**< End of the ramp. */
  double duration_;             /**< Duration of the ramp. */
  double initial_time_;         /**< Start time of the ramp. */

};

/***************************************************/
/*! \class controller::RampEffortControllerNode
    \brief Ram Input Controller ROS Node

    This class basically gives a ramp input to an
    acuator.

*/
/***************************************************/

class RampEffortControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  RampEffortControllerNode();

  /*!
   * \brief Destructor
   */
  ~RampEffortControllerNode();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  bool getActual(robot_mechanism_controllers::GetActual::request &req,
                  robot_mechanism_controllers::GetActual::response &resp);

private:
  RampEffortController *c_;
};
}


