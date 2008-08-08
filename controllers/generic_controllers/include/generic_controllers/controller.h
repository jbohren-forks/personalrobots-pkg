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
/*! \namespace controller
 \brief The controller namespace

 \class controller::Controller
 \brief A base level controller class.

 */
/***************************************************/

#include <misc_utils/factory.h>
#include <mechanism_model/robot.h>

#include <tinyxml/tinyxml.h>
//class TiXmlElement;

namespace controller
{

enum ControllerErrorCode
{
  CONTROLLER_ALL_OK, CONTROLLER_JOINT_LIMIT, CONTROLLER_TORQUE_LIMIT,
  CONTROLLER_MODE_ERROR, //e.g. Position command given while in CONTROLLER_VELOCITY mode
  CONTROLLER_JOINT_ERROR, CONTROLLER_ACTUATOR_DISABLED, CONTROLLER_ACTUATOR_ENABLED, CONTROLLER_COMPUTATION_ERROR,
  CONTROLLER_CMD_SET
};

enum ControllerControlMode
{
  CONTROLLER_MODE_SET, CONTROLLER_ENABLED, CONTROLLER_DISABLED, CONTROLLER_TORQUE, CONTROLLER_POSITION,
  CONTROLLER_VELOCITY, CONTROLLER_AUTOMATIC, ETHERDRIVE_SPEED
};

class Controller;
typedef Factory<Controller> ControllerFactory;

#define ROS_REGISTER_CONTROLLER(c) \
  controller::Controller *ROS_New_##c() { return new c(); }             \
  bool ROS_CONTROLLER_##c = \
    controller::ControllerFactory::instance().registerType(#c, ROS_New_##c);

class Controller
{
public:
  Controller()
  {
  }
  virtual ~Controller()
  {
  }
  virtual void update(void) = 0;
  virtual void initXml(mechanism::Robot *robot, TiXmlElement *config) = 0;
};

struct PidControlParam
{
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double windup_min_;
  double windup_max_;
};

}
