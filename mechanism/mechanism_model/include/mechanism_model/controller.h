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

#include <loki/Factory.h>
#include <mechanism_model/robot.h>

#include <tinyxml/tinyxml.h>

namespace controller
{

  /*! \struct
    \brief This class holds information for a joint control parameter structure.
   */
  typedef struct
  {
      double p_gain; /** P gain */

      double i_gain; /** I gain */

      double d_gain; /** D gain */

      double windup; /** windup protection value */

      std::string joint_name; /** joint name */

      std::string control_type; /** control type */

  }JointControlParam;


class Controller;
typedef Loki::SingletonHolder
<
  Loki::Factory< Controller, std::string >,
  Loki::CreateUsingNew,
  Loki::LongevityLifetime::DieAsSmallObjectChild
> ControllerFactory;

#define ROS_REGISTER_CONTROLLER(c) \
  controller::Controller *ROS_New_##c() { return new c(); }             \
  bool ROS_CONTROLLER_##c = \
    controller::ControllerFactory::Instance().Register(#c, ROS_New_##c);

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
  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config) = 0;
};

}
