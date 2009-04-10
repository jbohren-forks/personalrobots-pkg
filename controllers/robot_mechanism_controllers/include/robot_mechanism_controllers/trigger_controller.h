
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
#ifndef TRIGGER_CONTROLLER_H
#define TRIGGER_CONTROLLER_H

#include <ros/node.h>
#include <mechanism_model/controller.h>
#include <mechanism_model/robot.h>
#include <robot_mechanism_controllers/SetWaveform.h>
#include "hardware_interface/hardware_interface.h"

/** @class TriggerController
  * @brief Allows periodic triggering of cameras through the digital output
  * pin of the motor controller boards.
  * 
  */
  
namespace controller
{

class TriggerControllerNode;

class TriggerController : public Controller
{
  friend class TriggerControllerNode;
  
public:
  TriggerController();
  
  ~TriggerController();
  
  void update();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  
private:
  double getTick();
    
  mechanism::RobotState * robot_;
  ActuatorCommand *actuator_command_;
  
  double prev_tick_;
  
  // Configuration of controller.
  double rep_rate_;
  double phase_;
  double duty_cycle_;
  bool active_low_;
  bool running_;
  bool pulsed_;
  std::string actuator_name_;
};

/** @class TriggerControllerNode
* @brief Provides a thin wrapper for ROS communicaition with the trigger controller
*/
class TriggerControllerNode : public Controller
{
public:
  TriggerControllerNode();
  
  virtual ~TriggerControllerNode();

  void update();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  
private:
  bool setWaveformSrv(robot_mechanism_controllers::SetWaveform::Request &req,
      robot_mechanism_controllers::SetWaveform::Response &resp);
  
  TriggerController * c_;
};

};

#endif

