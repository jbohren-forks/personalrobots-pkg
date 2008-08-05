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

#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <stl_utils/stl_utils.h>

class ActuatorState{
public:
  ActuatorState() :
      last_calibration_high_transition_(0),
      last_calibration_low_transition_(0),
      is_enabled_(0),
      run_stop_hit_(0),
      last_requested_current_(0),
      last_commanded_current_(0),
      last_measured_current_(0),
      num_encoder_errors_(0)
  {}
  int encoder_count_;
  double timestamp_;
  double encoder_velocity_;
  bool calibration_reading_;
  int last_calibration_high_transition_;
  int last_calibration_low_transition_;
  bool is_enabled_;
  bool run_stop_hit_;

  double last_requested_current_;
  double last_commanded_current_;
  double last_measured_current_;

  int motor_voltage_;

  int num_encoder_errors_;
  int num_communication_errors_;
};

class ActuatorCommand
{
public:
  ActuatorCommand() : enable_(0), current_(0)
  {
  }
  bool enable_;
  double current_;
};

class Actuator
{
public:
  ActuatorState state_;
  ActuatorCommand command_;
};

class HardwareInterface
{
public:
  HardwareInterface(int num_actuators)
    : actuators_(num_actuators, (Actuator*)NULL)
  {
  }
  ~HardwareInterface()
  {
    deleteElements(&actuators_);
  }
  std::vector<Actuator*> actuators_;
  double current_time_;
};

#endif
