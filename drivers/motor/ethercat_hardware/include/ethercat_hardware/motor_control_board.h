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

#ifndef MOTOR_CONTROL_BOARD_H
#define MOTOR_CONTROL_BOARD_H

#include <vector>

#include <ethercat/ethercat_defs.h>
#include <al/ethercat_slave_handler.h>

#include <hw_interface/hardware_interface.h>

using namespace std;

class MotorControlBoard
{
public:
  MotorControlBoard(EC_UDINT productCode, int commandSize = 0, int statusSize = 0)
  {
    this->productCode = productCode;
    this->commandSize = commandSize;
    this->statusSize = statusSize;
  }
  virtual void configure(int &startAddress, EtherCAT_SlaveHandler *sh) = 0;
  virtual void convertCommand(ActuatorCommand &command, unsigned char *buffer) = 0;
  virtual void convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer) = 0;
  virtual bool hasActuator(void) = 0;

  EC_UDINT productCode;
  unsigned int commandSize;
  unsigned int statusSize;
};

extern vector<MotorControlBoard *> boards;

#endif /* MOTOR_CONTROL_BOARD_H */
