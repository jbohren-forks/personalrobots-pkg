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

#ifndef MK1001_H
#define MK1001_H

#include <ethercat_hardware/motor_control_board.h>

struct MK1001Command
{
  int16_t i_k;
  int16_t i_i;
  int16_t i_offset;
  int16_t i_desire;
  int16_t mode;
  int16_t shift;
  int16_t i_i_limit;
  int16_t qei_cpr;
  int16_t config;
}__attribute__ ((__packed__));

struct MK1001Status
{
  int32_t timestamp;
  int32_t qei_index_pos;
  int32_t qei_pos;
  int32_t qei_velocity;
  int16_t qei_error_count;
  int16_t debug;
  int16_t adc_current;
  int16_t adc_pot;
  int16_t adc_temp;
  int16_t adc_voltage;
  int16_t pwm_cmd;
}__attribute__ ((__packed__)) ;

class MK1001 : public MotorControlBoard
{
  static const int STATUS_PHY_ADDR = 0x1200;
  static const int COMMAND_PHY_ADDR = 0x1100;

  enum
  {
    MODE_OFF, MODE_PWM, MODE_PID
  };

public:
  MK1001() :
    MotorControlBoard(MK1001_PRODUCT_CODE, sizeof(MK1001Command), sizeof(MK1001Status))
  {
  }
  void configure(int &startAddress, EtherCAT_SlaveHandler *sh);
  void convertCommand(ActuatorCommand &command, unsigned char *buffer);
  void convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer);
  bool hasActuator(void)
  {
    return true;
  }

private:
  static const EC_UDINT MK1001_PRODUCT_CODE = 0x000003E9;
};

#endif /* MK1001_H */
