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

#ifndef WG05_H
#define WG05_H

#include <ethercat_hardware/ethercat_device.h>

struct WG05Status
{
  uint8_t mode_;
  uint8_t digital_out_;
  uint16_t programmed_pwm_value_;
  int16_t programmed_current_;
  int16_t measured_current_;
  uint32_t timestamp_;
  uint32_t encoder_count_;
  uint32_t encoder_index_pos_;
  uint16_t num_encoder_errors_;
  uint8_t encoder_status_;
  uint8_t calibration_reading_;
  uint32_t last_calibration_high_transition_;
  uint32_t last_calibration_low_transition_;
  uint16_t board_temperature_;
  uint16_t bridge_temperature_;
  uint16_t supply_voltage_;
  uint16_t motor_voltage_;
  uint16_t packet_count_;
  uint8_t pad_;
  uint8_t checksum_;
} __attribute__ ((__packed__));

struct WG05Command
{
  uint8_t mode_;
  uint8_t digital_out_;
  int16_t programmed_pwm;
  int16_t programmed_current_;
  uint8_t pad_;
  uint8_t checksum_;
} __attribute__ ((__packed__));

class WG05 : public EthercatDevice
{
  static const int STATUS_PHY_ADDR = 0x2000;
  static const int COMMAND_PHY_ADDR = 0x1000;

  enum
  {
    MODE_OFF = 0x00, MODE_CURRENT = 0x01, MODE_ENABLE = 0x02, MODE_UNDERVOLTAGE = 0x04, MODE_RESET = 0x80
  };

  enum
  {
    LIMIT_SENSOR_0_STATE = 1, LIMIT_SENSOR_1_STATE = 2,
    LIMIT_ON_TO_OFF = 4, LIMIT_OFF_TO_ON = 8
  };

public:
  WG05() : EthercatDevice(true, sizeof(WG05Command), sizeof(WG05Status)) {}

  void configure(int &start_address, EtherCAT_SlaveHandler *sh);

  void convertCommand(ActuatorCommand &command, unsigned char *buffer);
  void convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer);

  void truncateCurrent(ActuatorCommand &command);
  void verifyState(unsigned char *buffer);

  enum {PRODUCT_CODE = 6805005};

private:
  // TODO: pull these values from the EEPROM
  static const double CURRENT_FACTOR = 2000.0;
  double max_current_;

};

#endif /* WG05_H */
