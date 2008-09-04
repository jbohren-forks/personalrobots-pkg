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

struct WG05MbxHdr
{
  uint16_t address_;
  union
  {
    uint16_t command_;
    struct
    {
      uint16_t length_:12;
      uint16_t pad_:3;
      uint16_t write_nread_:1;
    }__attribute__ ((__packed__));
  };
  uint8_t checksum_;

  void build(uint16_t address, uint16_t length, bool write_nread);
  bool verify_checksum(void) const;
}__attribute__ ((__packed__));

static const unsigned MBX_SIZE = 512;
static const unsigned MBX_DATA_SIZE = (MBX_SIZE - sizeof(WG05MbxHdr) - 1);
struct WG05MbxCmd
{
  WG05MbxHdr hdr_;
  uint8_t data_[MBX_DATA_SIZE];
  uint8_t checksum_;

  void build(unsigned address, unsigned length, bool write_nread, void const* data);
}__attribute__ ((__packed__));

struct WG05ConfigInfo
{
  uint32_t product_id_;
  union
  {
    uint32_t revision_;
    struct
    {
      uint8_t firmware_minor_revision_;
      uint8_t firmware_major_revision_;
      uint8_t pca_revision_;
      uint8_t pcb_revision_;
    }__attribute__ ((__packed__));
  }__attribute__ ((__packed__));
  uint32_t device_serial_number_;
  uint8_t current_loop_kp_;
  uint8_t current_loop_ki_;
  uint16_t absolute_current_limit_;
  float nominal_current_scale_;
  float nominal_voltage_scale_;
  uint8_t pad_[8];
  uint8_t configuration_status_;
  uint8_t safety_disable_status_;
  uint8_t safety_disable_countdown_;
  uint8_t safety_disable_count_;

  static const unsigned CONFIG_INFO_BASE_ADDR = 0x0080;
}__attribute__ ((__packed__));

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
}__attribute__ ((__packed__));

struct WG05Command
{
  uint8_t mode_;
  uint8_t digital_out_;
  int16_t programmed_pwm;
  int16_t programmed_current_;
  uint8_t pad_;
  uint8_t checksum_;
}__attribute__ ((__packed__));

class WG05 : public EthercatDevice
{
public:
  WG05() : EthercatDevice(true, sizeof(WG05Command), sizeof(WG05Status)) {}

  void configure(int &start_address, EtherCAT_SlaveHandler *sh);

  void convertCommand(ActuatorCommand &command, unsigned char *buffer);
  void convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer);

  void truncateCurrent(ActuatorCommand &command);
  void verifyState(unsigned char *buffer);

  enum
  {
    PRODUCT_CODE = 6805005
  };

private:
  int mailboxRead(EtherCAT_SlaveHandler *sh, int address, void *data, int length);
  int writeData(EtherCAT_SlaveHandler *sh, EC_UINT address, void const* buffer, EC_UINT length);

  int readData(EtherCAT_SlaveHandler *sh, EC_UINT address, void* buffer, EC_UINT length);

  static const int COMMAND_PHY_ADDR = 0x1000;
  static const int STATUS_PHY_ADDR = 0x2000;
  static const int MBX_COMMAND_PHY_ADDR = 0x1400;
  static const int MBX_COMMAND_SIZE = 512;
  static const int MBX_STATUS_PHY_ADDR = 0x2400;
  static const int MBX_STATUS_SIZE = 512;

  enum
  {
    MODE_OFF = 0x00, MODE_CURRENT = 0x01, MODE_ENABLE = 0x02,
    MODE_UNDERVOLTAGE = 0x04, MODE_RESET = 0x80
  };

  enum
  {
    LIMIT_SENSOR_0_STATE = 1, LIMIT_SENSOR_1_STATE = 2,
    LIMIT_ON_TO_OFF = 4, LIMIT_OFF_TO_ON = 8
  };

  // Board configuration parameters
  double max_current_;
  WG05ConfigInfo config_info_;
};

#endif /* WG05_H */
