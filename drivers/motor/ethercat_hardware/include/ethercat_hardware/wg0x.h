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

#ifndef WG0X_H
#define WG0X_H

#include <ethercat_hardware/ethercat_device.h>

#ifndef WG0X_STANDALONE
#include <misc_utils/realtime_publisher.h>
#include <ethercat_hardware/PressureState.h>
#endif

struct WG0XMbxHdr
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
  bool verifyChecksum(void) const;
}__attribute__ ((__packed__));

static const unsigned MBX_SIZE = 512;
static const unsigned MBX_DATA_SIZE = (MBX_SIZE - sizeof(WG0XMbxHdr) - 1);
struct WG0XMbxCmd
{
  WG0XMbxHdr hdr_;
  uint8_t data_[MBX_DATA_SIZE];
  uint8_t checksum_;

  void build(unsigned address, unsigned length, bool write_nread, void const* data);
}__attribute__ ((__packed__));

struct WG0XSpiEepromCmd
{
  uint16_t page_;
  union
  {
    uint8_t command_;
    struct
    {
      uint8_t operation_ :4;
      uint8_t start_ :1;
      uint8_t busy_ :1;
      uint8_t unused2_ :2;
    }__attribute__ ((__packed__));
  };

  void build_read(unsigned page)
  {
    this->page_ = page & 0xffff;
    this->operation_ = SPI_READ_OP;
    this->start_ = 1;
  }
  void build_write(unsigned page)
  {
    this->page_ = page & 0xffff;
    this->operation_ = SPI_WRITE_OP;
    this->start_ = 1;
  }
  void build_arbitrary(unsigned length)
  {
    this->page_ = (length-1) & 0xffff;
    this->operation_ = SPI_ARBITRARY_OP;
    this->start_ = 1;
  }

  static const unsigned SPI_READ_OP = 0;
  static const unsigned SPI_WRITE_OP = 1;
  static const unsigned SPI_ARBITRARY_OP = 3;

  static const unsigned SPI_COMMAND_ADDR = 0x0230;
  static const unsigned SPI_BUFFER_ADDR = 0xF400;
}__attribute__ ((__packed__));

struct WG0XConfigInfo
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

struct WG0XActuatorInfo
{
  uint16_t major_;              // Major revision
  uint16_t minor_;              // Minor revision
  uint32_t id_;                 // Actuator ID
  char name_[64];               // Actuator name
  char robot_name_[32];         // Robot name
  char motor_make_[32];         // Motor manufacturer
  char motor_model_[32];        // Motor model #
  double max_current_;          // Maximum current
  double speed_constant_;       // Speed constant
  double resistance_;           // Resistance
  double motor_torque_constant_; // Motor torque constant
  uint32_t pulses_per_revolution_; // # of encoder ticks per revolution
  int32_t sign_;                // Sign of encoder count, should be 1 or -1
  uint8_t pad[52];              // Pad entire structure to 264 bytes
  uint32_t crc32_;              // CRC32 over structure (minus last 4 bytes)
};

struct WG0XStatus
{
  uint8_t mode_;
  uint8_t digital_out_;
  int16_t programmed_pwm_value_;
  int16_t programmed_current_;
  int16_t measured_current_;
  uint32_t timestamp_;
  int32_t encoder_count_;
  int32_t encoder_index_pos_;
  uint16_t num_encoder_errors_;
  uint8_t encoder_status_;
  uint8_t calibration_reading_;
  int32_t last_calibration_high_transition_;
  int32_t last_calibration_low_transition_;
  uint16_t board_temperature_;
  uint16_t bridge_temperature_;
  uint16_t supply_voltage_;
  int16_t motor_voltage_;
  uint16_t packet_count_;
  uint8_t pad_;
  uint8_t checksum_;
}__attribute__ ((__packed__));

struct WG0XCommand
{
  uint8_t mode_;
  uint8_t digital_out_;
  int16_t programmed_pwm;
  int16_t programmed_current_;
  uint8_t pad_;
  uint8_t checksum_;
}__attribute__ ((__packed__));

class WG0X : public EthercatDevice
{
public:
  WG0X(bool has_actuator = true, int command_size = sizeof(WG0XCommand), int status_size = sizeof(WG0XStatus)) : EthercatDevice(has_actuator, command_size, status_size)
  {
    strings_.reserve(10);
    values_.reserve(10);
  }

  EthercatDevice *configure(int &start_address, EtherCAT_SlaveHandler *sh);
  int initialize(Actuator *, bool);
  void initXml(TiXmlElement *);

  void convertCommand(ActuatorCommand &command, unsigned char *buffer);
  virtual void convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer);

  void computeCurrent(ActuatorCommand &command);
  void truncateCurrent(ActuatorCommand &command);
  void verifyState(ActuatorState &state, unsigned char *buffer);

  void program(WG0XActuatorInfo *);
  bool isProgrammed() { return actuator_info_.crc32_ != 0;}

  void diagnostics(robot_msgs::DiagnosticStatus &d, unsigned char *);

private:
  int readEeprom(EtherCAT_SlaveHandler *sh);
  int writeEeprom(EtherCAT_SlaveHandler *sh);
  int sendSpiCommand(EtherCAT_SlaveHandler *sh, WG0XSpiEepromCmd const * cmd);
  int writeMailbox(EtherCAT_SlaveHandler *sh, int address, void const *data, EC_UINT length);
  int readMailbox(EtherCAT_SlaveHandler *sh, int address, void *data, EC_UINT length);
  int writeData(EtherCAT_SlaveHandler *sh, EC_UINT address, void const* buffer, EC_UINT length);
  int readData(EtherCAT_SlaveHandler *sh, EC_UINT address, void* buffer, EC_UINT length);

  static const int COMMAND_PHY_ADDR = 0x1000;
  static const int STATUS_PHY_ADDR = 0x2000;
  static const int PRESSURE_PHY_ADDR = 0x2200;
  static const int MBX_COMMAND_PHY_ADDR = 0x1400;
  static const int MBX_COMMAND_SIZE = 512;
  static const int MBX_STATUS_PHY_ADDR = 0x2400;
  static const int MBX_STATUS_SIZE = 512;

  enum
  {
    MODE_OFF = 0x00,
    MODE_ENABLE = (1 << 0),
    MODE_CURRENT = (1 << 1),
    MODE_SAFETY_RESET = (1 << 4),
    MODE_SAFETY_LOCKOUT = (1 << 5),
    MODE_UNDERVOLTAGE = (1 << 6),
    MODE_RESET = (1 << 7)
  };

  enum
  {
    LIMIT_SENSOR_0_STATE = 1, LIMIT_SENSOR_1_STATE = 2,
    LIMIT_ON_TO_OFF = 4, LIMIT_OFF_TO_ON = 8
  };

  // Board configuration parameters
  WG0XConfigInfo config_info_;
  WG0XActuatorInfo actuator_info_;
  static const int ACTUATOR_INFO_PAGE = 4095;

  // Diagnostic message values
  vector<robot_msgs::DiagnosticString> strings_;
  vector<robot_msgs::DiagnosticValue> values_;
};

class WG05 : public WG0X
{
public:
  enum
  {
    PRODUCT_CODE = 6805005
  };
};

struct WG06Pressure
{
  uint32_t timestamp_;
  uint16_t data0_[22];
  uint16_t data1_[22];
  uint8_t pad_;
  uint8_t checksum_;
} __attribute__((__packed__));

class WG06 : public WG0X
{
public:
  WG06() : WG0X(true, sizeof(WG0XCommand), sizeof(WG0XStatus)+sizeof(WG06Pressure))
#ifndef WG0X_STANDALONE
    , publisher_("pressure", 1)
#endif
    {}
  void convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer);
  enum
  {
    PRODUCT_CODE = 6805006
  };
private:
  uint32_t last_pressure_time_;
#ifndef WG0X_STANDALONE
  misc_utils::RealtimePublisher<ethercat_hardware::PressureState> publisher_;
#endif
};

#endif /* WG0X_H */
