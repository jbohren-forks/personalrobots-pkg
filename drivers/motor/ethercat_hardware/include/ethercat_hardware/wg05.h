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

#include <ethercat_hardware/motor_control_board.h>

struct WG05Status
{
  uint16_t deviceTypeId;
  uint16_t deviceRev;
  uint8_t mode;
  uint8_t digitalOut;
  uint16_t pwmDuty;
  uint16_t programmedCurrent;
  uint8_t currentLoopKp;
  uint8_t currentLoopKi;
  uint16_t measuredCurrent;
  uint16_t pad1;
  uint32_t timestamp;
  uint32_t encoderCount;
  uint32_t encoderIndexPos;
  uint16_t numEncoderErrors;
  uint8_t encoderStatus;
  uint8_t calibrationReading;
  uint32_t lastCalibrationHighTransition;
  uint32_t lastCalibrationLowTransition;
  uint16_t supplyVoltage;
  uint16_t motorVoltage;
  uint16_t boardTemperature;
  uint16_t bridgeTemperature;
  uint8_t pdoCommandIrqCount;
  uint8_t mbxCommandIrqCount;
  uint16_t packetCount;
  uint16_t pdiTimeoutErrorCount;
  uint16_t pdiChecksumErrorCount;
  uint8_t pad2;
  uint8_t checksum;
}__attribute__ ((__packed__));

typedef WG05Status WG05Command;

class WG05 : public MotorControlBoard
{
  static const int STATUS_PHY_ADDR = 0x2000;
  static const int COMMAND_PHY_ADDR = 0x1000;

  static const int Ki = 8;
  static const int Kp = 4;

  static const double CURRENT_FACTOR = 2000.0;

  enum
  {
    MODE_OFF = 0x00, MODE_CURRENT = 0x01, MODE_ENABLE = 0x02, MODE_UNDERVOLTAGE = 0x04, MODE_RESET = 0x80
  };

public:
  WG05() :
    MotorControlBoard(WG05_PRODUCT_CODE, sizeof(WG05Command), sizeof(WG05Status))
  {
  }
  void configure(int &startAddress, EtherCAT_SlaveHandler *sh);
  void convertCommand(ActuatorCommand &command, unsigned char *buffer);
  void convertState(ActuatorState &state, unsigned char *buffer);

private:
  static const EC_UDINT WG05_PRODUCT_CODE = 0x57473035;
};

#endif /* WG05_H */
