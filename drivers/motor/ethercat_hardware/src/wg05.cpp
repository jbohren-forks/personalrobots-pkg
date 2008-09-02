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

#include <ethercat_hardware/wg05.h>

static bool reg = DeviceFactory::Instance().Register(WG05::PRODUCT_CODE, deviceCreator<WG05>);

static unsigned int rotate_right_8(unsigned in)
{
  in &= 0xff;
  in = (in >> 1) | (in << 7);
  in &= 0xff;
  return in;
}

static unsigned compute_checksum(void const *data, unsigned length)
{
  unsigned char *d = (unsigned char *) data;
  unsigned int checksum = 0x42;
  for (unsigned int i = 0; i < length; ++i)
  {
    checksum = rotate_right_8(checksum);
    checksum ^= d[i];
    checksum &= 0xff;
  }
  return checksum;
}

void WG05::configure(int &startAddress, EtherCAT_SlaveHandler *sh)
{
  static unsigned int phyStatusAddress(STATUS_PHY_ADDR);
  static unsigned int phyCommandAddress(COMMAND_PHY_ADDR);

  printf("configure WG05\n");
  EC_FMMU *statusFMMU = new EC_FMMU(startAddress, // Logical start address
                                    sizeof(WG05Status), // Logical length
                                    0x00, // Logical StartBit
                                    0x07, // Logical EndBit
                                    STATUS_PHY_ADDR, // Physical Start address
                                    0x00, // Physical StartBit
                                    true, // Read Enable
                                    false, // Write Enable
                                    true); // Enable

  startAddress += sizeof(WG05Status);

  EC_FMMU *commandFMMU = new EC_FMMU(startAddress, // Logical start address
                                     sizeof(WG05Command),// Logical length
                                     0x00, // Logical StartBit
                                     0x07, // Logical EndBit
                                     COMMAND_PHY_ADDR, // Physical Start address
                                     0x00, // Physical StartBit
                                     false, // Read Enable
                                     true, // Write Enable
                                     true); // Enable

  startAddress += sizeof(WG05Command);

  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);
  (*fmmu)[0] = *statusFMMU;
  (*fmmu)[1] = *commandFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

  // Sync manager for read data
  EC_SyncMan *statusSM = new EC_SyncMan(phyStatusAddress, sizeof(WG05Status));
  statusSM->ChannelEnable = true;
  statusSM->ALEventEnable = true;

  EC_SyncMan *commandSM = new EC_SyncMan(phyCommandAddress, sizeof(WG05Command), EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  commandSM->ChannelEnable = true;
  commandSM->ALEventEnable = true;

  (*pd)[0] = *commandSM;
  (*pd)[1] = *statusSM;

  sh->set_pd_config(pd);

  max_current_ = 1.0;
}

void WG05::convertCommand(ActuatorCommand &command, unsigned char *buffer)
{
  WG05Command c;

  memset(&c, 0, sizeof(c));

  c.programmed_current_ = CURRENT_FACTOR * command.current_;
  c.mode_ = command.enable_ ? (MODE_ENABLE | MODE_CURRENT) : MODE_OFF;
  c.checksum_ = rotate_right_8(compute_checksum(&c, sizeof(c) - 1));

  memcpy(buffer + sizeof(WG05Status), &c, sizeof(c));
}

void WG05::truncateCurrent(ActuatorCommand &command)
{
  if (command.current_ > max_current_)
  {
    command.current_ = max_current_;
  }
  else if (command.current_ < -max_current_)
  {
    command.current_ = -max_current_;
  }
}

void WG05::convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer)
{
  WG05Status current_status, last_status;
  WG05Command current_command, last_command;

  memcpy(&current_status, current_buffer, sizeof(current_status));
  memcpy(&current_command, current_buffer + sizeof(current_status), sizeof(current_command));

  memcpy(&last_status, last_buffer, sizeof(last_status));
  memcpy(&last_command, last_buffer + sizeof(last_status), sizeof(last_command));

  state.timestamp_ = current_status.timestamp_ / 1e+6;
  state.encoder_count_ = current_status.encoder_count_;
  state.encoder_velocity_ = double(int(current_status.encoder_count_ - last_status.encoder_count_)) / (current_status.timestamp_ - last_status.timestamp_) * 1e+6;
  state.calibration_reading_ = current_status.calibration_reading_ & LIMIT_SENSOR_0_STATE;
  state.last_calibration_high_transition_ = current_status.last_calibration_high_transition_;
  state.last_calibration_low_transition_ = current_status.last_calibration_low_transition_;
  state.is_enabled_ = current_status.mode_ != MODE_OFF;
  state.run_stop_hit_ = (current_status.mode_ & MODE_UNDERVOLTAGE) != 0;

  state.last_commanded_current_ = current_status.programmed_current_ / CURRENT_FACTOR;
  state.last_measured_current_ = current_status.measured_current_ / CURRENT_FACTOR;

  state.num_encoder_errors_ = current_status.num_encoder_errors_;
  state.num_communication_errors_ = 0; // TODO: communication errors are no longer reported in the process data

  state.motor_voltage_ = current_status.motor_voltage_;
}

void WG05::verifyState(unsigned char *buffer)
{
#if 0
  WG05Status status;

  memcpy(&status, buffer, sizeof(status));

  // Check board shutdown status
  // Report temperature shutdown, UV lockout, etc.
  // Report lots of diagnostics information

  // Check back-EMF consistency
  expected_voltage = status.measured_current_ * resistance + motor_velocity * backemf_constant_;
  voltage_error = fabs(expected_voltage - status.motor_voltage_); //Scaled to volts
  if(voltage_error > 5){ //Arbitary threshold
    //Something is wrong with the encoder, the motor, or the motor board
    //Disable motors
    //Try to diagnose further
    //motor_velocity == 0 -> encoder failure likely
    //measured_current_ ~= 0 -> motor open-circuit likely
    //motor_voltage_ ~= 0 -> motor short-circuit likely
    //else -> current-sense failure likely
    //Print error messages
  }

 //Check current-loop performance
 double current_error = status.measured_current_ - status.last_commanded_current;
 if(current_error > threshold);
   //complain and shut down

 //TODO: filter errors so that one-frame spikes don't shut down the system.
#endif
}
