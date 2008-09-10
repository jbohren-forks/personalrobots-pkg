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

#include <math.h>

#include <ethercat_hardware/wg05.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

static bool reg = DeviceFactory::Instance().Register(WG05::PRODUCT_CODE, deviceCreator<WG05> );

static unsigned int rotate_right_8(unsigned in)
{
  in &= 0xff;
  in = (in >> 1) | (in << 7);
  in &= 0xff;
  return in;
}

static unsigned compute_checksum(void const *data, unsigned length)
{
  unsigned char *d = (unsigned char *)data;
  unsigned int checksum = 0x42;
  for (unsigned int i = 0; i < length; ++i)
  {
    checksum = rotate_right_8(checksum);
    checksum ^= d[i];
    checksum &= 0xff;
  }
  return checksum;
}

void WG05MbxHdr::build(uint16_t address, uint16_t length, bool write_nread)
{
  address_ = address;
  length_ = length;
  pad_ = 0;
  write_nread_ = write_nread;
  checksum_ = rotate_right_8(compute_checksum(this, sizeof(*this) - 1));
}

bool WG05MbxHdr::verify_checksum(void) const
{
  return compute_checksum(this, sizeof(*this)) != 0;
}

void WG05MbxCmd::build(unsigned address, unsigned length, bool write_nread, void const *data)
{
  this->hdr_.build(address, length, write_nread);
  if (data != NULL)
  {
    memcpy(data_, data, length);
  }
  else
  {
    memset(data_, 0, length);
  }
  unsigned checksum = rotate_right_8(compute_checksum(data_, length));
  data_[length] = checksum;
}

EthercatDevice *WG05::configure(int &startAddress, EtherCAT_SlaveHandler *sh)
{
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

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(4);

  // Sync managers
  EC_SyncMan *commandSM = new EC_SyncMan(COMMAND_PHY_ADDR, sizeof(WG05Command), EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  commandSM->ChannelEnable = true;
  commandSM->ALEventEnable = true;

  EC_SyncMan *statusSM = new EC_SyncMan(STATUS_PHY_ADDR, sizeof(WG05Status));
  statusSM->ChannelEnable = true;
  statusSM->ALEventEnable = true;

  EC_SyncMan *mbxCommandSM = new EC_SyncMan(MBX_COMMAND_PHY_ADDR, MBX_COMMAND_SIZE, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
  commandSM->ChannelEnable = true;
  commandSM->ALEventEnable = true;

  EC_SyncMan *mbxStatusSM = new EC_SyncMan(MBX_STATUS_PHY_ADDR, MBX_STATUS_SIZE, EC_QUEUED);
  commandSM->ChannelEnable = true;
  commandSM->ALEventEnable = true;

  (*pd)[0] = *commandSM;
  (*pd)[1] = *statusSM;
  (*pd)[2] = *mbxCommandSM;
  (*pd)[3] = *mbxStatusSM;

  sh->set_pd_config(pd);
/*
  if (mailboxRead(sh, WG05ConfigInfo::CONFIG_INFO_BASE_ADDR, &config_info_, sizeof(config_info_)) != 0)
  {
    return NULL;
  }*/
  config_info_.nominal_current_scale_ = 1.0 / 2000.;
  max_current_ = 1.5;
  return this;
}

static const double motor_torque_constant = -0.0603;
static const int pulses_per_revolution = 1200;

void WG05::convertCommand(ActuatorCommand &command, unsigned char *buffer)
{
  WG05Command c;

  memset(&c, 0, sizeof(c));

  double current = command.effort_ / motor_torque_constant;
  current = max(min(current, max_current_), -max_current_);

  c.programmed_current_ = int(current / config_info_.nominal_current_scale_);
  c.mode_ = command.enable_ ? (MODE_ENABLE | MODE_CURRENT) : MODE_OFF;
  c.checksum_ = rotate_right_8(compute_checksum(&c, sizeof(c) - 1));

  memcpy(buffer + sizeof(WG05Status), &c, sizeof(c));
}

void WG05::truncateCurrent(ActuatorCommand &command)
{
  //command.current_ = max(min(command.current_, max_current_), -max_current_);
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
  state.position_ = double(current_status.encoder_count_) / pulses_per_revolution * 2 * M_PI - state.zero_offset_;
  state.encoder_velocity_ = double(int(current_status.encoder_count_ - last_status.encoder_count_))
      / (current_status.timestamp_ - last_status.timestamp_) * 1e+6;
  state.velocity_ = state.encoder_velocity_ / pulses_per_revolution * 2 * M_PI;
  state.calibration_reading_ = current_status.calibration_reading_ & LIMIT_SENSOR_0_STATE;
  state.last_calibration_high_transition_ = double(current_status.last_calibration_high_transition_) / pulses_per_revolution;
  state.last_calibration_low_transition_ = double(current_status.last_calibration_low_transition_) / pulses_per_revolution;
  state.is_enabled_ = current_status.mode_ != MODE_OFF;
  state.run_stop_hit_ = (current_status.mode_ & MODE_UNDERVOLTAGE) != 0;

  state.last_commanded_effort_ = current_status.programmed_current_ * config_info_.nominal_current_scale_ * motor_torque_constant;
  state.last_measured_effort_ = current_status.measured_current_ * config_info_.nominal_current_scale_ * motor_torque_constant;

  state.num_encoder_errors_ = current_status.num_encoder_errors_;
  state.num_communication_errors_ = 0; // TODO: communication errors are no longer reported in the process data

  state.motor_voltage_ = int(current_status.motor_voltage_ * config_info_.nominal_voltage_scale_);
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
  if(voltage_error> 5)
  { //Arbitary threshold
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
  if(current_error> threshold);
  //complain and shut down

  //TODO: filter errors so that one-frame spikes don't shut down the system.
#endif
}

int WG05::readData(EtherCAT_SlaveHandler *sh, EC_UINT address, void* buffer, EC_UINT length)
{
  unsigned char *p = (unsigned char *)buffer;
  EtherCAT_DataLinkLayer *dll = EtherCAT_DataLinkLayer::instance();
  EC_Logic *logic = EC_Logic::instance();

  // Build read telegram, use slave position
  APRD_Telegram status(logic->get_idx(), // Index
                       -sh->get_ring_position(), // Slave position on ethercat chain (auto increment address)
                       address, // ESC physical memory address (start address)
                       logic->get_wkc(), // Working counter
                       length, // Data Length,
                       p); // Buffer to put read result into

  // Put read telegram in ethercat/ethernet frame
  EC_Ethernet_Frame frame(&status);

  // Send/Recv data from slave
  if (!dll->txandrx(&frame))
  {
    status.set_wkc(logic->get_wkc());
    status.set_idx(logic->get_idx());
    if (!dll->txandrx(&frame))
    {
      return -1;
    }
  }

  // In some cases (clearing status mailbox) this is expected to occur
  if (status.get_wkc() != 1)
  {
    return -2;
  }

  return 0;
}

// Writes <length> amount of data from ethercat slave <sh_hub> from physical address <address> to <buffer>
int WG05::writeData(EtherCAT_SlaveHandler *sh, EC_UINT address, void const* buffer, EC_UINT length)
{
  unsigned char *p = (unsigned char *)buffer;
  EtherCAT_DataLinkLayer *m_dll_instance = EtherCAT_DataLinkLayer::instance();
  EC_Logic *m_logic_instance = EC_Logic::instance();

  // Build write telegram, use slave position
  APWR_Telegram command(m_logic_instance->get_idx(), // Index
                        -sh->get_ring_position(), // Slave position on ethercat chain (auto increment address) (
                        address, // ESC physical memory address (start address)
                        m_logic_instance->get_wkc(), // Working counter
                        length, // Data Length,
                        p); // Buffer to put read result into

  // Put read telegram in ethercat/ethernet frame
  EC_Ethernet_Frame frame(&command);

  // Send/Recv data from slave
  if (!m_dll_instance->txandrx(&frame))
  {
    command.set_wkc(m_logic_instance->get_wkc());
    command.set_idx(m_logic_instance->get_idx());
    if (!m_dll_instance->txandrx(&frame))
    {
      return -1;
    }
  }

  if (command.get_wkc() != 1)
  {
    return -2;
  }

  return 0;
}

int WG05::mailboxRead(EtherCAT_SlaveHandler *sh, int address, void *data, int length)
{
  // first (re)read current status mailbox data to prevent issues with
  // the status mailbox being full (and unread) from last command
  WG05MbxCmd stat;
  int result = readData(sh, MBX_STATUS_PHY_ADDR, &stat, sizeof(stat));
  if ((result != 0) && (result != -2))
  {
    fprintf(stderr, "CLEARING STATUS MBX FAILED result = %d\n", result);
    return -1;
  }

  // Build mailbox message and send read command
  WG05MbxCmd cmd;
  cmd.build(address, length, false /*read*/, data);
  int tries;
  for (tries = 0; tries < 10; ++tries)
  {
    int result = writeData(sh, MBX_COMMAND_PHY_ADDR, &cmd, sizeof(cmd));
    if (result == -2)
    {
      // FPGA hasn't written responded with status data, wait a
      // tiny bit and try again.
      usleep(100); // 1/10th of a millisecond
      continue;
    }
    else if (result == 0)
    {
      // Successful read of status data
      break;
    }
    else
    {
      fprintf(stderr, "WRITING COMMAND MBX FAILED\n");
      return -1;
    }
  }
  if (tries >= 10)
  {
    fprintf(stderr, "do_mailbox_write : Too many tries writing mailbox\n");
    return -1;
  }

  for (tries = 0; tries < 10; ++tries)
  {
    int result = readData(sh, MBX_STATUS_PHY_ADDR, &stat, sizeof(stat));
    if (result == -2)
    {
      // FPGA hasn't written responded with status data, wait a
      // tiny bit and try again.
      usleep(100); // 1/10th of a millisecond
      continue;
    }
    else if (result == 0)
    {
      // Successfull read of status data
      break;
    }
    else
    {
      fprintf(stderr, "READING MBX STATUS FAILED\n");
      return -1;
    }
  }
  if (tries >= 10)
  {
    fprintf(stderr, "do_mailbox_read : Too many tries reading mailbox\n");
    return -1;
  }

  if (compute_checksum(&stat, length + 1) != 0)
  {
    fprintf(stderr, "CHECKSUM ERROR READING MBX DATA\n");
    return -1;
  }
  memcpy(data, &stat, length);
  return 0;
}

