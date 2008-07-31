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

#include <ethercat_hardware/mk1001.h>

void MK1001::configure(int &startAddress, EtherCAT_SlaveHandler *sh)
{
  static unsigned int phyStatusAddress(STATUS_PHY_ADDR);
  static unsigned int phyCommandAddress(COMMAND_PHY_ADDR);

  printf("configure mk1001\n");
  EC_FMMU *statusFMMU = new EC_FMMU(startAddress, // Logical start address
                                    sizeof(MK1001Status), // Logical length
                                    0x00, // Logical StartBit
                                    0x07, // Logical EndBit
                                    STATUS_PHY_ADDR, // Physical Start address
                                    0x00, // Physical StartBit
                                    true, // Read Enable
                                    false, // Write Enable
                                    true); // Enable

  startAddress += sizeof(MK1001Status);

  EC_FMMU *commandFMMU = new EC_FMMU(startAddress, // Logical start address
                                     sizeof(MK1001Command), // Logical length
                                     0x00, // Logical StartBit
                                     0x07, // Logical EndBit
                                     COMMAND_PHY_ADDR, // Physical Start address
                                     0x00, // Physical StartBit
                                     false, // Read Enable
                                     true, // Write Enable
                                     true); // Enable

  startAddress += sizeof(MK1001Command);

  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);
  (*fmmu)[0] = *statusFMMU;
  (*fmmu)[1] = *commandFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

  // Sync manager for read data
  EC_SyncMan *statusSM = new EC_SyncMan(phyStatusAddress, sizeof(MK1001Status));
  statusSM->ChannelEnable = true;
  statusSM->ALEventEnable = true;

  EC_SyncMan *commandSM = new EC_SyncMan(phyCommandAddress, sizeof(MK1001Command), EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  commandSM->ChannelEnable = true;
  commandSM->ALEventEnable = true;

  (*pd)[0] = *commandSM;
  (*pd)[1] = *statusSM;

  sh->set_pd_config(pd);
}

void MK1001::convertCommand(ActuatorCommand &command, unsigned char *buffer)
{
  MK1001Command c;

  memset(&c, 0, sizeof(c));

  c.i_k = -5;
  c.i_i = -9;
  c.i_i_limit = 1000;
  c.shift = 4;
  c.qei_cpr = 500;
  c.config = 1;
  c.i_offset = 0;//-2000;

  c.i_desire = command.current_;
  c.mode = command.enable_ ? MODE_PWM : MODE_OFF;
  memcpy(buffer + sizeof(MK1001Status), &c, sizeof(c));
}

void MK1001::convertState(ActuatorState &state, unsigned char *buffer, unsigned char *not_used)
{
  MK1001Status s;
  MK1001Command c;

  memcpy(&s, buffer, sizeof(s));
  memcpy(&c, buffer + sizeof(s), sizeof(c));

  state.is_enabled_ = c.mode != MODE_OFF;
  state.timestamp_ = s.timestamp;
  state.encoder_count_ = s.qei_pos;
  state.encoder_velocity_ = s.qei_velocity;
  state.last_measured_current_ = s.adc_current;
}

