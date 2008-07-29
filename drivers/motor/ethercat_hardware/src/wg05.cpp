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
}

void WG05::convertCommand(ActuatorCommand &command, unsigned char *buffer)
{
  WG05Command c;

  memset(&c, 0, sizeof(c));

  c.currentLoopKi = Ki;

  c.programmedCurrent = CURRENT_FACTOR * command.current;
  c.mode = command.enable ? (MODE_ENABLE | MODE_CURRENT) : MODE_OFF;

  memcpy(buffer + sizeof(WG05Status), &c, sizeof(c));
}

void WG05::convertState(ActuatorState &state, unsigned char *current_buffer, unsigned char *last_buffer)
{
  WG05Status current_status, last_status;
  WG05Command current_command, last_command;

  memcpy(&current_status, current_buffer, sizeof(current_status));
  memcpy(&current_command, current_buffer + sizeof(current_status), sizeof(current_command));

  memcpy(&last_status, last_buffer, sizeof(last_status));
  memcpy(&last_command, last_buffer + sizeof(last_status), sizeof(last_command));

  state.encoderCount = current_status.encoderCount;
  state.encoderVelocity = double(int(current_status.encoderCount - last_status.encoderCount)) / (current_status.timestamp - last_status.timestamp) * 1e+6;
  state.calibrationReading = current_status.calibrationReading;
  state.lastCalibrationHighTransition = current_status.lastCalibrationHighTransition;
  state.lastCalibrationLowTransition = current_status.lastCalibrationLowTransition;
  state.isEnabled = current_status.mode != MODE_OFF;
  state.runStopHit = (current_status.mode & MODE_UNDERVOLTAGE) != 0;

  state.lastRequestedCurrent = current_command.programmedCurrent / CURRENT_FACTOR; // Should be actual request, before safety
  state.lastCommandedCurrent = current_status.programmedCurrent / CURRENT_FACTOR;
  state.lastMeasuredCurrent = current_status.measuredCurrent / CURRENT_FACTOR;

  state.numEncoderErrors = current_status.numEncoderErrors;
  state.numCommunicationErrors = current_status.pdiTimeoutErrorCount + current_status.pdiChecksumErrorCount;

  state.motorVoltage = current_status.motorVoltage;
#if 1
{
static int times = 0;

if (++times % 2000 == 0) {
  printf("-----------------------------------------------------\n");
  printf("timestamp = %#08x\n", current_status.timestamp);
  printf("encoderCount  = %#08x\n", current_status.encoderCount);
  printf("last timestamp = %#08x\n", last_status.timestamp);
  printf("last encoderCount  = %#08x\n", last_status.encoderCount);
  printf("delta encoderCount  = %#08x %f\n", current_status.encoderCount - last_status.encoderCount, double(int(current_status.encoderCount - last_status.encoderCount)));
  printf("delta timestamp  = %#08x %f\n", current_status.timestamp - last_status.timestamp, double(current_status.timestamp - last_status.timestamp));
  printf("encoderVelocity  = %f\n", state.encoderVelocity);
  printf("programmedCurrent = %#08x\n", current_status.programmedCurrent);
  printf("measuredCurrent = %#08x\n", current_status.measuredCurrent);
  printf("mode = %#08x\n", current_status.mode);
  printf("Kp = %#08x\n", int(current_status.currentLoopKp));
  printf("Ki = %#08x\n", int(current_status.currentLoopKi));
}
}
#endif

}


