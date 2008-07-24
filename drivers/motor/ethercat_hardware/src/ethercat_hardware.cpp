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

#include "ethercat_hardware/ethercat_hardware.h"
#include "ethercat_hardware/motor_control_board.h"

#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_dll.h>

EthercatHardware::EthercatHardware() :
  hw(0), ni(0), buffer(0), bufferSize(0)
{
}

EthercatHardware::~EthercatHardware()
{
  if (ni)
  {
    close_socket(ni);
  }
  if (buffer)
  {
    delete buffer;
  }
  if (hw)
  {
    delete hw;
  }
}

void EthercatHardware::init(char *interface, char *configuration)
{
  // Determine configuration from XML file 'configuration'
  // Create HardwareInterface

  // XXXwheeler: replace with XML parser
  hw = new HardwareInterface(5);

  hw->actuator[0].command.enable = true;
  hw->actuator[0].command.current = 0;
  hw->actuator[1].command.enable = true;
  hw->actuator[1].command.current = 0.0;
  hw->actuator[2].command.enable = true;
  hw->actuator[2].command.current = 0.0;
  hw->actuator[3].command.enable = true;
  hw->actuator[3].command.current = 0;
  hw->actuator[4].command.enable = true;
  hw->actuator[4].command.current = 0;

  // Initialize network interface
  if ((ni = init_ec(interface)) == NULL)
  {
    perror("init_ec");
    return;
  }

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni);
  if ((al = EtherCAT_AL::instance()) == NULL)
  {
    perror("EtherCAT_AL::instance");
    return;
  }

  unsigned int numSlaves = al->get_num_slaves();
  if (numSlaves == 0)
  {
    perror("Can't locate any slaves");
    return;
  }

  // Initialize Master
  if ((em = EtherCAT_Master::instance()) == NULL)
  {
    perror("EtherCAT_Master::instance");
    return;
  }

  slaves = new MotorControlBoard*[numSlaves];

  for (unsigned int slave = 0; slave < numSlaves; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em->get_slave_handler(fsa);
    if (sh == NULL)
    {
      perror("get_slave_handler");
      return;
    }

    if ((slaves[slave] = configSlave(sh)) != NULL)
    {
      bufferSize += slaves[slave]->commandSize + slaves[slave]->statusSize;
      if (!sh->to_state(EC_OP_STATE))
      {
        perror("to_state");
      }
    }
  }
  buffer = new unsigned char[bufferSize];
}

void EthercatHardware::update()
{
  unsigned char *p;

  // Convert HW Interface commands to MCB-specific buffers
  p = buffer;
  for (int i = 0; i < hw->numActuators; ++i)
  {
    slaves[i]->convertCommand(hw->actuator[i].command, p);
    p += slaves[i]->commandSize + slaves[i]->statusSize;
  }

  // Transmit process data
  em->txandrx_PD(bufferSize, buffer);

  // Convert status back to HW Interface
  p = buffer;
  for (int i = 0; i < hw->numActuators; ++i)
  {
    slaves[i]->convertState(hw->actuator[i].state, p);
    p += slaves[i]->commandSize + slaves[i]->statusSize;

  }
}

MotorControlBoard *
EthercatHardware::configSlave(EtherCAT_SlaveHandler *sh)
{
  static int startAddress = 0x00010000;

  vector<MotorControlBoard *>::iterator it;
  for (it = boards.begin(); it != boards.end(); ++it)
  {
    if (sh->get_product_code() == (*it)->productCode)
    {
      (*it)->configure(startAddress, sh);
      return *it;
    }
  }
  return NULL;
}
