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
  hw_(0), ni_(0), current_buffer_(0), last_buffer_(0), buffer_size_(0)
{
}

EthercatHardware::~EthercatHardware()
{
  if (ni_)
  {
    close_socket(ni_);
  }
  if (current_buffer_)
  {
    delete current_buffer_;
  }
  if (last_buffer_)
  {
    delete last_buffer_;
  }
  if (hw_)
  {
    delete hw_;
  }
}

void EthercatHardware::init(char *interface, TiXmlElement *configuration)
{
  // Initialize network interface
  if ((ni_ = init_ec(interface)) == NULL)
  {
    perror("init_ec");
    return;
  }

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni_);
  if ((al_ = EtherCAT_AL::instance()) == NULL)
  {
    perror("EtherCAT_AL::instance");
    return;
  }

  unsigned int num_slaves = al_->get_num_slaves();
  if (num_slaves == 0)
  {
    perror("Can't locate any slaves");
    return;
  }

  // Initialize Master
  if ((em_ = EtherCAT_Master::instance()) == NULL)
  {
    perror("EtherCAT_Master::instance");
    return;
  }

  slaves = new MotorControlBoard*[num_slaves];

  unsigned int num_actuators = 0;
  for (unsigned int slave = 0; slave < num_slaves; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em_->get_slave_handler(fsa);
    if (sh == NULL)
    {
      perror("get_slave_handler");
      return;
    }

    if ((slaves[slave] = configSlave(sh)) != NULL)
    {
      num_actuators += slaves[slave]->hasActuator();
      buffer_size_ += slaves[slave]->commandSize + slaves[slave]->statusSize;
      if (!sh->to_state(EC_OP_STATE))
      {
        perror("to_state");
      }
    }
  }
  buffers_ = new unsigned char[2 * buffer_size_];
  current_buffer_ = buffers_;
  last_buffer_ = buffers_ + buffer_size_;

  // Determine configuration from XML file 'configuration'
  // Create HardwareInterface
  hw_ = new HardwareInterface(num_actuators);
}

void EthercatHardware::update()
{
  unsigned char *current, *last;

  // Convert HW Interface commands to MCB-specific buffers
  current = current_buffer_;
  for (int i = 0; i < hw_->num_actuators_; ++i)
  {
    slaves[i]->convertCommand(hw_->actuators_[i].command_, current);
    current += slaves[i]->commandSize + slaves[i]->statusSize;
  }

  // Transmit process data
  em_->txandrx_PD(buffer_size_, current_buffer_);

  // Convert status back to HW Interface
  current = current_buffer_;
  last = last_buffer_;
  for (int i = 0; i < hw_->num_actuators_; ++i)
  {
    slaves[i]->convertState(hw_->actuators_[i].state_, current, last);
    current += slaves[i]->commandSize + slaves[i]->statusSize;
    last += slaves[i]->commandSize + slaves[i]->statusSize;
  }

  unsigned char *tmp = current_buffer_;
  current_buffer_ = last_buffer_;
  last_buffer_ = tmp;
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
