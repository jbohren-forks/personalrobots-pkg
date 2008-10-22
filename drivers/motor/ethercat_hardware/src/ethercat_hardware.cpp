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

#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_dll.h>

#include <ros/node.h>

EthercatHardware::EthercatHardware() :
  hw_(0), ni_(0), current_buffer_(0), last_buffer_(0), buffer_size_(0), halt_motors_(true), reset_state_(0), publisher_("/diagnostics", 1)
{
}

EthercatHardware::~EthercatHardware()
{
  if (ni_)
  {
    close_socket(ni_);
  }
  if (buffers_)
  {
    delete buffers_;
  }
  if (hw_)
  {
    delete hw_;
  }
  publisher_.stop();
}

static const int NSEC_PER_SEC = 1e+9;

static double now()
{
  struct timespec n;
  clock_gettime(CLOCK_REALTIME, &n);
  return double(n.tv_nsec) / NSEC_PER_SEC + n.tv_sec;
}

void EthercatHardware::init(char *interface, bool allow_unprogrammed)
{
  ros::node *node = ros::node::instance();

  // Initialize network interface
  interface_ = interface;
  if ((ni_ = init_ec(interface)) == NULL)
  {
    ROS_FATAL("Unable to initialize interface: %s", interface);
    ROS_BREAK();
  }

  if (set_socket_timeout(ni_, 1000*500))
  {
    ROS_FATAL("Unable to change socket timeout");
    ROS_BREAK();
  }

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni_);
  if ((al_ = EtherCAT_AL::instance()) == NULL)
  {
    ROS_FATAL("Unable to initialize Application Layer (AL): %08x", al_);
    ROS_BREAK();
  }

  num_slaves_ = al_->get_num_slaves();
  if (num_slaves_ == 0)
  {
    ROS_FATAL("Unable to locate any slaves");
    ROS_BREAK();
  }

  if (set_socket_timeout(ni_, 1000*(num_slaves_*10 + 100)))
  {
    ROS_FATAL("Unable to change socket timeout");
    ROS_BREAK();
  }

  // Initialize Master
  if ((em_ = EtherCAT_Master::instance()) == NULL)
  {
    ROS_FATAL("Unable to initialize EtherCAT_Master: %08x", em_);
    ROS_BREAK();
  }

  slaves_ = new EthercatDevice*[num_slaves_];

  // Configure slaves
  unsigned int num_actuators = 0;
  for (unsigned int slave = 0; slave < num_slaves_; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em_->get_slave_handler(fsa);
    if (sh == NULL)
    {
      ROS_FATAL("Unable to get slave handler #%d", slave);
      ROS_BREAK();
    }

    if ((slaves_[slave] = configSlave(sh)) != NULL)
    {
      if (!sh->to_state(EC_OP_STATE))
      {
        ROS_FATAL("Unable to initialize slave #%d, product code: %d", slave, sh->get_product_code());
        ROS_BREAK();
      }
      num_actuators += slaves_[slave]->has_actuator_;
      buffer_size_ += slaves_[slave]->command_size_ + slaves_[slave]->status_size_;
    }
    else
    {
      ROS_FATAL("Unable to configure slave #%d, product code: %d", slave, sh->get_product_code());
      ROS_BREAK();
    }
  }

  // Allocate buffers to send and receive commands
  buffers_ = new unsigned char[2 * buffer_size_];
  current_buffer_ = buffers_;
  last_buffer_ = buffers_ + buffer_size_;

  // Make sure motors are disabled
  memset(current_buffer_, 0, buffer_size_);
  em_->txandrx_PD(buffer_size_, current_buffer_);

  // Create HardwareInterface
  hw_ = new HardwareInterface(num_actuators);
  hw_->current_time_ = now();

  // Initialize slaves
  for (unsigned int slave = 0, a = 0; slave < num_slaves_; ++slave)
  {
    if (slaves_[slave]->initialize(hw_->actuators_[a], allow_unprogrammed) < 0)
    {
      ROS_FATAL("Unable to initialize slave #%d", slave);
      ROS_BREAK();
    }
    a += slaves_[slave]->has_actuator_;
  }

  // Initialize diagnostic data structures
  publisher_.msg_.status.reserve(num_slaves_ + 1);
  statuses_.reserve(num_slaves_ + 1);
  strings_.reserve(5);
  values_.reserve(5);
}

void EthercatHardware::initXml(TiXmlElement *config, bool allow_override)
{
  ros::node *node = ros::node::instance();
  unsigned int a = 0, s = 0;

  for (TiXmlElement *elt = config->FirstChildElement("actuator"); elt; elt = elt->NextSiblingElement("actuator"))
  {
    while (s < num_slaves_ && !slaves_[s]->has_actuator_)
      ++s;
    if (s == num_slaves_)
    {
      ROS_FATAL("Too many actuators defined in XML file");
      ROS_BREAK();
    }

    if (allow_override)
    {
      hw_->actuators_[a]->name_ = elt->Attribute("name");
      slaves_[s]->initXml(elt);
      ++s;
    }
    else
    {
      if (hw_->actuators_[a]->name_ != elt->Attribute("name"))
      {
        ROS_FATAL("Name programmed into board ('%s') does not equal actuator name in XML file ('%s')\n", hw_->actuators_[a]->name_.c_str(), elt->Attribute("name"));
        ROS_BREAK();
      }
    }
    ++a;
  }
}

void EthercatHardware::publishDiagnostics()
{
  // Publish status of EtherCAT master
  robot_msgs::DiagnosticStatus status;
  robot_msgs::DiagnosticValue v;
  robot_msgs::DiagnosticString s;

  strings_.clear();
  values_.clear();
  statuses_.clear();

  status.name = "EtherCAT Master";
  if (halt_motors_)
  {
    status.level = 2;
    status.message = "Motors halted";
  } else {
    status.level = 0;
    status.message = "OK";
  }

  // Motors halted?
  s.value = halt_motors_ ? "true" : "false";
  s.label = "Motors halted";
  strings_.push_back(s);

  // Num devices
  v.value = num_slaves_;
  v.label = "EtherCAT devices";
  values_.push_back(v);

  // Interface
  s.value = interface_;
  s.label = "Interface";
  strings_.push_back(s);

  // Interface
  v.value = reset_state_;
  v.label = "Reset state";
  values_.push_back(v);

  // Roundtrip
  double total = 0;
  for (int i = 0; i < 1000; ++i)
  {
    total += diagnostics_.iteration_[i].roundtrip_;
    diagnostics_.max_roundtrip_ = max(diagnostics_.max_roundtrip_, diagnostics_.iteration_[i].roundtrip_);
  }
  v.value = total / 1000.0;
  v.label = "Average roundtrip time";
  values_.push_back(v);

  v.value = diagnostics_.max_roundtrip_;
  v.label = "Maximum roundtrip time";
  values_.push_back(v);

  status.set_values_vec(values_);
  status.set_strings_vec(strings_);
  statuses_.push_back(status);

  unsigned char *current = current_buffer_;
  for (unsigned int s = 0; s < num_slaves_; ++s)
  {
    slaves_[s]->diagnostics(status, current);
    statuses_.push_back(status);
    current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
  }

  // Publish status of each EtherCAT device
  if (publisher_.trylock())
  {
    publisher_.msg_.set_status_vec(statuses_);
    publisher_.unlockAndPublish();
  }
}

void EthercatHardware::update(bool reset)
{
  unsigned char *current, *last;

  static int count = 0;

  // Convert HW Interface commands to MCB-specific buffers
  current = current_buffer_;

  if (reset)
  {
    reset_state_ = num_slaves_ + 10;
    halt_motors_ = false;
  }

  for (unsigned int s = 0, a = 0; s < num_slaves_; ++s)
  {
    if (slaves_[s]->has_actuator_)
    {
      Actuator *act = hw_->actuators_[a];
      slaves_[s]->computeCurrent(act->command_);
      act->state_.last_requested_effort_ = act->command_.effort_;
      act->state_.last_requested_current_ = act->command_.current_;
      slaves_[s]->truncateCurrent(act->command_);
      // Bringup motor boards, one per tick
      if (halt_motors_ || (reset_state_ && ((s + 9) < reset_state_)))
      {
        bool tmp = act->command_.enable_;
        act->command_.enable_ = false;
        act->command_.current_ = 0;
        slaves_[s]->convertCommand(act->command_, current);
        act->command_.enable_ = tmp;
      } else {
        slaves_[s]->convertCommand(act->command_, current);
      }
      current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      ++a;
    }
  }

  // Transmit process data
  double start = now();
  if (!em_->txandrx_PD(buffer_size_, current_buffer_)) {
    printf("Doh!\n");
  }
  diagnostics_.iteration_[count].roundtrip_ = now() - start;

  // Convert status back to HW Interface
  current = current_buffer_;
  last = last_buffer_;
  for (unsigned int s = 0, a = 0; s < num_slaves_; ++s)
  {
    if (slaves_[s]->has_actuator_)
    {
      Actuator *act = hw_->actuators_[a];
      slaves_[s]->convertState(act->state_, current, last);
      // Don't halt motors during a reset
      if (!slaves_[s]->verifyState(act->state_, current, last) && !reset_state_)
        halt_motors_ = true;
      current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      last += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      ++a;
    }
  }

  if (reset_state_)
    --reset_state_;

  // Update current time
  hw_->current_time_ = now();

  unsigned char *tmp = current_buffer_;
  current_buffer_ = last_buffer_;
  last_buffer_ = tmp;

  if (++count == 1000)
  {
    publishDiagnostics();
    count = 0;
  }
}

EthercatDevice *
EthercatHardware::configSlave(EtherCAT_SlaveHandler *sh)
{
  static int startAddress = 0x00010000;

  EthercatDevice *p = NULL;
  try
  {
    p = DeviceFactory::Instance().CreateObject(sh->get_product_code());
    p = p->configure(startAddress, sh);
  }
  catch (Loki::DefaultFactoryError<unsigned int, EthercatDevice>::Exception)
  {
  }
  return p;
}
