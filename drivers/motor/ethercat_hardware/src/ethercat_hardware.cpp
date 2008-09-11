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
  hw_(0), ni_(0), current_buffer_(0), last_buffer_(0), buffer_size_(0), publisher_("/diagnostics", 1)
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

void EthercatHardware::init(char *interface)
{
  ros::node *node = ros::node::instance();

  // Initialize network interface
  interface_ = interface;
  if ((ni_ = init_ec(interface)) == NULL)
  {
    node->log(ros::FATAL, "Unable to initialize interface: %s", interface);
  }

  // Initialize Application Layer (AL)
  EtherCAT_DataLinkLayer::instance()->attach(ni_);
  if ((al_ = EtherCAT_AL::instance()) == NULL)
  {
    node->log(ros::FATAL, "Unable to initialize Application Layer (AL): %08x", al_);
  }

  num_slaves_ = al_->get_num_slaves();
  if (num_slaves_ == 0)
  {
    node->log(ros::FATAL, "Unable to locate any slaves");
  }

  // Initialize Master
  if ((em_ = EtherCAT_Master::instance()) == NULL)
  {
    node->log(ros::FATAL, "Unable to initialize EtherCAT_Master: %08x", em_);
  }

  slaves_ = new EthercatDevice*[num_slaves_];

  unsigned int num_actuators = 0;
  for (unsigned int slave = 0; slave < num_slaves_; ++slave)
  {
    EC_FixedStationAddress fsa(slave + 1);
    EtherCAT_SlaveHandler *sh = em_->get_slave_handler(fsa);
    if (sh == NULL)
    {
      node->log(ros::FATAL, "Unable to get slave handler #%d", slave);
    }

    if ((slaves_[slave] = configSlave(sh)) != NULL)
    {
      if (!sh->to_state(EC_OP_STATE))
      {
        node->log(ros::FATAL, "Unable to initialize slave #%d, product code: %d", slave, sh->get_product_code());
      }
      num_actuators += slaves_[slave]->has_actuator_;
      buffer_size_ += slaves_[slave]->command_size_ + slaves_[slave]->status_size_;
    }
    else
    {
      node->log(ros::FATAL, "Unable to configure slave #%d, product code: %d", slave, sh->get_product_code());
    }
  }

  buffers_ = new unsigned char[2 * buffer_size_];
  current_buffer_ = buffers_;
  last_buffer_ = buffers_ + buffer_size_;

  // Create HardwareInterface
  hw_ = new HardwareInterface(num_actuators);
  hw_->current_time_ = now();

  for (unsigned int slave = 0, a = 0; slave < num_slaves_; ++slave)
  {
    if (slaves_[slave]->initialize(hw_->actuators_[a]) < 0)
    {
      node->log(ros::FATAL, "Unable to initialize slave #%d", slave);
    }
    a += slaves_[slave]->has_actuator_;
  }
  // Initialize diagnostic data structures
  diagnostic_message_.set_status_size(1 + num_slaves_);
  diagnostic_message_.status[0].set_values_size(3);
  for (unsigned int slave = 0; slave < num_slaves_; ++slave)
  {
    diagnostic_message_.status[1 + slave].set_values_size(1);
  }
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
      node->log(ros::FATAL, "Too many actuators defined in XML file");

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
        node->log(ros::FATAL, "Name programmed into board ('%s') does not equal actuator name in XML file ('%s')\n", hw_->actuators_[a]->name_.c_str(), elt->Attribute("name"));
      }
    }
    ++a;
  }
}

void EthercatHardware::publishDiagnostics()
{
  vector<robot_msgs::DiagnosticStatus> status_vec;

  // Publish status of EtherCAT master
  {
    robot_msgs::DiagnosticStatus *master = diagnostic_message_.status;
    master->level = 0;
    master->name = "EtherCAT Master";
    master->message = "OK";

    robot_msgs::DiagnosticValue *value = master->values;

    // Num devices
    value->value = num_slaves_;
    value->value_label = "EtherCAT devices";
    ++value;

    // Roundtrip
    double total = 0;
    for (int i = 0; i < 1000; ++i)
    {
      total += diagnostics_.iteration_[i].roundtrip_;
      diagnostics_.max_roundtrip_ = max(diagnostics_.max_roundtrip_, diagnostics_.iteration_[i].roundtrip_);
    }
    value->value = total / 1000.0;
    value->value_label = "Average roundtrip time";
    ++value;

    value->value = diagnostics_.max_roundtrip_;
    value->value_label = "Maximum roundtrip time";
    ++value;
  }

  for (unsigned int slave = 0; slave < num_slaves_; ++slave)
  {
    robot_msgs::DiagnosticStatus *device = diagnostic_message_.status + 1 + slave;
    robot_msgs::DiagnosticValue *value = device->values;

    stringstream str;
    str << "EtherCAT Device #" << slave;
    device->name = str.str();

    device->message = "OK";
    device->level = 0;

    value->value = 10;
    value->value_label = "Temperature";
    value++;
  }

  // Publish status of each EtherCAT device
  publisher_.publish(diagnostic_message_);
}

void EthercatHardware::update()
{
  unsigned char *current, *last;

  static int count = 0;

  // Convert HW Interface commands to MCB-specific buffers
  current = current_buffer_;
  for (unsigned int s = 0, a = 0; s < num_slaves_; ++s)
  {
    if (slaves_[s]->has_actuator_)
    {
      hw_->actuators_[a]->state_.last_requested_effort_ = hw_->actuators_[a]->command_.effort_;
      slaves_[s]->truncateCurrent(hw_->actuators_[a]->command_);
      slaves_[s]->convertCommand(hw_->actuators_[a]->command_, current);
      current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      ++a;
    }
  }

  // Transmit process data
  double start = now();
  em_->txandrx_PD(buffer_size_, current_buffer_);
  diagnostics_.iteration_[count].roundtrip_ = now() - start;

  // Convert status back to HW Interface
  current = current_buffer_;
  last = last_buffer_;
  for (unsigned int s = 0, a = 0; s < num_slaves_; ++s)
  {
    if (slaves_[s]->has_actuator_)
    {
      slaves_[s]->verifyState(current);
      slaves_[s]->convertState(hw_->actuators_[a]->state_, current, last);
      current += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      last += slaves_[s]->command_size_ + slaves_[s]->status_size_;
      ++a;
    }
  }

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
