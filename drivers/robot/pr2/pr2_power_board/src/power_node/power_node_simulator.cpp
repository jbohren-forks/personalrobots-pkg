/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <boost/thread/thread.hpp>

// Internet/Socket stuff
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "power_comm.h"
#include "power_node.h"
#include "robot_msgs/DiagnosticMessage.h"
#include "rosconsole/macros_generated.h"
#include "ros/console.h"

using namespace std;

// Keep a pointer to the last message recieved for
// Each board.
static std::vector<Device*> Devices;
static PowerBoard *myBoard;

void processSentMessage(CommandMessage *cmd);

int PowerBoard::send_command(int selected_device, int circuit_breaker, const std::string &command, unsigned flags)
{
  if (Devices.size() == 0) {
    fprintf(stderr,"No devices to send command to\n");
    return -1;
  }

  if ((selected_device < 0) || (selected_device >= (int)Devices.size())) {
    fprintf(stderr, "Device number must be between 0 and %u\n", Devices.size()-1);
    return -1;
  }

  Device* device = Devices[selected_device];
  assert(device != NULL);


  if ((circuit_breaker < 0) || (circuit_breaker > 2)) {
    fprintf(stderr, "Circuit breaker number must be between 0 and 2\n");
    return -1;
  }

  ROS_DEBUG("circuit=%d command=%s flags=%x\n", circuit_breaker, command.c_str(), flags);

  // Determine what command to send
  char command_enum = NONE;
  if (command == "start") {
    command_enum = COMMAND_START;
  }
  else if (command ==  "stop") {
    command_enum = COMMAND_STOP;
  }
  else if (command == "reset") {
    command_enum = COMMAND_RESET;
  }
  else if (command == "disable") {
    command_enum = COMMAND_DISABLE;
  }
  else if (command == "none") {
    command_enum = NONE;
  }
  else if (command == "terrible_hack_shutdown") {
    exit(0);
  }
  else {
    ROS_ERROR("invalid command '%s'", command.c_str());
    return -1;
  }
  //" -c : command to send to device : 'start', 'stop', 'reset', 'disable'\n"


  // Build command message
  CommandMessage cmdmsg;
  memset(&cmdmsg, 0, sizeof(cmdmsg));
  cmdmsg.header.message_revision = CURRENT_MESSAGE_REVISION;
  cmdmsg.header.message_id = MESSAGE_ID_COMMAND;
  cmdmsg.header.serial_num = device->pmsg.header.serial_num;
  //cmdmsg.header.serial_num = 0x12345678;
  strncpy(cmdmsg.header.text, "power command message", sizeof(cmdmsg.header.text));
  cmdmsg.command.CB0_command = NONE;
  cmdmsg.command.CB1_command = NONE;
  cmdmsg.command.CB2_command = NONE;
  if (circuit_breaker==0) {
    cmdmsg.command.CB0_command = command_enum;
  }
  else if (circuit_breaker==1) {
    cmdmsg.command.CB1_command = command_enum;
  }
  else if (circuit_breaker==2) {
    cmdmsg.command.CB2_command = command_enum;
  }
  else if (circuit_breaker==-1) {
    cmdmsg.command.CB0_command = command_enum;
    cmdmsg.command.CB1_command = command_enum;
    cmdmsg.command.CB2_command = command_enum;
  }

  cmdmsg.command.flags = flags;

  errno = 0;
  
  processSentMessage(&cmdmsg);

  ROS_DEBUG("Send to Serial=%u, revision=%u", cmdmsg.header.serial_num, cmdmsg.header.message_revision);
  ROS_DEBUG("Sent command %s(%d) to device %d, circuit %d",
         command.c_str(), command_enum, selected_device, circuit_breaker);

  return 0;
}


const char* PowerBoard::cb_state_to_str(char state)
{
  //enum CB_State { STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED };
  switch(state) {
  case STATE_NOPOWER:
    return "no-power";
  case STATE_STANDBY:
    return "Standby";
  case STATE_PUMPING:
    return "pumping";
  case STATE_ON:
    return "On";
  case STATE_DISABLED:
    return "Disabled";
  }
  return "???";
}

const char* PowerBoard::master_state_to_str(char state)
{
  //enum CB_State { STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED };
  switch(state) {
  case MASTER_NOPOWER:
    return "no-power";
  case MASTER_STANDBY:
    return "stand-by";
  case MASTER_ON:
    return "on";
  case MASTER_OFF:
    return "off";
  }
  return "???";
}


// Determine if a record of the device already exists...
// If it does use newest message a fill in pointer to old one .
// If it does not.. use
int PowerBoard::process_message(const PowerMessage *msg)
{
  if (msg->header.message_revision != CURRENT_MESSAGE_REVISION) {
    ROS_WARN("Got message with incorrect message revision %u\n", msg->header.message_revision);
    return -1;
  }

  // Look for device serial number in list of devices...
  for (unsigned i = 0; i<Devices.size(); ++i) {
    if (Devices[i]->pmsg.header.serial_num == msg->header.serial_num) {
      boost::mutex::scoped_lock(library_lock_);
      Devices[i]->message_time = ros::Time::now();
      memcpy(&(Devices[i]->pmsg), msg, sizeof(PowerMessage));
      return 0;
    }
  }

  // Add new device to list
  Device *newDevice = new Device();
  Devices.push_back(newDevice);
  newDevice->message_time = ros::Time::now();
  memcpy(&(newDevice->pmsg), msg, sizeof(PowerMessage));
  return 0;
}

int PowerBoard::process_transition_message(const TransitionMessage *msg)
{
  if (msg->header.message_revision != CURRENT_MESSAGE_REVISION) {
    ROS_WARN("Got message with incorrect message revision %u\n", msg->header.message_revision);
    return -1;
  }

  // Look for device serial number in list of devices...
  for (unsigned i = 0; i<Devices.size(); ++i) {
    if (Devices[i]->pmsg.header.serial_num == msg->header.serial_num) {
      boost::mutex::scoped_lock(library_lock_);
      memcpy(&(Devices[i]->tmsg), msg, sizeof(TransitionMessage));
      return 0;
    }
  }

  // Add new device to list
  Device *newDevice = new Device();
  Devices.push_back(newDevice);
  memcpy(&(newDevice->tmsg), msg, sizeof(TransitionMessage));
  return 0;
}

PowerBoard::PowerBoard(): ros::Node ("pr2_power_board")
{

  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  if( my_logger->getLevel() == 0 )    //has anyone set our level??
  {
    // Set the ROS logger
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  }

  advertiseService("power_board_control", &PowerBoard::commandCallback);
  advertise<robot_msgs::DiagnosticMessage>("/diagnostics", 2);
}

bool PowerBoard::commandCallback(pr2_power_board::PowerBoardCommand::Request &req_,
                     pr2_power_board::PowerBoardCommand::Response &res_)
{
  res_.retval = send_command( 0, req_.breaker_number, req_.command, req_.flags);

  return true;
}

void PowerBoard::sendDiagnostic()
{
  robot_msgs::DiagnosticMessage msg_out;
  robot_msgs::DiagnosticStatus stat;
  robot_msgs::DiagnosticValue val;
  robot_msgs::DiagnosticString strval;

  while(ok())
  {
    ros::Duration(1,0).sleep();
    //ROS_DEBUG("-");
    boost::mutex::scoped_lock(library_lock_);
  
    ros::Time new_diagnostic_time = ros::Time::now();

    for (unsigned i = 0; i<Devices.size(); ++i)
    {
      msg_out.status.clear();
      stat.values.clear();
      stat.strings.clear();

      Device *device = Devices[i];
      PowerMessage *pmsg = &device->pmsg;

      // Stop sending diagnostics if we stop getting packets
      ROS_DEBUG("message_time: %f last_diagnostic_time: %f new_diagnostic_time: %f", 
          device->message_time.toSec(), last_diagnostic_time.toSec(),
          new_diagnostic_time.toSec());
      if (device->message_time < last_diagnostic_time)
        break;

      ostringstream ss;
      ss << "Power board " << i;
      stat.name = ss.str();
      stat.level = 0;///@todo fixem
      stat.message = "Power Node";
      StatusStruct *status = &Devices[i]->pmsg.status;

      ROS_DEBUG("Device %u", i);
      ROS_DEBUG(" Serial       = %u", pmsg->header.serial_num);

      //val.label = "Time";
      //val.value = (float)Devices[i]->message_time;
      //stat.values.push_back(val);

      ss.str("");
      ss << pmsg->header.serial_num;
      strval.value = ss.str();
      strval.label = "Serial Number";
      stat.strings.push_back(strval);

      ROS_DEBUG(" Current      = %f", status->input_current);
      val.value = status->input_current;
      val.label = "Input Current";
      stat.values.push_back(val);

      ROS_DEBUG(" Voltages:");
      ROS_DEBUG("  Input       = %f", status->input_voltage);
      val.value = status->input_voltage;
      val.label = "Input Voltage";
      stat.values.push_back(val);
      ROS_DEBUG("  DCDC 12     = %f", status->DCDC_12V_out_voltage);
      val.value = status->DCDC_12V_out_voltage;
      val.label = "DCDC12";
      stat.values.push_back(val);
      ROS_DEBUG("  DCDC 15     = %f", status->DCDC_19V_out_voltage);
      val.value = status->DCDC_19V_out_voltage;
      val.label = "DCDC 15";
      stat.values.push_back(val);
      ROS_DEBUG("  CB0 (Base)  = %f", status->CB0_voltage);
      val.value = status->CB0_voltage;
      val.label = "Breaker 0 Voltage";
      stat.values.push_back(val);
      ROS_DEBUG("  CB1 (R-arm) = %f", status->CB1_voltage);
      val.value = status->CB1_voltage;
      val.label = "Breaker 1 Voltage";
      stat.values.push_back(val);
      ROS_DEBUG("  CB2 (L-arm) = %f", status->CB2_voltage);
      val.value = status->CB2_voltage;
      val.label = "Breaker 2 Voltage";
      stat.values.push_back(val);

      ROS_DEBUG(" Board Temp   = %f", status->ambient_temp);
      val.value = status->ambient_temp;
      val.label = "Board Temperature";
      stat.values.push_back(val);
      ROS_DEBUG(" Fan Speeds:");
      ROS_DEBUG("  Fan 0       = %u", status->fan0_speed);
      val.value = status->fan0_speed;
      val.label = "Fan 0 Speed";
      stat.values.push_back(val);
      ROS_DEBUG("  Fan 1       = %u", status->fan1_speed);
      val.value = status->fan1_speed;
      val.label = "Fan 1 Speed";
      stat.values.push_back(val);
      ROS_DEBUG("  Fan 2       = %u", status->fan2_speed);
      val.value = status->fan2_speed;
      val.label = "Fan 2 Speed";
      stat.values.push_back(val);
      ROS_DEBUG("  Fan 3       = %u", status->fan3_speed);
      val.value = status->fan3_speed;
      val.label = "Fan 3 Speed";
      stat.values.push_back(val);

      ROS_DEBUG(" State:");
      ROS_DEBUG("  CB0 (Base)  = %s", cb_state_to_str(status->CB0_state));
      strval.value = cb_state_to_str(status->CB0_state);
      strval.label = "Breaker 0 State";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB1 (R-arm) = %s", cb_state_to_str(status->CB1_state));
      strval.value = cb_state_to_str(status->CB1_state);
      strval.label = "Breaker 1 State";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB2 (L-arm) = %s", cb_state_to_str(status->CB2_state));
      strval.value = cb_state_to_str(status->CB2_state);
      strval.label = "Breaker 2 State";
      stat.strings.push_back(strval);
      ROS_DEBUG("  DCDC        = %s", master_state_to_str(status->DCDC_state));
      strval.value = master_state_to_str(status->DCDC_state);
      strval.label = "DCDC state";
      stat.strings.push_back(strval);

      ROS_DEBUG(" Status:");
      ROS_DEBUG("  CB0 (Base)  = %s", (status->CB0_status) ? "On" : "Off");
      strval.value = (status->CB0_status) ? "On" : "Off";
      strval.label = "Breaker 0 Status";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB1 (R-arm) = %s", (status->CB1_status) ? "On" : "Off");
      strval.value = (status->CB1_status) ? "On" : "Off";
      strval.label = "Breaker 1 Status";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB2 (L-arm) = %s", (status->CB2_status) ? "On" : "Off");
      strval.value = (status->CB2_status) ? "On" : "Off";
      strval.label = "Breaker 2 Status";
      stat.strings.push_back(strval);
      ROS_DEBUG("  estop_button= %x", (status->estop_button_status));
      val.value = status->estop_button_status;
      val.label = "RunStop Button Status";
      stat.values.push_back(val);
      ROS_DEBUG("  estop_status= %x", (status->estop_status));
      val.value = status->estop_status;
      val.label = "RunStop Status";
      stat.values.push_back(val);

      ROS_DEBUG(" Revisions:");
      ROS_DEBUG("         PCA = %c", status->pca_rev);
      ss.str("");
      ss << status->pca_rev;
      strval.value = ss.str();
      strval.label = "PCA";
      stat.strings.push_back(strval);
      ROS_DEBUG("         PCB = %c", status->pcb_rev);
      ss.str("");
      ss << status->pcb_rev;
      strval.value = ss.str();
      strval.label = "PCB";
      stat.strings.push_back(strval);
      ROS_DEBUG("       Major = %c", status->major_rev);
      ss.str("");
      ss << status->major_rev;
      strval.value = ss.str();
      strval.label = "Major Revision";
      stat.strings.push_back(strval);
      ROS_DEBUG("       Minor = %c", status->minor_rev);
      ss.str("");
      ss << status->minor_rev;
      strval.value = ss.str();
      strval.label = "Minor Revision";
      stat.strings.push_back(strval);

      val.value = status->min_input_voltage;
      val.label = "Min Voltage";
      stat.values.push_back(val);
      val.value = status->max_input_current;
      val.label = "Max Current";
      stat.values.push_back(val);


      TransitionMessage *tmsg = &device->tmsg;
      for(int cb_index=0; cb_index < 3; ++cb_index)
      {
        TransitionCount *trans = &tmsg->cb[cb_index];
        ROS_DEBUG("Transition: CB%d", cb_index);
        val.value = trans->stop_count;
        ss.str("");
        ss << "CB" << cb_index << " Stop Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->estop_count;
        ss.str("");
        ss << "CB" << cb_index << " E-Stop Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->trip_count;
        ss.str("");
        ss << "CB" << cb_index << " Trip Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->fail_18V_count;
        ss.str("");
        ss << "CB" << cb_index << " 18V Fail Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->disable_count;
        ss.str("");
        ss << "CB" << cb_index << " Disable Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->start_count;
        ss.str("");
        ss << "CB" << cb_index << " Start Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->pump_fail_count;
        ss.str("");
        ss << "CB" << cb_index << " Pump Fail Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->reset_count;
        ss.str("");
        ss << "CB" << cb_index << " Reset Count";
        val.label = ss.str();
        stat.values.push_back(val);
      }


      msg_out.status.push_back(stat);
      robot_msgs::DiagnosticStatus stat;
      stat.name = "pr2_power_board";
      stat.level = 0;
      stat.message = "Running";
      msg_out.status.push_back(stat);

      //ROS_DEBUG("Publishing ");
      publish("/diagnostics", msg_out);
    }

    last_diagnostic_time = new_diagnostic_time;
  }
}

void sendMessages()
{
  myBoard->sendDiagnostic();
}

void CloseAllDevices(void) {
  for (unsigned i=0; i<Devices.size(); ++i){
    if (Devices[i] != NULL) {
      delete Devices[i];
    }
  }
}


// Build list of interfaces
int CreateAllInterfaces(void)
{
  return 0;
}

void generateDeviceMessages()
{
  static PowerMessage pm;
  
  pm.header.message_revision = CURRENT_MESSAGE_REVISION; //32 bit 
  pm.header.serial_num = 0xDADABABA;       //32 bit  Unique ID number
  //pm.header.text[32];         //Description identifier
  pm.header.message_id = MESSAGE_ID_POWER;
  pm.header.data_length = sizeof(pm.status);      //Length of the following structure
  
  /*pm.status.CB0_state;   // CB_State enum
  pm.status.CB1_state;
  pm.status.CB2_state;
  pm.status.DCDC_state;  // Master_State enum
  
  pm.status.input_voltage;
  pm.status.input_current;
  pm.status.DCDC_12V_out_voltage;
  pm.status.DCDC_19V_out_voltage;
  pm.status.CB0_voltage;
  pm.status.CB1_voltage;
  pm.status.CB2_voltage;
  pm.status.ambient_temp;
  pm.status.fan0_speed; 
  pm.status.fan1_speed;
  pm.status.fan2_speed;
  pm.status.fan3_speed;
  pm.status.CB0_status;  //0-off 1-on
  pm.status.CB1_status;
  pm.status.CB2_status;
  pm.status.estop_button_status;
  pm.status.estop_status;
  pm.status.pca_rev;
  pm.status.pcb_rev;
  pm.status.major_rev;
  pm.status.minor_rev;
  pm.status.min_input_voltage;
  pm.status.max_input_current;*/

  int i = 0;
  while (myBoard->ok())
  {
    i++;
    if (i % 200 < 100) // Alternate between sending and not sending messages to test the display.
    {
      myBoard->process_message(&pm);
      ROS_INFO("Sent message");
    }
    else
      ROS_INFO("Idling");
    ros::Duration(0,1e8).sleep();
  }
}

void processSentMessage(CommandMessage *cmd)
{
  ROS_DEBUG("processSentMessage");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv);

  CreateAllInterfaces();
  myBoard = new PowerBoard();

  boost::thread sendThread( &sendMessages );
  boost::thread fakeDeviceThread( &generateDeviceMessages );

  myBoard->spin(); //wait for ros to shut us down

  sendThread.join();

  CloseAllDevices();

  
  delete myBoard;
  return 0;

}
