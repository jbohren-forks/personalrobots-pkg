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
#include <sys/time.h>
#include <sys/select.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <vector>
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


// Keep a pointer to the last message recieved for
// Each board.
static std::vector<Device*> Devices;
static std::vector<Interface*> Interfaces;
static PowerBoard *myBoard;

Interface::Interface(const char* ifname) :
  recv_sock(-1),
  send_sock(-1) {		
  memset(&interface, 0, sizeof(interface));
  assert(strlen(ifname) <= sizeof(interface.ifr_name));
  strncpy(interface.ifr_name, ifname, IFNAMSIZ);
}


void Interface::Close(void) {
  if (recv_sock != -1) {
    close(recv_sock);
    recv_sock = -1;
  }
  if (send_sock != -1) {
    close(send_sock);
    send_sock = -1;
  }
}

	
int Interface::Init(void) 
{
  recv_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (recv_sock == -1) {
    perror("Couldn't create recv socket");		
    return -1;
  }
  send_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (send_sock == -1) {
    Close();
    perror("Couldn't create send socket");		
    return -1;
  }
  /*		
  // Use specific interface for recv/send traffic 	
  if (setsockopt(recv_sock, SOL_SOCKET, SO_BINDTODEVICE, 
                 (char *)&interface.ifr_name, sizeof(interface.ifr_name))  < 0) {
    fprintf(stderr, "Error binding send sock to interface '%s' : %s --case1\n",
            interface.ifr_name, strerror(errno));
    Close();
    return -1;
  }
  if (setsockopt(send_sock, SOL_SOCKET, SO_BINDTODEVICE, 
                 (char *)&interface.ifr_name, sizeof(interface.ifr_name))  < 0) {
    fprintf(stderr, "Error binding send sock to interface '%s' : %s\n",
            interface.ifr_name, strerror(errno));
    Close();
    return -1;
  }
  */
		
 // Allow reuse of recieve port
  int opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("Couldn't set reuse addr on recv socket\n");
    Close();
    return -1;
  }
		
  // Allow broadcast on send socket
  opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt))) {
    perror("Setting broadcast option on recv");
    Close();
    return -1;
  }
  // All recieving packets sent to broadcast address
  opt = 1;
  if (setsockopt(send_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt))) {
    perror("Setting broadcast option on send");
    Close();
    return -1;
  }

	
  // Bind socket to recieve packets on <UDP_STATUS_PORT> from any address/interface
  struct sockaddr_in sin;
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  //sin.sin_port = htons(POWER_PORT);
  sin.sin_port = htons(POWER_PORT);
  sin.sin_addr.s_addr = (INADDR_ANY);
  if (bind(recv_sock, (struct sockaddr*)&sin, sizeof(sin))) {
    perror("Couldn't Bind socket to port");
    Close();
    return -1;
  }	
	
  // Connect send socket to use broadcast address and same port as recieve sock		
  sin.sin_port = htons(POWER_PORT);
  //sin.sin_addr.s_addr = inet_addr("255.255.255.255");
  sin.sin_addr.s_addr = INADDR_BROADCAST; //inet_addr("192.168.10.255");
  //sin.sin_addr.s_addr = (INADDR_ANY);
  if (connect(send_sock, (struct sockaddr*)&sin, sizeof(sin))) {
    perror("Connect'ing socket failed");
    Close();
    return -1;
  }

  return 0;		
}

void Interface::AddToReadSet(fd_set &set, int &max_sock) const {
  FD_SET(recv_sock,&set);	
  if (recv_sock > max_sock)
    max_sock = recv_sock;
}

bool Interface::IsReadSet(fd_set set) const {
  return FD_ISSET(recv_sock,&set);	
}

int PowerBoard::send_command(const char* input) 
{	
  int selected_device=-1;
  int circuit_breaker=-1;
  char command[8+1];

  if (Devices.size() == 0) {
    fprintf(stderr,"No devices to send command to\n");
    return -1;
  }
	
  if (sscanf(input,"%i %i %8s",&selected_device, &circuit_breaker, command) != 3) {
    printf("Need two values : device# & circuit breaker#, got '%s'\n", input);
    return -1;
  }

  if ((selected_device < 0) || (selected_device >= (int)Devices.size())) {
    fprintf(stderr, "Device number must be between 0 and %u\n", Devices.size()-1);
    return -1;
  }

  Device* device = Devices[selected_device];
  assert(device != NULL);
  assert(device->iface != NULL);

	
  if ((circuit_breaker < 0) || (circuit_breaker >= 3)) {
    fprintf(stderr, "Circuit breaker number must be between 0 and 2\n");
    return -1;
  }	

  //printf("command = '%s'\n",command);
	
  // Determine what command to send
  command[sizeof(command)-1] = '\0';	
  char command_enum = NONE;
  if (strncmp(command, "start",sizeof(command)) == 0) {
    command_enum = COMMAND_START;
  }
  else if (strncmp(command, "stop",sizeof(command)) == 0) {
    command_enum = COMMAND_STOP;
  }
  else if (strncmp(command, "reset",sizeof(command)) == 0) {
    command_enum = COMMAND_RESET;		
  }
  else if (strncmp(command, "disable",sizeof(command)) == 0) {
    command_enum = COMMAND_DISABLE;
  }
  else if (strncmp(command, "none",sizeof(command)) == 0) {
    command_enum = NONE;
  }
  else {
    fprintf(stderr, "invalid command '%s'\n", command);
    return -1;
  }
  //" -c : command to send to device : 'start', 'stop', 'reset', 'disable'\n"	

	
  // Build command message
  CommandMessage cmdmsg;
  memset(&cmdmsg, 0, sizeof(cmdmsg));
  cmdmsg.header.message_revision = CURRENT_MESSAGE_REVISION;
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
   	
  errno = 0;
  int result = send(device->iface->send_sock, &cmdmsg, sizeof(cmdmsg), 0);
  if (result == -1) {
    perror("Error sending");
    return -1;
  } else if (result != sizeof(cmdmsg)) {
    fprintf(stderr,"Error sending : send only took %d of %d bytes\n",
            result, sizeof(cmdmsg));
  }
				
  printf("Sent command %s(%d) to device %d, circuit %d\n",
         command, command_enum, selected_device, circuit_breaker);
	
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
int PowerBoard::process_message(PowerMessage &msg, Interface *recvInterface)
{
  assert (recvInterface != NULL);

  if (msg.header.message_revision != CURRENT_MESSAGE_REVISION) {
    fprintf(stderr, "Got message with incorrect message revision %u\n", msg.header.message_revision);
    return -1;
  }
	
  // Look for device serial number in list of devices...
  for (unsigned i = 0; i<Devices.size(); ++i) {
    if (Devices[i]->pmsg.header.serial_num == msg.header.serial_num) {
      library_lock_.lock();
      memcpy(&(Devices[i]->pmsg), &msg, sizeof(msg)); 
      library_lock_.unlock();
      return 0;
    }
  }

  // Add new device to list
  Device *newDevice = new Device(recvInterface);
  Devices.push_back(newDevice);
  memcpy(&(newDevice->pmsg), &msg, sizeof(msg)); 
  return 0;
}

// collect status packets for 0.5 seconds.  Keep the last packet sent
// from each seperate power device.
int PowerBoard::collect_messages() 
{
  PowerMessage tmp_msg;

  timeval timeout;  //timeout once a second to check if we should die or not.
  timeout.tv_sec = 1; 
  timeout.tv_usec = 0;
	
  while (1) {
		
    // Wait for packets to arrive on socket. 
    fd_set read_set;
    int max_sock = -1;
    FD_ZERO(&read_set);
    for (unsigned i = 0; i<Interfaces.size(); ++i)
      Interfaces[i]->AddToReadSet(read_set,max_sock);
		
    int result = select(max_sock+1, &read_set, NULL, NULL, &timeout);

    //fprintf(stderr,"*");
		
    if (result == -1) {
      perror("Select");
      return -1;
    }
    else if (result == 0) {
      return 0;
    }
    else if (result >= 1) {
      Interface *recvInterface = NULL;
      for (unsigned i = 0; i<Interfaces.size(); ++i) {
        //figure out which interface we recieved on
        if (Interfaces[i]->IsReadSet(read_set)) {
          recvInterface = Interfaces[i];
          break;
        }
      }
			
      int len = recv(recvInterface->recv_sock, &tmp_msg, sizeof(tmp_msg), 0);
      if (len == -1) {
        perror("Error recieving on socket");
        return -1;
      }
      else if (len != sizeof(tmp_msg)) {
        fprintf(stderr, "recieved message of incorrect size %d\n", len);
      }
      else {
        if (process_message(tmp_msg, recvInterface)){
          return -1;
        }
      }
    }
    else {
      fprintf(stderr,"Unexpected select result %d\n", result);
      return -1;
    }
  }

  return 0;
}

PowerBoard::PowerBoard(): ros::node ("pr2_power_board")
{
  advertise_service("power_board_control", &PowerBoard::commandCallback);
  advertise<robot_msgs::DiagnosticMessage>("/diagnostics", 2);
}
  
bool PowerBoard::commandCallback(pr2_power_board::PowerBoardCommand::request &req_,
                     pr2_power_board::PowerBoardCommand::response &res_)
{
  std::stringstream ss;
  ss << " 0 " << req_.breaker_number<<" " << req_.command;
  //library_lock_.lock();
  res_.retval = send_command(ss.str().c_str());
  //library_lock_.unlock();
  return true;
}

void PowerBoard::collectMessages()
{
  while(ok())
  {
    //library_lock_.lock();
    collect_messages();
    //library_lock_.unlock();
    printf("*");
    fflush(stdout);
  }
}
  
void PowerBoard::sendDiagnostic()
{
  
  while(ok())
  {
    sleep(1);
    printf("-");
    fflush(stdout);
    library_lock_.lock();
    robot_msgs::DiagnosticMessage msg_out;

    for (unsigned i = 0; i<Devices.size(); ++i) {
      Device *device = Devices[i];
      PowerMessage *pmsg = &device->pmsg;		
    
      robot_msgs::DiagnosticStatus stat;
      std::stringstream ss;
      ss << "Power board" << i;
      stat.name = ss.str();
      stat.level = 0;///@todo fixem
      stat.message = "fixme";
      StatusStruct *status = &Devices[i]->pmsg.status;
      
      printf("\nDevice %u\n", i);
      printf(" Serial       = %u\n", pmsg->header.serial_num);
      robot_msgs::DiagnosticValue val;
      robot_msgs::DiagnosticString strval;
      ss.str("");
      ss << pmsg->header.serial_num;
      strval.value = ss.str();
      strval.label = "Serial Number";
      stat.strings.push_back(strval);

      printf(" Current      = %f\n", status->input_current);
      val.value = status->input_current;
      val.label = "Input Current";
      stat.values.push_back(val);

      printf(" Voltages:\n");
      printf("  Input       = %f\n", status->input_voltage);
      val.value = status->input_voltage;
      val.label = "Input Voltage";
      stat.values.push_back(val);
      printf("  DCDC 12     = %f\n", status->DCDC_12V_out_voltage);
      val.value = status->DCDC_12V_out_voltage;
      val.label = "DCDC12";
      stat.values.push_back(val);
      printf("  DCDC 15     = %f\n", status->DCDC_19V_out_voltage);
      val.value = status->DCDC_19V_out_voltage;
      val.label = "DCDC 15";
      stat.values.push_back(val);
      printf("  CB0 (Base)  = %f\n", status->CB0_voltage);
      val.value = status->CB0_voltage;
      val.label = "Breaker 0 Voltage";
      stat.values.push_back(val);
      printf("  CB1 (R-arm) = %f\n", status->CB1_voltage);
      val.value = status->CB1_voltage;
      val.label = "Breaker 1 Voltage";
      stat.values.push_back(val);
      printf("  CB2 (L-arm) = %f\n", status->CB2_voltage);
      val.value = status->CB2_voltage;
      val.label = "Breaker 2 Voltage";
      stat.values.push_back(val);
      
      printf(" Board Temp   = %f\n", status->ambient_temp);
      val.value = status->ambient_temp;
      val.label = "Board Temperature";
      stat.values.push_back(val);
      printf(" Fan Speeds:\n");
      printf("  Fan 0       = %u\n", status->fan0_speed);
      val.value = status->fan0_speed;
      val.label = "Fan 0 Speed";
      stat.values.push_back(val);
      printf("  Fan 1       = %u\n", status->fan1_speed);
      val.value = status->fan1_speed;
      val.label = "Fan 1 Speed";
      stat.values.push_back(val);
      printf("  Fan 2       = %u\n", status->fan2_speed);
      val.value = status->fan2_speed;
      val.label = "Fan 2 Speed";
      stat.values.push_back(val);
      printf("  Fan 3       = %u\n", status->fan3_speed);
      val.value = status->fan3_speed;
      val.label = "Fan 3 Speed";
      stat.values.push_back(val);
      
      printf(" State:\n");		
      printf("  CB0 (Base)  = %s\n", cb_state_to_str(status->CB0_state));
      strval.value = cb_state_to_str(status->CB0_state);
      strval.label = "Breaker 0 State";
      stat.strings.push_back(strval);
      printf("  CB1 (R-arm) = %s\n", cb_state_to_str(status->CB1_state));
      strval.value = cb_state_to_str(status->CB1_state);
      strval.label = "Breaker 1 State";
      stat.strings.push_back(strval);
      printf("  CB2 (L-arm) = %s\n", cb_state_to_str(status->CB2_state));
      strval.value = cb_state_to_str(status->CB2_state);
      strval.label = "Breaker 2 State";
      stat.strings.push_back(strval);
      printf("  DCDC        = %s\n", master_state_to_str(status->DCDC_state));
      strval.value = master_state_to_str(status->DCDC_state);
      strval.label = "DCDC state";
      stat.strings.push_back(strval);
      
      printf(" Status:\n");		
      printf("  CB0 (Base)  = %s\n", (status->CB0_status) ? "On" : "Off");
      strval.value = (status->CB0_status) ? "On" : "Off";
      strval.label = "Breaker 0 Status";
      stat.strings.push_back(strval);      
      printf("  CB1 (R-arm) = %s\n", (status->CB1_status) ? "On" : "Off");
      strval.value = (status->CB1_status) ? "On" : "Off";
      strval.label = "Breaker 1 Status";
      stat.strings.push_back(strval);
      printf("  CB2 (L-arm) = %s\n", (status->CB2_status) ? "On" : "Off");
      strval.value = (status->CB2_status) ? "On" : "Off";
      strval.label = "Breaker 2 Status";
      stat.strings.push_back(strval);
      printf("  estop_button= %x\n", (status->estop_button_status));
      val.value = status->estop_button_status;
      val.label = "Estop Button Status";
      stat.values.push_back(val);
      printf("  estop_status= %x\n", (status->estop_status));
      val.value = status->estop_status;
      val.label = "Estop Status";
      stat.values.push_back(val);
      
      printf(" Revisions:\n");
      printf("         PCA = %c\n", status->pca_rev);
      ss.str("");
      ss << status->pca_rev;
      strval.value = ss.str();
      strval.label = "PCA";
      stat.strings.push_back(strval);
      printf("         PCB = %c\n", status->pcb_rev);
      ss.str("");
      ss << status->pcb_rev;
      strval.value = ss.str();
      strval.label = "PCB";
      stat.strings.push_back(strval);
      printf("       Major = %c\n", status->major_rev);
      ss.str("");
      ss << status->major_rev;
      strval.value = ss.str();
      strval.label = "Major Revision";
      stat.strings.push_back(strval);
      printf("       Minor = %c\n", status->minor_rev);
      ss.str("");
      ss << status->minor_rev;
      strval.value = ss.str();
      strval.label = "Minor Revision";
      stat.strings.push_back(strval);

      msg_out.status.push_back(stat);      
    }
    robot_msgs::DiagnosticStatus stat;
    std::stringstream ss;
    stat.name = "pr2_power_board";
    stat.level = 0;
    stat.message = "Running";
    msg_out.status.push_back(stat);      

    //printf("Publishing \n");        
    publish("/diagnostics", msg_out);    
    library_lock_.unlock();
  }
}

void getMessages()
{
  myBoard->collectMessages();
}

void sendMessages()
{
  myBoard->sendDiagnostic();
}

// CloseAll
void CloseAllInterfaces(void) {
  for (unsigned i=0; i<Interfaces.size(); ++i){
    if (Interfaces[i] != NULL) {
      delete Interfaces[i];
    }
  }
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
  struct ifreq req;

  //
  int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == -1) {
    perror("Couldn't create socket for ioctl requests");		
    return -1;
  }
	
  // Go though interfaces 0-10 and get interface names for them
  for (int ifindex=0; ifindex<10; ++ifindex) {
    memset(&req, 0, sizeof(req));
    req.ifr_ifindex = ifindex;
    if (ioctl(sock,SIOCGIFNAME,&req)) {
      if (errno != ENODEV) {
        fprintf(stderr,"Could not get name of interface at index %d : %s (%d)\n",
                req.ifr_ifindex, strerror(errno), errno);
      }
      continue;
    }
    if ((strncmp("lo", req.ifr_name, sizeof(req.ifr_name)) == 0) || 
        (strncmp("vmnet", req.ifr_name, 5) == 0))
    {
      printf("Ignoring interface %*s\n",sizeof(req.ifr_name), req.ifr_name);
      continue;
    } else {
      printf("Found interface    %*s\n",sizeof(req.ifr_name), req.ifr_name);
      Interface *newInterface = new Interface(req.ifr_name);
      assert(newInterface != NULL);
      if (newInterface == NULL) {
        close(sock);
        return -1;				
      }
      if (newInterface->Init()) {
        printf("Error initializing interface %*s\n", sizeof(req.ifr_name), req.ifr_name);
        delete newInterface;
        newInterface = NULL;
        continue;
      } else {
        // Interface is good add it to interface list
        Interfaces.push_back(newInterface);

      }
    }
  }

  printf("Found %d usable interfaces\n\n", Interfaces.size());	

	
  return 0;
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv);

  CreateAllInterfaces();
  myBoard = new PowerBoard();

  boost::thread getThread( &getMessages );
  boost::thread sendThread( &sendMessages );

  myBoard->spin(); //wait for ros to shut us down

  sendThread.join();
  getThread.join();

  CloseAllDevices();
  CloseAllInterfaces();
		
  ros::fini();
  delete myBoard;
  return 0;
	
}
