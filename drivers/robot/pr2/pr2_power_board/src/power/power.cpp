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
//#include <readline/readline.h>
#include <iostream>

// Internet/Socket stuff
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "power_comm.h"



struct Interface {
	struct ifreq interface;
	int recv_sock;
	int send_sock;
	Interface(const char* ifname);
	~Interface(void) {Close();}
	void Close(void);
	int Init(void);
	void AddToReadSet(fd_set &set, int &max_sock) const;
	bool IsReadSet(fd_set set) const;
};


struct Device {
	PowerMessage *pmsg;  //last power message recived from device
	Interface *iface;   //interface last message was recieved on;
	Device(PowerMessage *_pmsg, Interface *_iface) :
		pmsg(_pmsg),
		iface(_iface) {}
	~Device(void) {
		if (pmsg!=NULL) delete pmsg;
		pmsg = NULL;
	}	
	void Print(unsigned);
};


// Keep a pointer to the last message recieved for
// Each board.
static std::vector<Device*> Devices;
static std::vector<Interface*> Interfaces;
//static const char *interface = NULL;
int quit = 0;


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
int CreateAllInterfaces(void) {
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
		
	// Use specific interface for recv/send traffic 	
	if (setsockopt(recv_sock, SOL_SOCKET, SO_BINDTODEVICE, 
				   (char *)&interface.ifr_name, sizeof(interface.ifr_name))  < 0) {
		fprintf(stderr, "Error binding send sock to interface '%s' : %s\n",
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



int send_command(const char* input) {	
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
	assert(device->pmsg != NULL);

	
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
	cmdmsg.header.serial_num = device->pmsg->header.serial_num;
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


void Device::Print(unsigned index) {
	assert(this != NULL);
	assert(pmsg != NULL);
	printf("[%u] %u : %.*s\n", index,
		   pmsg->header.serial_num,
	       sizeof(pmsg->header.text),
	       pmsg->header.text);	
}

int list_devices(void)
{	
	if (Devices.size() == 0) {
		printf("No devices detected\n");
		return 0;
	}

	printf("Devices:\n");
	for (unsigned i = 0; i<Devices.size(); ++i) {
		assert(Devices[i] != NULL);
		Devices[i]->Print(i);
	}
	
	return 0;
}


const char* cb_state_to_str(char state)
{
	//enum CB_State { STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED };
	switch(state) {
		case STATE_NOPOWER:
			return "no-power";
		case STATE_STANDBY:
			return "stand-by";
		case STATE_PUMPING:
			return "pumping";
		case STATE_ON:
			return "on";
		case STATE_DISABLED:
			return "disabled";
	}
	return "???";			
}

const char* master_state_to_str(char state)
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

int print_status(void)
{
	if (Devices.size() == 0) {
		printf("No devices detected\n");
		return 0;
	}
	
	for (unsigned i = 0; i<Devices.size(); ++i) {
		Device *device = Devices[i];
		PowerMessage *pmsg = device->pmsg;		
		
		StatusStruct *status = &Devices[i]->pmsg->status;
		
		printf("\nDevice %u\n", i);
		printf(" Serial       = %u\n", pmsg->header.serial_num);

		printf(" Current      = %f\n", status->input_current);

		printf(" Voltages:\n");
		printf("  Input       = %f\n", status->input_voltage);
		printf("  DCDC 12     = %f\n", status->DCDC_12V_out_voltage);
		printf("  DCDC 19     = %f\n", status->DCDC_12V_out_voltage);
		printf("  DCDC 12     = %f\n", status->DCDC_12V_out_voltage);
		printf("  CB0 (L-arm)  = %f\n", status->CB0_voltage);
		printf("  CB1 (Base) = %f\n", status->CB1_voltage);
		printf("  CB2 (R-arm) = %f\n", status->CB2_voltage);

		printf(" Board Temp   = %f\n", status->ambient_temp);
		printf(" Fan Speeds:\n");
		printf("  Fan 0       = %u\n", status->fan0_speed);
		printf("  Fan 1       = %u\n", status->fan1_speed);
		printf("  Fan 2       = %u\n", status->fan2_speed);
		printf("  Fan 3       = %u\n", status->fan3_speed);

		printf(" State:\n");		
		printf("  CB0 (L-arm)  = %s\n", cb_state_to_str(status->CB0_state));
		printf("  CB1 (Base) = %s\n", cb_state_to_str(status->CB1_state));
		printf("  CB2 (R-arm) = %s\n", cb_state_to_str(status->CB2_state));
		printf("  DCDC        = %s\n", master_state_to_str(status->DCDC_state));
		
		printf(" Status:\n");		
		printf("  CB0 (L-arm)  = %s\n", (status->CB0_status) ? "On" : "Off");
		printf("  CB1 (Base) = %s\n", (status->CB1_status) ? "On" : "Off");
		printf("  CB2 (R-arm) = %s\n", (status->CB2_status) ? "On" : "Off");
		printf("  estop_button= %x\n", (status->estop_button_status));
		printf("  estop_status= %x\n", (status->estop_status));

    printf(" Revisions:\n");
    printf("         PCA = %c\n", status->pca_rev);
    printf("         PCB = %c\n", status->pcb_rev);
    printf("       Major = %c\n", status->major_rev);
    printf("       Minor = %c\n", status->minor_rev);
		
	}
	
	return 0;
}

// Determine if a record of the device already exists...
// If it does use newest message a fill in pointer to old one .
// If it does not.. use 
int process_message(PowerMessage* &msg, Interface *recvInterface)
{
	assert (recvInterface != NULL);

	if (msg->header.message_revision != CURRENT_MESSAGE_REVISION) {
		fprintf(stderr, "Got message with incorrect message revision %u\n", msg->header.message_revision);
		return -1;
	}
	
	// Look for device serial number in list of devices...
	for (unsigned i = 0; i<Devices.size(); ++i) {
		if (Devices[i]->pmsg->header.serial_num == msg->header.serial_num) {
			PowerMessage *old = Devices[i]->pmsg;
			Devices[i]->pmsg = msg; 
			msg = old; // recylce old power message
			return 0;
		}
	}

	// Add new device to list
	Device *newDevice = new Device(msg, recvInterface);
	Devices.push_back(newDevice);
	msg = NULL; // Make sure calling function can use power message
				// any more
	return 0;
}

// collect status packets for 0.5 seconds.  Keep the last packet sent
// from each seperate power device.
int collect_messages(void) {
	PowerMessage *tmp_msg(NULL);

	timeval timeout;
	timeout.tv_sec = 0; 
	timeout.tv_usec = 1000*100; //Collect packets for 1/10th of a second

	
	while (1) {
		if (tmp_msg == NULL)
			tmp_msg = new PowerMessage;
		assert(tmp_msg);
		
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
			// timeout
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
			
			int len = recv(recvInterface->recv_sock, tmp_msg, sizeof(*tmp_msg), 0);
			if (len == -1) {
				perror("Error recieving on socket");
				return -1;
			}
			else if (len != sizeof(*tmp_msg)) {
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

	if (tmp_msg)
		delete tmp_msg;

	return 0;
}


// List devices and prompt use to select one
int select_device(const char *command)
{
	
	return 0;
}


void print_menu(void) {
	printf(
		"l - list devices\n"
		"s - print status of devices\n"
		"c - send command to selected device\n"
		"    Must provide three arguments - device#, CB#, and command\n"		
		"    1st : Device Num : must be integer from 0 to #devices-1\n"
		"        Use (l)ist, to print list of detected devices\n"
		"    2nd : Circuit Breaker Num : must be integer 0-2\n"
		"        (0=LeftArm, 1=Base, 2=RightArm)\n"
		"    3rd : Command to secnd :\n"
		"        (start, stop, reset, disable, none)\n"
		"q - quit\n\n"
		   );
}


void catch_signal(int sig)
{
	if (!quit) {
		quit = 1;
	} else {		
		fprintf(stderr, "signal caught twice - quiting\n");
		CloseAllInterfaces();
		exit(1);
	}
}


int main(int argc, char** argv) {

	CreateAllInterfaces();
	
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);
	signal(SIGHUP, catch_signal);
	
	print_menu();
	
  char last_command = '?';
	while (!quit) {

          char command [200];
	  std::cin.getline(command, sizeof(command));


#if 0
		if (command == NULL) {
			print_menu();
			continue;
		}				
#endif
		
		collect_messages();
		
		switch((command[0] == 0) ? last_command : command[0]) {
			case 'l':
				list_devices();
				break;
			case 's':
				print_status();
				break;
			case 'c':
				send_command(command+1);
				break;
			case 'q':
				quit = true;
				break;
      case '?':
			default:
				print_menu();
				break;
		}

		assert(command!=NULL);

    if(command[0] != 0)
      last_command = command[0];

	}

	CloseAllDevices();
	CloseAllInterfaces();
		
	return 0;
	
}
