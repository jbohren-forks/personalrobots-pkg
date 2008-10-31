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

#include "etherdrive/etherdrive.h"
#include <cstring>

EtherDrive::EtherDrive()
{

  ready = false;

  for (int i = 0; i < 6; i++) {
    last_drv[i] = 0;
  }

  cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
  mot_sock = socket(AF_INET, SOCK_DGRAM, 0);

}

EtherDrive::~EtherDrive()
{
  shutdown();
}

bool EtherDrive::init(string ip, string bind_ip) {

  if (ready) {
    shutdown();
  }

  struct sockaddr_in mot_addr;
  struct sockaddr_in cmd_addr;

  

  cmd_addr.sin_family = AF_INET;
  cmd_addr.sin_port = htons(4950);
  cmd_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind_ip != string("")) {
    cmd_addr.sin_addr.s_addr = inet_addr(bind_ip.c_str());
    printf("Setting command port ip to: %s\n", bind_ip.c_str());
  }
  

  if (bind(cmd_sock, (struct sockaddr *)&cmd_addr, sizeof(cmd_addr)) < 0) {
    printf("ERROR on binding to command port: 4950\n");
    return false;
  }


  mot_addr.sin_family = AF_INET;
  mot_addr.sin_port = htons(4900);
  mot_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind_ip != string("")) {
    mot_addr.sin_addr.s_addr = inet_addr(bind_ip.c_str());
    printf("Motor port ip to: %s\n", bind_ip.c_str());
  }

 
  if (bind(mot_sock, (struct sockaddr *)&mot_addr, sizeof(mot_addr)) < 0) {
    printf("ERROR on binding to motor port: 4900\n");
    
    close(cmd_sock);

    return false;
  }

  mot_addr_out.sin_family = AF_INET;
  mot_addr_out.sin_port = htons(4900);
  mot_addr_out.sin_addr.s_addr = inet_addr(ip.c_str());

  cmd_addr_out.sin_family = AF_INET;
  cmd_addr_out.sin_port = htons(4950);
  cmd_addr_out.sin_addr.s_addr = inet_addr(ip.c_str());

  ready = true;
  return true;
}


void EtherDrive::shutdown() {

  if (ready = true) {
    close(cmd_sock);
    close(mot_sock);
  }

  ready = false;
}


int EtherDrive::send_cmd(char* cmd, size_t cmd_len, char* buf, size_t buf_len) {

  if (!ready)
    return -1;

  size_t n_sent;
  size_t n_recv;

  struct sockaddr_in from;
  socklen_t fromlen = sizeof(struct sockaddr_in);

  n_sent = sendto(cmd_sock, cmd, cmd_len, 0, (struct sockaddr *)&cmd_addr_out, sizeof(mot_addr_out));

  if (n_sent != cmd_len) {
    return -1;
  }

  n_recv = recvfrom(cmd_sock, buf, buf_len, 0, (struct sockaddr *)&from, &fromlen);

  return n_recv;
}

bool EtherDrive::motors_on() {


  if (ready) {
    int32_t cmd[3];
    cmd[0] = 'm';
    cmd[1] = 1;
    cmd[2] = 0;
  
    int32_t res[3];
    size_t res_len = send_cmd((char*)cmd, 3*sizeof(int32_t), (char*)res, 100*sizeof(int32_t));
    if (res_len == 3*sizeof(int32_t)) {
      if (res[1] == 1) {
	return true;
      } 
    }    
  }
  return false;
}

bool EtherDrive::motors_off() {

  if (ready) {
    int32_t cmd[3];
    cmd[0] = 'm';
    cmd[1] = 0;
    cmd[2] = 0;

    int32_t res[3];

    size_t res_len = send_cmd((char*)cmd, 3*sizeof(int32_t), (char*)res, 100*sizeof(int32_t));

    if (res_len == 3*sizeof(int32_t)) {
      if (res[1] == 0) {
	return true;
      } 
    }    
  }

  return false;

}

bool EtherDrive::set_control_mode(int8_t m) {
  if (ready) {
    if (m == 0 || m == 1 || m == 2) {

      int32_t cmd[3];
      cmd[0] = 'i';
      cmd[1] = m;
      cmd[2] = 0;

      int32_t res[3];

      size_t res_len = send_cmd((char*)cmd, 3*sizeof(int32_t), (char*)res, 100*sizeof(int32_t));

      if (res_len == 3*sizeof(int32_t)) {
	if (res[1] == m) {
	  return true;
	} 
      }
    }
  }

  return false;
}


bool EtherDrive::set_gain(uint32_t m, char G, int32_t val) {
  if (ready) {

    if (m < 6) {

      char cmds[] = "PIDWMZ";
      if ( memchr(cmds, G, strlen(cmds)) != NULL) {

	int32_t cmd[3];
	cmd[0] = G;
	cmd[1] = m;
	cmd[2] = val;
	
	int32_t res[3];
	
	size_t res_len = send_cmd((char*)cmd, 3*sizeof(int32_t), (char*)res, 100*sizeof(int32_t));
	
	if (res_len == 3*sizeof(int32_t)) {
	  if (res[1] == (int32_t)(m) && res[2] == val) {
	    return true;
	  } 
	}
      }
    }
  }
  return false;
}


EDMotor EtherDrive::get_motor(uint32_t m) {
    return EDMotor(this, m);
} 

void EtherDrive::set_drv(uint32_t m, int32_t drv) {
  if (m < 6) {
    last_drv[m] = drv;
  }
}

int32_t EtherDrive::get_enc(uint32_t m) {
  if (m < 6) {
    return last_enc[m];
  }
  return 0;
}

int32_t EtherDrive::get_cur(uint32_t m) {
  if (m < 6) {
    return last_cur[m];
  }
  return 0;
}

int32_t EtherDrive::get_pwm(uint32_t m) {
  if (m < 6) {
    return last_pwm[m];
  }
  return 0;
}


bool EtherDrive::tick(size_t num,  int32_t* enc, int32_t* cur, int32_t* pwm)
{

  if (!ready)
    return false;

  if (num > 6) {
    return false;
  }

  int n_sent;
  int n_recv;

  struct sockaddr_in from;
  socklen_t fromlen = sizeof(struct sockaddr_in);

  n_sent = sendto(mot_sock, (char*)last_drv, 6*sizeof(int32_t), 0, (struct sockaddr *)&mot_addr_out, sizeof(mot_addr_out));

  if (n_sent != 6*sizeof(int32_t)) {
    return false;
  }
  
  int32_t buf[72];

  n_recv = recvfrom(mot_sock, (char*)buf, 72, 0, (struct sockaddr *)&from, &fromlen);

  if (n_recv != 72) {
    return false;
  }


  for (int i = 0; i < 6; i++) {
    last_enc[i] = buf[i];
  }
  for (int i = 0; i < 6; i++) {
    last_cur[i] = buf[6+i];
  }
  for (int i = 0; i < 6; i++) {
    last_pwm[i] = buf[12+i];
  }


  if (enc > 0) {
    for (size_t i = 0; i < num; i++) {
      enc[i] = last_enc[i];
    }
  }
  if (cur > 0) {
    for (size_t i = 0; i < num; i++) {
      cur[i] = last_cur[i];
    }
  }
  if (pwm > 0) {
    for (size_t i = 0; i < num; i++) {
      pwm[i] = last_pwm[i];
    }
  }

  return n_recv;
}

bool EtherDrive::drive(size_t num, int32_t* drv)
{
  if (num > 6) {
    return false;
  }

  for (size_t i = 0; i < num; i++) {
    last_drv[i] = drv[i];
  }

  return true;
}

