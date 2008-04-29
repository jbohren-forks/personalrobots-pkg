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

#ifndef ETHERDRIVE_H
#define ETHERDRIVE_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <string>

using namespace std;

class EDMotor;

class EtherDrive
{
public:
  EtherDrive();
  ~EtherDrive();

  // Initialize 
  bool init(string ip);
  
  void shutdown();

  // Manually send EtherDrive command.
  int send_cmd(char* cmd, size_t cmd_len, char* buf, size_t buf_len);

  // Enable motors
  bool motors_on();

  // Disable motors
  bool motors_off();

  bool set_control_mode(int8_t m);

  bool set_gain(uint32_t m, char G, int32_t val);

  EDMotor get_motor(uint32_t m);

  void set_drv(uint32_t m, int32_t drv);
  int32_t get_enc(uint32_t m);
  int32_t get_cur(uint32_t m);
  int32_t get_pwm(uint32_t m);

  // Send an array of motor commands up to 6 in length.
  bool drive(size_t num, int32_t* drv);

  // Send most recent motor commands, and retrieve update (this must be run at sufficient rate).
  bool tick(size_t num = 0, int32_t* enc = 0, int32_t* curr = 0, int32_t* pwm = 0);



private:
  bool ready;

  int32_t last_drv[6];
  int32_t last_enc[6];
  int32_t last_cur[6];
  int32_t last_pwm[6];

  int mot_sock;
  int cmd_sock;

  struct sockaddr_in mot_addr_out;
  struct sockaddr_in cmd_addr_out;
};

class EDMotor
{
  friend class EtherDrive;
public:
  void set_drv(int32_t drv) {
    driver->set_drv(motor, drv);
  }

  int32_t get_enc() {
    return driver->get_enc(motor);
  }

  int32_t get_cur() {
    return driver->get_cur(motor);
  }

  int32_t get_pwm() {
    return driver->get_pwm(motor);
  }

  bool set_gains(int32_t P, int32_t I, int32_t D, int32_t W, int32_t M, int32_t Z) {
    return driver->set_gain(motor, 'P', P) & 
      driver->set_gain(motor, 'I', I) & 
      driver->set_gain(motor, 'D', D) & 
      driver->set_gain(motor, 'W', W) & 
      driver->set_gain(motor, 'M', M) & 
      driver->set_gain(motor, 'Z', Z);
  }
protected:
  EDMotor(EtherDrive* driver, uint32_t motor) : driver(driver), motor(motor) {}

private:
  EtherDrive* driver;

  uint32_t motor;
};


#endif

