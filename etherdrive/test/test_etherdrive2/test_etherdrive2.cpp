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
#include <iostream>
#include <fstream>
#include <signal.h>

using namespace std;

void shutdown(int sig);

EtherDrive e;

int main() {

  (void) signal(SIGINT,shutdown); 

  ofstream outstr;
  outstr.open("output");

  extern EtherDrive e;
  
  if (!e.init("192.168.1.12")) {
    cout << "Could not initialize etherdrive." << endl;
    return -1;
  }

  EDMotor m0 = e.get_motor(0);
  //  EDMotor m1 = e.get_motor(1);
  EDMotor m2 = e.get_motor(2);

  e.motors_on();
  
  int count = 0;
  int counter2 = 0;

  int wait_time = 1000;
  int junk = 50;
  int drv = junk;
  
  e.set_control_mode(0); // 0 = voltage, 1 = current, 2 = position

  if (!m0.set_gains(150, 0, 0, 80, 300, 0)) { // P, I, D, Windup, Clamp, Deadzone
    printf("Setting gains failed!\n");
    return 0;
  }

  while (1) {

    m0.set_drv(drv);

    if (!e.tick())
      printf("Tick problem!.");

    printf("Encoder0: %d Current: %d PWM: %d Drv: %d\n", m0.get_enc(), m0.get_cur(), m0.get_pwm(), drv);
    std::cout << count <<" " << counter2;
outstr <<count  << " " << m0.get_enc() << " " << m0.get_cur() << " "  << m0.get_pwm() << " "  << drv <<std::endl;
    //printf("Encoder1: %d Current: %d PWM: %d\n", m1.get_enc(), m1.get_cur(), m1.get_pwm());

    if (m0.get_enc() >= 15000) {
      if (counter2 < wait_time) //waiting
	{
	  drv = 0;
	}
      else //done waiting
	drv = -junk;
      //note that we're waiting
      counter2++;
    } else if (m0.get_enc() <= -15000) {
      if (counter2 < wait_time) //waiting
	{
	  drv = 0;
	}
      else //done waiting
	drv = junk;
      //note that we're done waiting
      counter2++;
    }
    else { 
      counter2 = 0; // We're not in a waiting zone, reset counter
    }
    count++;
  }

  e.shutdown();
  outstr.close();
}


void shutdown(int sig)
{
  extern EtherDrive e;
  e.shutdown();
  exit(0);
}
