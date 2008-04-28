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

using namespace std;

int main() {

  EtherDrive e;
  
  if (!e.init("192.168.0.100")) {
    cout << "Could not initialize etherdrive." << endl;
    return -1;
  }

  EDMotor m0 = e.get_motor(0);
  EDMotor m1 = e.get_motor(1);

  e.motors_on();
  
  int count = 0;

  int drv = 100000;

  e.set_control_mode(2);

  if (!m1.set_gains(150, 20, -10, 50, 200, 13)) {
    printf("Setting gains failed!\n");
    return 0;
  }

  while (1) {

    m0.set_drv(drv);

    if (!e.tick())
      printf("Tick problem!.");

    printf("Encoder0: %d Current: %d PWM: %d\n", m0.get_enc(), m0.get_cur(), m0.get_pwm());
    printf("Encoder1: %d Current: %d PWM: %d\n", m1.get_enc(), m1.get_cur(), m1.get_pwm());

    
    // Crappy control
    if (abs(m0.get_cur()) > 200) {
      if (count++ > 50)
	e.motors_off();
    } else {
      count = 0;
    }

    if (m0.get_enc() == 100000) {
      drv = -100000;
    } else if (m0.get_enc() == -100000) {
      drv = 100000;
    }
  }

  e.shutdown();
}
