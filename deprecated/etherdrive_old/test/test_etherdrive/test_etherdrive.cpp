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
  
  if (!e.init("10.0.0.151")) {
    cout << "Could not initialize etherdrive." << endl;
    return -1;
  }
  e.motors_on();
  
  int drv = 100000;
  int enc[2];
  int cur[2];

  int count = 0;

  while (1) {
    if (!e.drive(1,&drv))
      printf("Drive problem!.");
    if (!e.tick(2,enc,cur))
      printf("Tick problem!.");

    printf("Encoder0: %d Current: %d\n", enc[0], cur[0]);
    printf("Encoder1: %d Current: %d\n", enc[1], cur[1]);

    if (abs(cur[0]) > 200) {
      if (count++ > 50)
	e.motors_off();
    } else {
      count = 0;
    }

    if (enc[0] == 100000) {
      drv = -100000;
    } else if (enc[0] == -100000) {
      drv = 100000;
    }
  }

  e.shutdown();
}
