
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Jimmy Sastra, Sachin Chitta
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

//Base test

#include <ros/node.h>
#include <sstream>
#include <joy/Joy.h>
#include <pr2Controllers/BaseController.h>
#include <pr2_etherDrive/BaseTest.h>

BaseTest::BaseTest() : ros::node("BaseTest"){
   subscribe("joy", joy_msg, &BaseTest::wiiInput); 
} 

void BaseTest::wiiInput() {

   vx = joy_msg.axes[0];
   vy = joy_msg.axes[1];
   vw = joy_msg.axes[2];
//    b = joy_msg.buttons[0];
   b = 0;
   printf("vx: %f, vy: %f, vw: %f,  B : %i \n", vx, vy, vw, b);
}

double BaseTest::getTime(){
   double time;
   time = 0;
   return time;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv);
   BaseTest b;
   printf("start\n");

   BaseController bc;
   bc.Init();

   while (b.ok())
   {
      //bc.setVelocity(vx, vy, vw);
      //bc.Update();
      usleep(1000);
   }

   ros::fini();
   return 0;
}
