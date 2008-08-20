///////////////////////////////////////////////////////////////////////////////
// This ROS node talks to the etherdrive or the caster node and commands it.
//
// Copyright (C) 2008, Jimmy Sastra
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



#include "ros/node.h"
#include "std_msgs/ActuatorState.h"
#include "std_msgs/ActuatorCmd.h"
#include <joy/Joy.h>
//#include "unstable_msgs/MsgActuator.h"
#include <std_msgs/BaseVel.h>
#include "etherdrive/etherdrive.h"
#include <sstream>

#include <stdlib.h>
#include <stdio.h>

#include <string>



using std::string;

int dir;
int prevDir;
class EtherDrive_Node_Controller : public ros::node
{
public:
  std_msgs::ActuatorState mot[12];     //in
  std_msgs::ActuatorCmd mot_cmd[12]; //out
  joy::Joy joy_msg;
  // MsgBaseVel cmdvel;
 // std_msgs::ActuatorCmd mot_cmd[6];
  //int dir;

  char name[100];

  EtherDrive_Node_Controller() : ros::node("etherdrivecontroller")
  {
    for (int i = 0;i < 12;i++) {
      ostringstream oss; // bus
      sprintf(name, "mot%d_cmd", i);
      printf("advertising %s\n", name);
      advertise<std_msgs::ActuatorCmd>(name);
   }
   for(int i = 0; i < 12; i++){
      sprintf(name, "mot%d", i);     
      subscribe(name, mot[i], &EtherDrive_Node_Controller::displayMotorValues);

//      subscribe("cmd_vel",cmdvel,&EtherDrive_Node_Controller::joystickInput); 
      subscribe("joy", joy_msg, &EtherDrive_Node_Controller::wiiInput); 
    }
  }


  void displayMotorValues() {
    int pos[12];
    
    for(int i = 0;i < 12 ;i++) {
      pos[i] = mot[i].pos;
       if(i == 4) {
 //         printf("Motor %i pos: %i\n",i, pos[i]);
         if(mot[i].pos >= 20000) {
            dir = -1;
        }
        else if(mot[i].pos <= 0) {
            dir = 1;
        }
      }
    }
  }

  void joystickInput() { 
/*  float vx;
  float vw;

  vx = 0.1*cmdvel.vx;
  vw = 0.01*cmdvel.vw;
  printf("vx: %f\n", vx);
  printf("vw: %f\n", vw);
  
  sendMotorCommand(0,vw,0,true,0);
  sendMotorCommand(1,vx,0,true,0);
  sendMotorCommand(2,vx,0,true,0);
  */
  }
  void wiiInput() {
    printf("axis 0: %f, axis 1: %f%f\n", joy_msg.axes[0], joy_msg.axes[1] );
  float w;
  float v;
  w = joy_msg.axes[0];
  v = joy_msg.axes[1];

  
        sendMotorCommand(0,v*500,0,true, 2);
        sendMotorCommand(1,-v*500,0,true, 2);
        sendMotorCommand(2,w*90,0,true, 1);
        sendMotorCommand(3,v*500,0,true, 2);
        sendMotorCommand(4,-v*500,0,true, 2);
        sendMotorCommand(5,-w*90,0,true, 1);
        sendMotorCommand(6,-v*500,0,true, 2);
        sendMotorCommand(7,v*500,0,true, 2);
        sendMotorCommand(8,w*90,0,true, 1);
        sendMotorCommand(9,-v*500,0,true, 2);
        sendMotorCommand(10,v*500,0,true, 2);
        sendMotorCommand(11,-w*90,0,true, 1);
        


  }
  void sendMotorCommand(int motor, float val, int rel, bool valid, int mode) {
    mot_cmd[motor].cmd = val;
    mot_cmd[motor].rel= rel;
    mot_cmd[motor].valid= valid;
    mot_cmd[motor].mode = mode;
    
    ostringstream oss;
    oss << "mot" << motor << "_cmd";
    publish(oss.str().c_str(), mot_cmd[motor]);

  }



};



int main(int argc, char **argv)
{
  ros::init(argc, argv);
  EtherDrive_Node_Controller a;
printf("start\n");
  int fooCounter = 0;
    dir = 1;
    prevDir = 1;
  int controlMode = 3; //2; //0=command directly, 1=reverse by time interval, 2=switch by encoder count, 3=keyboard
  switch(controlMode) {
    case 0:
      while(a.ok()) {
        char inputstring[512];
        float val = 0.0;
        int motor = 0;
        int mode = 0;
            
        printf("Enter Motor (0-11):\n");
        gets(inputstring);
        motor = atoi(inputstring);

        printf("Enter Mode (0=direct, 1=position, 2=velocity):\n");
        gets(inputstring);
        mode = atoi(inputstring);

        printf("Enter value:\n");
        gets(inputstring);
        val = atof(inputstring);    

        a.sendMotorCommand(motor,val,0,true, mode);
      }
      break;
    case 1:   //square
      while(a.ok()) {
        a.sendMotorCommand(0,500,0,true, 2);
        a.sendMotorCommand(1,-500,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,500,0,true, 2);
        a.sendMotorCommand(4,-500,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        a.sendMotorCommand(6,-500,0,true, 2);
        a.sendMotorCommand(7,500,0,true, 2);
        a.sendMotorCommand(8,0,0,true, 1);
        a.sendMotorCommand(9,-500,0,true, 2);
        a.sendMotorCommand(10,500,0,true, 2);
        a.sendMotorCommand(11,0,0,true, 1);
        sleep(3);
/*      square
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        a.sendMotorCommand(6,0,0,true, 2);
        a.sendMotorCommand(7,0,0,true, 2);
        a.sendMotorCommand(8,90,0,true, 1);
        a.sendMotorCommand(9,0,0,true, 2);
        a.sendMotorCommand(10,0,0,true, 2);
        a.sendMotorCommand(11,90,0,true, 1);
        sleep(3);
*/
/*
        a.sendMotorCommand(0,200,0,true, 2);
        a.sendMotorCommand(1,-200,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
=======
        a.sendMotorCommand(3,500,0,true, 2);
        a.sendMotorCommand(4,-500,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        a.sendMotorCommand(6,-500,0,true, 2);
        a.sendMotorCommand(7,500,0,true, 2);
        a.sendMotorCommand(8,0,0,true, 1);
        a.sendMotorCommand(9,-500,0,true, 2);
        a.sendMotorCommand(10,500,0,true, 2);
        a.sendMotorCommand(11,0,0,true, 1);
        sleep(3);
/*
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        a.sendMotorCommand(6,0,0,true, 2);
        a.sendMotorCommand(7,0,0,true, 2);
        a.sendMotorCommand(8,90,0,true, 1);
        a.sendMotorCommand(9,0,0,true, 2);
        a.sendMotorCommand(10,0,0,true, 2);
        a.sendMotorCommand(11,90,0,true, 1);
        sleep(3);
*/
/*
        a.sendMotorCommand(0,200,0,true, 2);
        a.sendMotorCommand(1,-200,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
>>>>>>> .r1604
        a.sendMotorCommand(3,-200,0,true, 2);
        a.sendMotorCommand(4,200,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(3);// turn
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(3);
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        sleep(3); // end turn
        a.sendMotorCommand(0,200,0,true, 2);
        a.sendMotorCommand(1,-200,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,-200,0,true, 2);
        a.sendMotorCommand(4,200,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        sleep(3);
        // turn
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        sleep(3);
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(3); // end turn
        a.sendMotorCommand(0,-200,0,true, 2);
        a.sendMotorCommand(1,200,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,200,0,true, 2);
        a.sendMotorCommand(4,-200,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(3);// turn
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(3);
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        sleep(3); // end turn
        a.sendMotorCommand(0,-200,0,true, 2);
        a.sendMotorCommand(1,200,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,200,0,true, 2);
        a.sendMotorCommand(4,-200,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        sleep(3);// turn
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,90,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,90,0,true, 1);
        sleep(3);
        a.sendMotorCommand(0,0,0,true, 2);
        a.sendMotorCommand(1,0,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,0,0,true, 2);
        a.sendMotorCommand(4,0,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(3); // end turn
*/
      }
      break;
    case 2:
     while (a.ok()) {
       if(dir == 1) {        
        a.sendMotorCommand(0,0,0,true, 1);
        a.sendMotorCommand(1,500,0,true, 2);
        a.sendMotorCommand(2,500,0,true, 2);
        a.sendMotorCommand(3,500,0,true, 2);
        a.sendMotorCommand(4,-500,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        a.sendMotorCommand(6,-500,0,true, 2);
        a.sendMotorCommand(7,500,0,true, 2);
        a.sendMotorCommand(8,0,0,true, 1);
        a.sendMotorCommand(9,-500,0,true, 2);
        a.sendMotorCommand(10,500,0,true, 2);
        a.sendMotorCommand(11,0,0,true, 1);
        }
      else { // dir == -1
        a.sendMotorCommand(0,-500,0,true, 2);
        a.sendMotorCommand(1,500,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,-500,0,true, 2);
        a.sendMotorCommand(4,500,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        a.sendMotorCommand(6,500,0,true, 2);
        a.sendMotorCommand(7,-500,0,true, 2);
        a.sendMotorCommand(8,0,0,true, 1);
        a.sendMotorCommand(9,500,0,true, 2);
        a.sendMotorCommand(10,-500,0,true, 2);
        a.sendMotorCommand(11,0,0,true, 1);
//      a.sendMotorCommand(5,34009,0,true, 1); rotate 180

       }
      }
      usleep(1000);
    default:
      while(a.ok()) {
usleep(1000);
      }
      break;

  }                 

  ros::fini();
  return 0;
}
