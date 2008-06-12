///////////////////////////////////////////////////////////////////////////////
// This ROS node talks to the etherdrive node and controls it.
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
#include "unstable_msgs/MsgActuator.h"
#include <std_msgs/MsgBaseVel.h>
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
  MsgActuator mot[6];     //in
  MsgActuator mot_cmd[6]; //out
  MsgBaseVel cmdvel;
  //int dir;

  char name[100];

  EtherDrive_Node_Controller() : ros::node("etherdrivecontroller")
  {
    for (int i = 0;i < 6;i++) {
      ostringstream oss; // bus
      sprintf(name, "mot%d_cmd", i);
      printf("advertising %s\n", name);
      advertise<MsgActuator>(name);
   }
   for(int i = 0; i < 6; i++){
      sprintf(name, "mot%d", i);     
      subscribe(name, mot[i], &EtherDrive_Node_Controller::displayMotorValues);

      subscribe("cmd_vel",cmdvel,&EtherDrive_Node_Controller::joystickInput); 
    }
  }


  void displayMotorValues() {
    int pos[6];
    
    for(int i = 0;i < 6 ;i++) {
      pos[i] = mot[i].enc;
       if(i == 4) {
 //         printf("Motor %i pos: %i\n",i, pos[i]);
         if(mot[i].enc >= 20000) {
            dir = -1;
        }
        else if(mot[i].enc <= 0) {
            dir = 1;
        }
      }
    }
/*

    int pos[6];
    
    for(int i = 0;i < 6 ;i++) {
      pos[i] = mot[i].enc;
      if(i == 3) {
          printf("Motor %i pos: %i\n",i, pos[i]);
//        if(mot[i].enc <= -600000 | mot[i].enc >= 0) dir = -dir;
        if(mot[i].enc >= 20000) {
            dir = -1;
            if(prevDir == 1) {
              sendMotorCommand(5,0,0,true, 1);
              sendMotorCommand(4,0,0,true, 2);
              sendMotorCommand(3,0,0,true, 2);
              printf("changing direction -1\n");
              sleep(5);  
              prevDir = -1;
            } 


        }
        else if(mot[i].enc <= 0) {
            dir = 1;
            if(prevDir == -1) {
              sendMotorCommand(5,0,0,true, 1);
              sendMotorCommand(4,0,0,true, 2);
              sendMotorCommand(3,0,0,true, 2);
              printf("changing direction +1\n");
              sleep(5);
              prevDir = 1;
            }
            

        }
          
   //       sleep(3);
        
      }

    }
*/


//Close control loop on joint 3
/*
    printf("%d\n", mot[3].enc);
    int command = mot[3].enc * -0.00001;
    if(command < -100)
      command = -100;
    if(command > 100)
      command = 100;
    sendMotorCommand(3, command, 0, true);
*/
  }

  void joystickInput() { 
  float vx;
  float vw;

  vx = 0.1*cmdvel.vx;
  vw = 0.01*cmdvel.vw;
  printf("vx: %f\n", vx);
  printf("vw: %f\n", vw);
  
  sendMotorCommand(0,vw,0,true,0);
  sendMotorCommand(1,vx,0,true,0);
  sendMotorCommand(2,vx,0,true,0);
  
  }

  void sendMotorCommand(int motor, float val, int rel, bool valid, int mode) {
    mot_cmd[motor].val = val;
    mot_cmd[motor].rel= rel;
    mot_cmd[motor].valid= valid;
    mot_cmd[motor].mode = mode;
    
    ostringstream oss;
    oss << "mot" << motor << "_cmd";
    publish(oss.str().c_str(), mot_cmd[motor]);

  }

};

void switchByTime() {

}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  EtherDrive_Node_Controller a;
printf("start\n");
  int fooCounter = 0;
    dir = 1;
    prevDir = 1;
  int controlMode = 1; //2; //0=command directly, 1=reverse by time interval, 2=switch by encoder count
  switch(controlMode) {
    case 0:
      while(a.ok()) {
        char inputstring[512];
        float val = 0.0;
        int motor = 0;
        int mode = 0;
            
        printf("Enter Motor (0-5):\n");
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
    case 1:  
      while(a.ok()) {
        a.sendMotorCommand(0,-200,0,true, 2);
        a.sendMotorCommand(1,200,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,-200,0,true, 2);
        a.sendMotorCommand(4,200,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(5);
        a.sendMotorCommand(0,200,0,true, 2);
        a.sendMotorCommand(1,-200,0,true, 2);
        a.sendMotorCommand(2,0,0,true, 1);
        a.sendMotorCommand(3,200,0,true, 2);
        a.sendMotorCommand(4,-200,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        sleep(5);
      }
      break;
    case 2:
     while (a.ok()) {
       if(dir == 1) {        
        a.sendMotorCommand(0,0,0,true, 1);
        a.sendMotorCommand(1,500,0,true, 2);
        a.sendMotorCommand(2,-500,0,true, 2);
        a.sendMotorCommand(3,500,0,true, 2);
        a.sendMotorCommand(4,500,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
        }
      else { // dir == -1
        a.sendMotorCommand(4,-500,0,true, 2);
        a.sendMotorCommand(3,-500,0,true, 2);
        a.sendMotorCommand(5,0,0,true, 1);
//      a.sendMotorCommand(5,34009,0,true, 1); rotate 180

       }
      }
      usleep(1000);
      break;
    default:
      break;

  }                 

  ros::fini();
  return 0;
}
