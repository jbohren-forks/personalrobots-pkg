/*********************************************************************
* This node takes OpenCV images and tracks blobs 
*
*
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Jimmy Sastra
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


#include "ros/node.h"
#include "std_msgs/Blob.h"
#include "std_msgs/BaseVel.h"
//#include "elphel_cam/Blob.h"
#include "math_utils/angles.h"
#include <genericControllers/Pid.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace controller;
float targetPos[3] = {0, 0, 0}; // x, y, theta

double cmd[3] = {0, 0, 0};
  Pid pidController[3];
  float max_vx = 0.5;
  float max_vy = 0.5;
  float max_vw = 0.5;

class Blob_Tracker_Node_Test : public ros::node
{


public:

  std_msgs::Blob blob_msg;
  std_msgs::BaseVel base_vel_msg;
  
  Blob_Tracker_Node_Test() : ros::node("blobtrackertest") {

    subscribe("blob",blob_msg, &Blob_Tracker_Node_Test::printBaseCoords); 
    base_vel_msg.vx = 0;
    base_vel_msg.vy = 0;
    base_vel_msg.vw = 0;
    advertise<std_msgs::BaseVel>("base_command");  
  }

  virtual ~Blob_Tracker_Node_Test() {

  }


  void printBlobCoords() // in raw pixel coords 
  {
    for(int i=0; i < blob_msg.get_x_size(); i++) {
      printf("i: %i, x: %f, y: %f\n", i, blob_msg.x[i], blob_msg.y[i]);
    }
  } 

  void printBaseCoords() // label only 2 blobs: first blob as tail of the base, second blob as the head
  {
    float actPos[3]; // x, y, theta
    actPos[0] =  (blob_msg.x[0] + blob_msg.x[1])/2;
    actPos[1] =  (blob_msg.y[0] + blob_msg.y[1])/2;
    actPos[2] = atan2((blob_msg.y[1] - blob_msg.y[0]), (blob_msg.x[1] - blob_msg.x[0]));
    printf("actPos:: x: %f, y: %f, w: %f     ", actPos[0], actPos[1], actPos[2]);
    float baseTargetPos[3]; // x, y, theta in base coords 
    transform(targetPos, actPos, baseTargetPos);

    double dummy_dt = 1; // only using proportional so don't need dt
    for ( int i = 0; i < 3; i++ ) {
      cmd[i] = pidController[i].UpdatePid(double( - baseTargetPos[i]), dummy_dt); 
    }

    setBaseVel(cmd[0], cmd[1], cmd[2]);   
   
  } 
  
  void transform(const float* targetPos, const float* actPos,  float* baseTargetPos) {
    // transforms targetPos in real world frame to base frame according to actPos
    // translation:
    float x, y;
    x = targetPos[0]-actPos[0];
    y = targetPos[1]-actPos[1];
    


    // rotation:
    baseTargetPos[0] = cos(actPos[2])*x + sin(actPos[2])*y;
    baseTargetPos[1] = -sin(actPos[2])*x + cos(actPos[2])*y;
    baseTargetPos[2] = targetPos[2]-actPos[2];
    //printf("baseTargetPos::  x: %f, y: %f, w: %f\n", baseTargetPos[0], baseTargetPos[1], baseTargetPos[2]);
 
  }

  void setBaseVel(float vx, float vy, float vw) {
    // printf("vx: %f, vy: %f, vw: %f\n", clamp(vx, max_vx),clamp(vy, max_vy),clamp(vw, max_vw));
     printf("vx: %f, vy: %f, vw: %f\n", vx, vy, vw);
     base_vel_msg.vx = clamp(vx, max_vx);
     base_vel_msg.vy = clamp(vy, max_vy);
     base_vel_msg.vw = clamp(vw, max_vw);
     printf("publishing\n");
     publish("base_command", base_vel_msg);
  }


  float clamp (float term, float limit) {
    if(term > limit ) term = limit;
    if(term < -limit) term = - limit;
    return term;
  }
};


int main(int argc, char **argv) 
{
  
  int counter = 0;
  ros::init(argc, argv);
  Blob_Tracker_Node_Test n;
  string line;
  ifstream myfile;
  double pGain = 0.1;
  double iGain = 0;
  double dGain = 0;
  double iMax = 0;
  double iMin = 0;
 
  for ( int i = 0; i < 2; i++ ) {
    pidController[i].InitPid(pGain,iGain,dGain,iMax,iMin);
  }
  pidController[2].InitPid(0.3,iGain,dGain,iMax,iMin);
  
  while(n.ok()) {
    if ( counter++>100 ) {
      myfile.open("targetPos.txt");
      if (myfile.is_open())
	{
	  int i = 0;
	  while (! myfile.eof() )
	    {
	      getline (myfile,line);
	      targetPos[i] = strtod(line.c_str(), NULL);
	      i++;
	    }
	  myfile.close();
	}

      else cout << "Unable to open file"; 
      counter = 0;
    }
    usleep(1000);      
  }
  ros::fini();
  return 0;
}
