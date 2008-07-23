
/*
 * BaseTest
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

//Base test

#include <ros/node.h> //just for ros::init
#include <sstream>

#include <etherdrive_hardware/etherdrive_hardware.h>
#include <mechanism_control/base_control.h>

#include <sys/time.h>
#include <signal.h>

bool notDone = true;

void finalize(){
   notDone = false;
}

int main(int argc, char *argv[]){
   ros::init(argc, argv); //Everyone else expects ros::init to be already called
   ros::node *node = new ros::node("mechanism_control"); //Can be found by ros::node *ros::g_node for now.  Will eventually be superceded by ros::peer interface

   //set up signals
  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

   //For now possibly hard-code EtherDrive base parameters here
  int numBoards = 2;
  int numActuators = 12;
  //  int boardLookUp[] ={0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1}; 
  // int portLookUp[] = {0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};

  int boardLookUp[] = {0,0,0,1,1,1,0,0,0,1,1,1};
  int portLookUp[]  = {2,0,1,2,1,0,5,3,4,5,4,3};

  //int jointId[]={1,2, 0, 7, 8, 6, 4, 5, 3, 10, 11, 9};
  int jointId[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  string etherIP[] = {"10.12.0.103", "10.11.0.102"};
  string hostIP[] = {"10.12.0.2", "10.11.0.3"};
  
  EtherdriveHardware h(numBoards, numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP);
   //h.init("BaseEtherdrive.xml"); //Should this be a command-line argument?

   //mc.init(h.hardwareInterface, char *namespace or char *init.xml);
   MechanismControl mc;
   mc.init(h.hw); //If not hard-coded, this is where the ROS namespace or configuration file would be passed in
   mc.controller->setVelocity(-0,0,0);


   //Realtime loop would spawn a thread and make it realtime to run this loop

   while(notDone){ //Decide if this is how we want to do this
      h.update();
      mc.update();
      //read out fingertip sensor data from etherCAT hardware
      usleep(1000); //clock_nanosleep();
   }

   //mc.fini();   //If needed
   //h.fini();    //If needed

   delete(node); //Before ros::fini

   ros::fini();

   return 0;
}
