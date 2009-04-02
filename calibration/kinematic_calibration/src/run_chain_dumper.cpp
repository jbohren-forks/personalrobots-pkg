/*
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

// Author: Vijay Pradeep


#include <string>

#include "ros/node.h"
#include "kinematic_calibration/chain_dumper.h"
#include "mechanism_model/robot.h"
#include "tinyxml/tinyxml.h"
#include "hardware_interface/hardware_interface.h"
using namespace std ;

void FancyChain(const string& robot_desc) ;

int main(int argc, char** argv)
{
  ros::init(argc, argv) ;
  ros::Node node("chain_dumper") ;

  printf("Getting robot description from param server...") ;
  fflush(stdout) ;
  string robot_desc ;
  bool success ;
  success = node.getParam("robotdesc/pr2", robot_desc) ;
  if (!success)
  {
    printf("ERROR: Could not access robot_desc/pr2 from param server\n") ;
  }
  else
    printf("Success!\n") ;

  //OldChain(robot_desc) ;
  FancyChain(robot_desc) ;

  //ros::fini() ;
  return 0 ;
}

void FancyChain(const string& robot_desc)
{
  bool success ;

  printf("Parsing robotdesc/pr2...") ;
  fflush(stdout) ;
  TiXmlDocument doc ;
  doc.Parse(robot_desc.c_str()) ;
  printf("Success!\n") ;

  TiXmlElement *root = doc.FirstChildElement("robot");
  if (!root)
  {
    printf("Error finding 'robot' tag in xml\n");
  }

  printf("Initializing Robot...") ;
  fflush(stdout) ;
  mechanism::Robot robot ;
  HardwareInterface hw(0) ;
  robot.hw_ = &hw ;
  robot.initXml(root) ;
  printf("Success!\n") ;

  printf("Displaying All the links:\n") ;
  for (unsigned int i=0; i<robot.links_.size(); i++)
  {
    printf("%02u) %p ", i,  robot.links_[i]) ;
    fflush(stdout) ;
    printf("%s\n", robot.links_[i]->name_.c_str())  ;
  }

  printf("Displaying All the joints:\n") ;
  for (unsigned int i=0; i<robot.joints_.size(); i++)
  {
    printf("%02u) %p ", i,  robot.joints_[i]) ;
    fflush(stdout) ;
    printf("%s\n", robot.joints_[i]->name_.c_str()) ;
  }

  printf("Starting Chain Dumper...\n") ;
  fflush(stdout) ;
  success = kinematic_calibration::ChainDumper::dumpChain(&robot, "torso_lift_link", "r_wrist_roll_link", "./r_arm") ;
//    success = kinematic_calibration::ChainDumper::dumpChain(&robot, "torso_lift_link", "stereo_optical_link", "./head") ;
  if (!success)
    printf("Error dumping chain\n") ;

  printf("Starting Chain Dumper...") ;
  fflush(stdout) ;
  success = kinematic_calibration::ChainDumper::dumpChain(&robot, "torso_lift_link", "stereo_optical_frame", "./head") ;
  if (!success)
    printf("Error dumping chain\n") ;
}
