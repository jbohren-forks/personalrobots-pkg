/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
/*
 * agilent_node.cpp
 *
 *  Created on: Jul 14, 2009
 *      Author: piccoli
 */

#include <pr2_gripper_controller/LxiInterface.h>
#include <pr2_gripper_controller/Agilent34410A.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <ros/node.h>


std::string ip_address_;
std_msgs::Float64 measurment;
double hz_;

int main(int argc, char** argv)
{
  ros::init(argc, argv);
  ros::Node n("agilent");
  ros::Node::instance()->param<std::string>("ip_address", ip_address_, "10.0.1.68");
  ros::Node::instance()->param<double>("hz", hz_, 10.0);
  ros::Rate loop_rate(hz_);
  LxiInterface* lxi_interface = new LxiInterface(ip_address_);
  lxi_interface->Open();
  Agilent34410A* agilent = new Agilent34410A(*lxi_interface, 0);
  agilent->Reset();
  agilent->Initialize();
  agilent->setMode(agilent->VOLTAGE);
  ros::Node::instance()->advertise<std_msgs::Float64> ("agilent_measure", 1);

  while(n.ok())
  {
    measurment.data = agilent->Measure();
    ros::Node::instance()->publish("agilent_measure", measurment);
    loop_rate.sleep();
  }

  return (0);
}

