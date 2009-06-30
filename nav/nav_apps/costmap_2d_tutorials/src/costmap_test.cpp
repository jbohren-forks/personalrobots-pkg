/*********************************************************************
*
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include "ros/ros.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "tf/transform_listener.h"

void doSomething(const ros::TimerEvent& e, costmap_2d::Costmap2DROS& costmap_ros){
  //get a copy of the underlying costmap
  costmap_2d::Costmap2D costmap;
  costmap_ros.getCostmapCopy(costmap);

  //do whatever you want with the map here
  ROS_INFO("The size of the map in meters is: (%.2f, %.2f)", costmap.metersSizeX(), costmap.metersSizeY());
}

int main(int argc, char** argv){
  //initialize ROS
  ros::init(argc, argv, "costmap_tester");

  //create a Transform Listener
  tf::TransformListener tf(ros::Duration(10));
  
  //create a ROS wrapper for the costmap, passing it a name and a reference to a TransformListener
  //which will configure itself based on parameters
  costmap_2d::Costmap2DROS costmap_ros("costmap", tf);

  //we'll need a node handle
  ros::NodeHandle n;

  //create a timer for our silly print callback that will call it once a second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(doSomething, _1, boost::ref(costmap_ros)));
  
  //start processing data
  ros::spin();

  return(0);

}

