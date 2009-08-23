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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef SET_HOKUYO_MODE_H
#define SET_HOKUYO_MODE_H

#include <ros/node.h>
#include <ros/ros.h>

namespace pr2_robot_actions {
  static bool setHokuyoMode(std::string node_name, std::string mode){
    return true;
    /*
    ros::NodeHandle n;
    if(mode == "intensity"){
      n.setParam(node_name + "/intensity", true);
      n.setParam(node_name + "/skip", 1);
    }
    else if(mode == "navigate"){
      n.setParam(node_name + "/intensity", false);
      n.setParam(node_name + "/skip", 0);
    }
    else{
      ROS_ERROR("The only supported modes are intensity and navigate, please pick one");
      return false;
    }

    //ask the hokuyo node to reconfigure
    n.setParam(node_name + "/reconfigure", true);

    ros::Rate r(100.0);
    bool reconfigure;
    while(n.getParam(node_name + "reconfigure", reconfigure, true)){
      r.sleep();
    }

    ROS_INFO("Laser mode successfully set");
    return true;
    */
  }
}

#endif
