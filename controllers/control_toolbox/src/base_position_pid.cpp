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

#include "control_toolbox/base_position_pid.h"
#include "angles/angles.h"                                              // For angular distance error calculation

using namespace control_toolbox ;

BasePositionPid::BasePositionPid()
{

}

BasePositionPid::~BasePositionPid()
{
  
  
}

bool BasePositionPid::initXml(TiXmlElement *config)
{
  TiXmlElement *pid_x_elem = config->FirstChildElement("pid_x") ;
  TiXmlElement *pid_y_elem = config->FirstChildElement("pid_y") ;
  TiXmlElement *pid_w_elem = config->FirstChildElement("pid_w") ;

  if (!pid_x_elem || !pid_y_elem || !pid_w_elem)
  {
    printf("BasePositionController:: Error loading XML\n") ;
    return false ;
  }
  
  bool success_x = pid_x_.initXml(pid_x_elem) ;
  bool success_y = pid_y_.initXml(pid_y_elem) ;
  bool success_w = pid_w_.initXml(pid_w_elem) ;
  
  if (!success_x || !success_y || !success_w)
  {
    printf("BasePositionController:: Error loading pid controllers XML\n") ;
    return false ;    
  }
  
  return true ;
}

tf::Vector3 BasePositionPid::updateControl(const tf::Vector3& commanded_pos, const tf::Vector3& actual_pos, double time_elapsed)
{
  double err_x = actual_pos.x() - commanded_pos.x() ;
  double err_y = actual_pos.y() - commanded_pos.y() ;
  double err_w = angles::shortest_angular_distance(commanded_pos.z(), actual_pos.z()) ;
  
  tf::Vector3 velocity_cmd ;
  
  double odom_cmd_x = pid_x_.updatePid(err_x, time_elapsed) ;            // Translation X in the odometric frame
  double odom_cmd_y = pid_y_.updatePid(err_y, time_elapsed) ;            // Translation Y in the odometric frame

  // Rotate the translation commands so that they're in the base frame (instead of the odom frame)
  velocity_cmd.setX( odom_cmd_x*cos(actual_pos.z()) + odom_cmd_y*sin(actual_pos.z())) ;
  velocity_cmd.setY(-odom_cmd_x*sin(actual_pos.z()) + odom_cmd_y*cos(actual_pos.z())) ;

  velocity_cmd.setZ(pid_w_.updatePid(err_w, time_elapsed)) ;             // Rotation command is same is Odom and Base frames
  
  return velocity_cmd ;
}
