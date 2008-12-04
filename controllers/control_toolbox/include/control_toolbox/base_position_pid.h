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


#ifndef CONTROL_TOOLBOX_BASE_POSITION_PID_H_
#define CONTROL_TOOLBOX_BASE_POSITION_PID_H_

#include "control_toolbox/pid.h"
#include "tinyxml/tinyxml.h"
#include "tf/transform_datatypes.h"

namespace control_toolbox
{

/**
 * This is a simple math utility to generate velocity commands in order to close the loop around position (and rotation).
 */
class BasePositionPid
{
public:

  BasePositionPid() ;
  ~BasePositionPid() ;
  
  bool initXml(TiXmlElement *config) ;

  /**
   * Determines the base's x,y,w control velocities to try to achieve the commanded position.
   * \param commanded_pos The [x,y,w] position we're trying to acheive
   * \param actual_pos The [x,y,w] position we're currently at
   * \param elapsed_time The time between our current update and the previous update
   */
  tf::Vector3 updateControl(const tf::Vector3& commanded_pos, const tf::Vector3& actual_pos, double elapsed_time) ;
  
private:
  control_toolbox::Pid pid_x_ ;                //!< Input: odom x error.   Output: Odom x velocity command
  control_toolbox::Pid pid_y_ ;                //!< Input: odom y error.   Output: Odom y velocity command
  control_toolbox::Pid pid_w_ ;                //!< Input: ang  w error.   Output: Odom/Base angular velocity command
} ;

}

#endif /* CONTROL_TOOLBOX_BASE_POSITION_PID_H_ */
