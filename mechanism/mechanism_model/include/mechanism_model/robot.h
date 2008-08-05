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
//The robot model is populated by the control code infrastructure and used by all the controllers to read mechanism state and command mechanism motion.

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <map>
#include <string>
#include "stl_utils/stl_utils.h"
#include "mechanism_model/link.h"
#include "mechanism_model/joint.h"
#include "mechanism_model/transmission.h"
#include "hardware_interface/hardware_interface.h"

namespace mechanism
{

class Robot
{
public:
  Robot(char *ns){}

  ~Robot()
  {
    deleteElements(&transmissions_);
    deleteElements(&joints_);
  }

  std::vector<Joint*> joints_;
  std::vector<Transmission*> transmissions_;

  // Supports looking up joints and actuators by name.  The IndexMap
  // structure maps the name of the item to its index in the vectors.
  typedef std::map<std::string,int> IndexMap;
  IndexMap joints_lookup_;
  IndexMap actuators_lookup_;
  Joint* getJoint(const std::string &name)
  {
    IndexMap::iterator it = joints_lookup_.find(name);
    if (it == joints_lookup_.end())
      return NULL;
    return joints_[it->second];
  }
  Actuator* getActuator(const std::string &name)
  {
    IndexMap::iterator it = actuators_lookup_.find(name);
    if (it == actuators_lookup_.end())
      return NULL;
    return hw_->actuators_[it->second];
  }

  HardwareInterface *hw_;
};

}

#endif
