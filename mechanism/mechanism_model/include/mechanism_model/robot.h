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

/*
 * The robot model tracks the state of the robot.
 *
 * State path:
 *               +---------------+                +--------+
 * Actuators --> | Transmissions | --> Joints --> | Chains | --> Links
 *               +---------------+                +--------+
 *
 * The actuators, joints, and links, hold the state information.  The
 * actuators hold the encoder info, the joints hold the joint angles
 * and velocities, and the links hold the frame transforms of the body
 * segments.
 *
 * The transmissions and chains are for propagating the state through
 * the model, and they themselves do not hold any information on the
 * robot's current state.
 *
 * Author: Stuart Glaser
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <map>
#include <string>
#include "stl_utils/stl_utils.h"
#include "mechanism_model/link.h"
#include "mechanism_model/joint.h"
#include "mechanism_model/transmission.h"
#include "mechanism_model/chain.h"
#include "hardware_interface/hardware_interface.h"

class TiXmlElement;

namespace mechanism
{

class Robot
{
public:
  Robot(const char *ns){}

  ~Robot()
  {
    deleteElements(&transmissions_);
    deleteElements(&joints_);
    deleteElements(&links_);
  }

  bool initXml(TiXmlElement *root);

  HardwareInterface *hw_;  // Holds the array of actuators
  std::vector<Transmission*> transmissions_;
  std::vector<Joint*> joints_;
  std::vector<Chain*> chains_;
  std::vector<Link*> links_;

  Joint* getJoint(const std::string &name);
  Actuator* getActuator(const std::string &name);
  Link* getLink(const std::string &name);

  // For debugging
  void printLinkTree();
};

}

#endif
