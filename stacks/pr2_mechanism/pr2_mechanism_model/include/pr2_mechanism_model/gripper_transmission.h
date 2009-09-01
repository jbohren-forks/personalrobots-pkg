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

/*
 * <transmission type="GripperTransmission" name="gripper_l_transmission">
 *   <actuator name="gripper_l_motor" />
 *   <joint name="gripper_l_upper1_joint" reduction="4" />
 *   <joint name="gripper_l_lower1_joint" reduction="-4" />
 *   <joint name="gripper_l_upper2_joint" reduction="-8" />
 *   <joint name="gripper_l_lower2_joint" reduction="8" />
 *   <!-- GripTransmission uses a PID controller to keep the joint angles aligned in Gazebo -->
 *   <pid p="1.0" i="2.0" d="3.0" iClamp="2.0" /> <!-- Only needed for Gazebo -->
 * </transmission>
 *
 * Author: Stuart Glaser
 */

#ifndef GRIPPER_TRANSMISSION_H
#define GRIPPER_TRANSMISSION_H

#include <vector>
#include "tinyxml/tinyxml.h"
#include "pr2_mechanism_model/transmission.h"
#include "pr2_mechanism_model/robot.h"

namespace pr2_mechanism {

class GripperTransmission : public Transmission
{
public:
  GripperTransmission() : A_(1), B_(0) {}
  virtual ~GripperTransmission() {}

  bool initXml(TiXmlElement *config, Robot *robot);

  void propagatePosition(std::vector<Actuator*>&, std::vector<JointState*>&);
  void propagatePositionBackwards(std::vector<JointState*>&, std::vector<Actuator*>&);
  void propagateEffort(std::vector<JointState*>&, std::vector<Actuator*>&);
  void propagateEffortBackwards(std::vector<Actuator*>&, std::vector<JointState*>&);

  std::vector<double> preductions_, ereductions_;  // Mechanical reduction for each joint, different for position and effort

private:
  double A_, B_; // gripper angle = reduction*acos(A*motor+B)
};

} // namespace pr2_mechanism

#endif
