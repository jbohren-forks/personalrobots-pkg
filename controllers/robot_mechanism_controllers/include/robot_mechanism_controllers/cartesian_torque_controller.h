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
 * Author: Stuart Glaser
 */

#ifndef CARTESIAN_TORQUE_CONTROLLER_H
#define CARTESIAN_TORQUE_CONTROLLER_H

#include <vector>
#include "ros/node.h"
#include "mechanism_model/controller.h"
#include "tf/transform_datatypes.h"
#include "misc_utils/advertised_service_guard.h"
#include "robot_srvs/SetVector.h"

namespace controller {

class CartesianTorqueController : public Controller
{
public:
  CartesianTorqueController();
  ~CartesianTorqueController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();

  tf::Vector3 command_;

  std::vector<mechanism::LinkState*> links_;  // root to tip
  std::vector<mechanism::JointState*> joints_;
};


class CartesianTorqueControllerNode :  public Controller
{
public:
  CartesianTorqueControllerNode();
  ~CartesianTorqueControllerNode();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();

  bool setCommand(robot_srvs::SetVector::request &req,
                  robot_srvs::SetVector::response &resp);
  void command();

private:
  CartesianTorqueController c_;
  AdvertisedServiceGuard guard_set_command_;
};

}

#endif
