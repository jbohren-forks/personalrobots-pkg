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
 * Example config:

 <controller type="CartesianVelocityController" name="controller_name">
   <chain root="root_link" tip="tip_link" />
 </controller>

 * The root is fixed, and all commands are specified in its coordinate
 * frame.
 *
 * Author: Stuart Glaser
 */

#ifndef CARTESIAN_VELOCITY_CONTROLLER_H
#define CARTESIAN_VELOCITY_CONTROLLER_H


#include <vector>
#include "ros/node.h"
#include "robot_srvs/SetVector.h"
#include "robot_srvs/GetVector.h"
#include "robot_mechanism_controllers/cartesian_effort_controller.h"
#include "control_toolbox/pid.h"
#include "mechanism_model/controller.h"
#include "tf/transform_datatypes.h"
#include "misc_utils/advertised_service_guard.h"
#include "misc_utils/subscription_guard.h"
#include "misc_utils/realtime_publisher.h"
#include "misc_utils/realtime_infuser.h"

namespace controller {

class CartesianVelocityController : public Controller
{
public:
  CartesianVelocityController();
  ~CartesianVelocityController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();

  //tf::Vector3 command_;
  misc_utils::RealtimeInfuser<tf::Vector3> command_;
  void getTipVelocity(tf::Vector3 *v);

private:
  mechanism::RobotState *robot_;
  mechanism::LinkState *tip_;
  CartesianEffortController effort_;
  control_toolbox::Pid pid_x_, pid_y_, pid_z_;
  double last_time_;
};

class CartesianVelocityControllerNode : public Controller
{
public:
  CartesianVelocityControllerNode();
  ~CartesianVelocityControllerNode();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();

  bool setCommand(robot_srvs::SetVector::request &req,
                  robot_srvs::SetVector::response &resp);
  bool getActual(robot_srvs::GetVector::request &req,
                 robot_srvs::GetVector::response &resp);
  void command();

private:
  CartesianVelocityController c_;
  AdvertisedServiceGuard guard_set_command_, guard_get_actual_;
  SubscriptionGuard guard_command_;

  std_msgs::Vector3 command_msg_;

  misc_utils::RealtimePublisher<std_msgs::Vector3> *vel_publisher_;
  int loop_count_;
};

}

#endif
