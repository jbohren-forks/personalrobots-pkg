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

 Example config:

 <controller type="CasterController" name="a_caster">
   <joints caster="caster_joint" wheel_l="wheel_left_joint" wheel_r="wheel_right_joint">
   <caster_pid p="5.0" i="0.0" d="0.0" iClamp="0.0" />
   <wheel_pid p="4.0" i="0.0" d="0.0" iClamp="0.0" />
 </controller>
 */
#ifndef CASTER_CONTROLLER_H
#define CASTER_CONTROLLER_H

#include "mechanism_model/controller.h"
#include "mechanism_model/robot.h"
#include "control_toolbox/pid.h"
#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "misc_utils/subscription_guard.h"
#include "std_msgs/Float64.h"

namespace controller {

class CasterController : public Controller
{
public:
  const static double WHEEL_RADIUS;
  const static double WHEEL_OFFSET;

  CasterController();
  ~CasterController();

  bool init(mechanism::RobotState *robot_state,
            const std::string &caster_joint,
            const std::string &wheel_l_joint, const std::string &wheel_r_joint,
            const control_toolbox::Pid &caster_pid, const control_toolbox::Pid &wheel_pid);
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  void update();

  double steer_velocity_;
  double drive_velocity_;

  double getSteerPosition() { return caster_->position_; }
  double getSteerVelocity() { return caster_->velocity_; }

private:
  mechanism::JointState *caster_;
  JointVelocityController caster_vel_, wheel_l_vel_, wheel_r_vel_;
};


/*
 * Listens on /name/steer_velocity and /name/drive_velocity
 */

class CasterControllerNode : public Controller
{
public:
  CasterControllerNode();
  ~CasterControllerNode();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  void update();

  void setSteerVelocity() {
    c_.steer_velocity_ = steer_velocity_msg_.data;
  }
  void setDriveVelocity() {
    c_.drive_velocity_ = drive_velocity_msg_.data;
  }

private:
  CasterController c_;

  SubscriptionGuard guard_steer_velocity_, guard_drive_velocity_;
  std_msgs::Float64 steer_velocity_msg_, drive_velocity_msg_;
};

}

#endif
