/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "dallas_controller/dallas_controller.h"
#include "pluginlib/class_list_macros.h"
#include "pr2_controller_interface/controller.h"
#include <angles/angles.h>


namespace controller {

const double DALLAS_R = 0.5;

DallasController::DallasController()
  : robot_(NULL), last_time_(0)
{
  setCommand(0, 0);
}

DallasController::~DallasController()
{
  sub_command_.shutdown();
}

bool DallasController::init(pr2_mechanism::RobotState *robot, const ros::NodeHandle &n)
{
  robot_ = robot;
  node_ = n;

  if (!cc_.init(robot, node_))
    return false;

  node_.param<double> ("kp_caster_steer", kp_caster_steer_, 40.0);

  sub_command_ = node_.subscribe<geometry_msgs::Twist>(
    "cmd_vel", 1, &DallasController::command, this);

  return true;
}

bool DallasController::starting()
{
  if (!cc_.starting())
    return false;
  return true;
}

void DallasController::update()
{
  ros::Time time = robot_->getTime();

  Command &command = infuser_.next();
  if (true || command.received_time <= time + ros::Duration(1.0))
  {
    // Where should the caster be facing?
    double caster_angle = cc_.getSteerPosition();
    double caster_angle_desi = atan2(-DALLAS_R * command.va, command.vx);
    double direction = 1.0;
    if (fabs(angles::shortest_angular_distance(caster_angle_desi, caster_angle)) >
        fabs(angles::shortest_angular_distance(caster_angle_desi + M_PI, caster_angle)))
    {
      caster_angle_desi += M_PI;
      direction = -1.0;
    }
    double caster_angle_error =
      angles::shortest_angular_distance(caster_angle_desi, caster_angle);
    //ROS_ERROR("Angle desi = %.3lf  (From %.3lf)  Error: %.3lf", caster_angle_desi, cc_.getSteerPosition(), caster_angle_error);

    cc_.steer_velocity_ = -kp_caster_steer_ * caster_angle_error;
    cc_.drive_velocity_ = -direction * cos(caster_angle_error) *
      sqrt(pow(command.vx, 2.0) + pow(DALLAS_R * command.va, 2.0));
    //ROS_ERROR("Drive: %.3lf, %.3lf", command.vx, cos(caster_angle_error));
    //ROS_ERROR("Vels: %.3lf, %.3lf", cc_.steer_velocity_, cc_.drive_velocity_);
    //cc_.steer_velocity_ = 0.0;
    //cc_.drive_velocity_ = 0.0;
    cc_.update();
  }

  last_time_ = time;
}

void DallasController::command(const geometry_msgs::TwistConstPtr &msg)
{
  Command c;
  c.received_time = last_time_;
  c.vx = msg->linear.x;
  c.va = msg->angular.z;
  ROS_ERROR("Command: %.3lf   (%.3lf, %.3lf)", c.received_time.toSec(), c.vx, c.va);
  infuser_.set(c);
}

} // namespace

using namespace controller;

PLUGINLIB_REGISTER_CLASS(DallasController, DallasController, Controller)


