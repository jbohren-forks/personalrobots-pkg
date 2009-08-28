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

#ifndef DALLAS_CONTROLLER_H
#define DALLAS_CONTROLLER_H

#include "ros/ros.h"
#include "pr2_mechanism_controllers/caster_controller.h"
#include "realtime_tools/realtime_infuser.h"
#include "geometry_msgs/Twist.h"

namespace controller {

class DallasController : public Controller
{
public:
  DallasController();
  ~DallasController();

  virtual bool init(mechanism::RobotState *robot, const ros::NodeHandle &n);

  virtual bool starting();
  virtual void update();

  void setCommand(double vx, double va) {
    Command c;
    c.vx = vx;
    c.va = va;
    c.received_time = last_time_;
    infuser_.set(c);
  }

private:
  mechanism::RobotState *robot_;
  ros::NodeHandle node_;

  ros::Time last_time_;
  ros::Subscriber sub_command_;
  void command(const geometry_msgs::TwistConstPtr &msg);

  // For pushing commands into realtime atomically
  struct Command {
    double vx, va;
    ros::Time received_time;
  };
  realtime_tools::RealtimeInfuser<Command> infuser_;

  double kp_caster_steer_;

  CasterController cc_;
};

} // namespace

#endif
