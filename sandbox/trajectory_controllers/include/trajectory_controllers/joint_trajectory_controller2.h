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

/*!
  \author Stuart Glaser

  \class controller::JointTrajectoryController2

  Example config:<br>
    <controller name>:
      type: "JointTrajectoryController2"
      joints:
        r_shoulder_pan_joint: {p: 140.0, d: 30.0}
        r_shoulder_lift_joint: {p: 170.0, d: 30.0}
        r_upper_arm_roll_joint: {p: 300.0, d: 7.0}
*/

#ifndef JOINT_TRAJECTORY_CONTROLLER2_H
#define JOINT_TRAJECTORY_CONTROLLER2_H

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/node_handle.h>
#include <control_toolbox/pid.h>
#include <pr2_mechanism_control/recorder.h>
#include <pr2_controller_interface/controller.h>
#include <realtime_tools/realtime_infuser.h>

#include "trajectory_controllers/Trajectory.h"

namespace controller {

class JointTrajectoryController2 : public Controller
{
public:

  JointTrajectoryController2();
  ~JointTrajectoryController2();

  bool init(mechanism::RobotState *robot, const ros::NodeHandle &n);

  bool starting();
  void update();

private:
  mechanism::RobotState *robot_;
  ros::Time last_time_;
  std::vector<mechanism::JointState*> joints_;
  std::vector<control_toolbox::Pid> pids_;

  ros::NodeHandle node_;

  void commandCB(const trajectory_controllers::TrajectoryConstPtr &msg);
  ros::Subscriber sub_command_;

  // ------ Mechanisms for passing the trajectory into realtime

  // coef[0] + coef[1]*t + ... + coef[5]*t^5
  struct Spline
  {
    std::vector<double> coef;

    Spline() : coef(6, 0.0) {}
  };

  struct Segment
  {
    double start_time;
    double duration;
    std::vector<Spline> splines;
  };

  typedef std::vector<Segment> SpecifiedTrajectory;
  realtime_tools::RealtimeInfuser<SpecifiedTrajectory> incoming_trajectory_;
  boost::recursive_mutex incoming_trajectory_lock_;  // For non-realtime only

  std::vector<double> q, qd, qdd;  // Preallocated in init

  enum { QS };
  mechanism::Recorder recorder_;

  // Samples, but handling time bounds.  When the time is past the end
  // of the spline duration, the position is the last valid position,
  // and the derivatives are all 0.
  static void sampleSplineWithTimeBounds(std::vector<double>& coefficients, double duration, double time,
                                         double& position, double& velocity, double& acceleration);
};

}

#endif
