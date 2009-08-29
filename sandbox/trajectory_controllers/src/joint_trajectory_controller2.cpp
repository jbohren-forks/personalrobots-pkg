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

#include "trajectory_controllers/joint_trajectory_controller2.h"
#include <sstream>
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include "spline_smoother/splines.h"

PLUGINLIB_REGISTER_CLASS(JointTrajectoryController2, controller::JointTrajectoryController2, controller::Controller)

namespace controller {


JointTrajectoryController2::JointTrajectoryController2()
: robot_(NULL)
{
}

JointTrajectoryController2::~JointTrajectoryController2()
{
  sub_command_.shutdown();
}

bool JointTrajectoryController2::init(mechanism::RobotState *robot, const ros::NodeHandle &n)
{
  using namespace XmlRpc;
  node_ = n;
  robot_ = robot;

  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                node_.getNamespace().c_str());
      return false;
    }

    mechanism::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",
                ((std::string)name_value).c_str(), node_.getNamespace().c_str());
      return false;
    }
    joints_.push_back(j);
  }

  // Sets up pid controllers for all of the joints
  std::string gains_ns;
  if (!node_.getParam("gains", gains_ns))
    gains_ns = node_.getNamespace() + "/gains";
  pids_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i]->joint_->name_)))
      return false;

  sub_command_ = node_.subscribe("command", 1, &JointTrajectoryController2::commandCB, this);

  q.resize(joints_.size());
  qd.resize(joints_.size());
  qdd.resize(joints_.size());

  char buf[64];
  for (size_t i = 0; i < q.size(); ++i)
  {
    sprintf(buf, "q%d", i);   recorder_.channel(QS + 3*i + 0, buf);
    sprintf(buf, "qd%d", i);  recorder_.channel(QS + 3*i + 1, buf);
    sprintf(buf, "qdd%d", i); recorder_.channel(QS + 3*i + 2, buf);
  }
  recorder_.init(robot_, node_);

  return true;
}

bool JointTrajectoryController2::starting()
{
  last_time_ = robot_->getTime();
  return true;
}

void JointTrajectoryController2::update()
{
  // Checks if all the joints are calibrated.

  ros::Time time = robot_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  // Is it time to advance to the next trajectory?

  SpecifiedTrajectory &traj = incoming_trajectory_.next();

  // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
  int seg = -1;
  while (seg + 1 < (int)traj.size() &&
         traj[seg+1].start_time < time.toSec())
  {
    ++seg;
  }
  if (seg == -1)
    return;

  // ------ Trajectory Generation

  for (size_t i = 0; i < q.size(); ++i)
  {
    sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration,
                               time.toSec() - traj[seg].start_time,
                               q[i], qd[i], qdd[i]);
  }

  for (size_t i = 0; i < q.size(); ++i)
  {
    recorder_.record(QS + 3*i + 0, q[i]);
    recorder_.record(QS + 3*i + 1, qd[i]);
    recorder_.record(QS + 3*i + 2, qdd[i]);
  }

  // ------ Trajectory Following

  for (size_t i = 0; i < joints_.size(); ++i)
  {
    double error = 0.0;
    switch (joints_[i]->joint_->type_)
    {
    case mechanism::JOINT_ROTARY:
      angles::shortest_angular_distance_with_limits(
        q[i], joints_[i]->position_,
        joints_[i]->joint_->joint_limit_min_, joints_[i]->joint_->joint_limit_max_, error);
      break;
    case mechanism::JOINT_CONTINUOUS:
      error = angles::shortest_angular_distance(q[i], joints_[i]->position_);
      break;
    case mechanism::JOINT_PRISMATIC:
      error = joints_[i]->position_ - q[i];
      break;
    default:
      ROS_FATAL("Joint type: %d", joints_[i]->joint_->type_);
    }

    joints_[i]->commanded_effort_ += pids_[i].updatePid(error, joints_[i]->velocity_ - qd[i], dt);
  }
}

void JointTrajectoryController2::commandCB(const trajectory_controllers::TrajectoryConstPtr &msg)
{
  ros::Time time = last_time_;
  ROS_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf",
            time.toSec(), msg->header.stamp.toSec());

  SpecifiedTrajectory new_traj;

  // Makes sure the realtime process is using the latest trajectory
  boost::recursive_mutex::scoped_lock guard(incoming_trajectory_lock_);
  while (incoming_trajectory_.has_next())
    usleep(1000);
  SpecifiedTrajectory &prev_traj = incoming_trajectory_.next();

  // ------ Copies over the segments from the previous trajectory that are still useful.

  for (size_t i = 0; i < prev_traj.size(); ++i)
  {
    // If this segment lasts beyond the current time and starts before the new trajectory.
    if (prev_traj[i].start_time + prev_traj[i].duration > time.toSec() &&
        prev_traj[i].start_time < msg->header.stamp.toSec())
    {
      new_traj.push_back(prev_traj[i]);

      ROS_DEBUG("Saving segment %2d: %.3lf for %.3lf",
                i, prev_traj[i].start_time, prev_traj[i].duration);
    }
    else
    {
      ROS_DEBUG("Removing segment %2d: %.3lf for %.3lf",
                i, prev_traj[i].start_time, prev_traj[i].duration);
    }
  }

  // ------ Determines when and where the new segments start

  double start_time = msg->header.stamp.toSec();

  // Finds the end conditions of the final segment
  Segment &last = new_traj[new_traj.size() - 1];
  std::vector<double> prev_positions(joints_.size());
  std::vector<double> prev_velocities(joints_.size());
  std::vector<double> prev_accelerations(joints_.size());

  for (size_t i = 0; i < joints_.size(); ++i)
  {
    sampleSplineWithTimeBounds(last.splines[i].coef, last.duration,
                               start_time,
                               prev_positions[i], prev_velocities[i], prev_accelerations[i]);
  }

  // ------ Tacks on the new segments

  for (size_t i = 0; i < msg->points.size(); ++i)
  {
    Segment seg;

    seg.start_time = start_time;
    seg.duration = msg->points[i].duration;
    seg.splines.resize(joints_.size());

    //! \todo Need to use the "names" field in the message to reorder the splines

    for (size_t j = 0; j < joints_.size(); ++i)
    {
      if (prev_accelerations.size() > 0 && msg->points[i].accelerations.size() > 0)
      {
        spline_smoother::getQuinticSplineCoefficients(
          prev_positions[j], prev_velocities[j], prev_accelerations[j],
          msg->points[i].positions[j], msg->points[i].velocities[j], msg->points[i].accelerations[j],
          msg->points[i].duration,
          seg.splines[j].coef);
      }
      else
      {
        spline_smoother::getCubicSplineCoefficients(
          prev_positions[j], prev_velocities[j],
          msg->points[i].positions[j], msg->points[i].velocities[j],
          msg->points[i].duration,
          seg.splines[j].coef);
      }
    }

    new_traj.push_back(seg);

    // Computes the starting conditions for the next segment
    start_time += msg->points[i].duration;
    prev_positions = msg->points[i].positions;
    prev_velocities = msg->points[i].velocities;
    prev_accelerations = msg->points[i].accelerations;
  }

  // Sends the new set of segments to the realtime process
  incoming_trajectory_.set(new_traj);

  ROS_DEBUG("The new trajectory has %d segments", new_traj.size());
}

void JointTrajectoryController2::sampleSplineWithTimeBounds(
  std::vector<double>& coefficients, double duration, double time,
  double& position, double& velocity, double& acceleration)
{
  if (time < 0)
  {
    double _;
    spline_smoother::sampleQuinticSpline(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else if (time > duration)
  {
    double _;
    spline_smoother::sampleQuinticSpline(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    spline_smoother::sampleQuinticSpline(coefficients, time,
                                         position, velocity, acceleration);
  }
}

}
