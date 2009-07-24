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

#include <experimental_controllers/joint_trajectory_controller2.h>
#include <sstream>
#include <angles/angles.h>

namespace controller {

ROS_REGISTER_CONTROLLER(JointTrajectoryController2)

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
  last_time_ = robot_->hw_->current_time_;
  return true;
}

void JointTrajectoryController2::update()
{

  // Checks if all the joints are calibrated.

  double time = robot_->hw_->current_time_;
  double dt = time - last_time_;
  last_time_ = time;

  // Is it time to advance to the next trajectory?

  SpecifiedTrajectory &traj = incoming_trajectory_.next();

  // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
  int seg = -1;
  while (seg + 1 < (int)traj.start_times.size() &&
         traj.start_times[seg + 1] < time)
  {
    ++seg;
  }
  if (seg == -1)
    return;

  // ------ Trajectory Generation

  if (traj.start_times[seg] + traj.segments[seg].duration.toSec() < time)
  {
    // Done with this segment, but not ready for the next one.  Hold here.
    double t = traj.segments[seg].duration.toSec();
    for (size_t i = 0; i < q.size(); ++i)
    {
      q[i] =
        traj.segments[seg].a[i] +
        traj.segments[seg].b[i] * t +
        traj.segments[seg].c[i] * pow(t, 2) +
        traj.segments[seg].d[i] * pow(t, 3) +
        traj.segments[seg].e[i] * pow(t, 4) +
        traj.segments[seg].f[i] * pow(t, 5);
      qd[i] = 0;
      qdd[i] = 0;
    }
  }
  else
  {
    double t = time - traj.start_times[seg];
    for (size_t i = 0; i < q.size(); ++i)
    {
      q[i] =
        traj.segments[seg].a[i] +
        traj.segments[seg].b[i] * t +
        traj.segments[seg].c[i] * pow(t, 2) +
        traj.segments[seg].d[i] * pow(t, 3) +
        traj.segments[seg].e[i] * pow(t, 4) +
        traj.segments[seg].f[i] * pow(t, 5);
      qd[i] =
        traj.segments[seg].b[i] +
        2 * traj.segments[seg].c[i] * t +
        3 * traj.segments[seg].d[i] * pow(t, 2) +
        4 * traj.segments[seg].e[i] * pow(t, 3) +
        5 * traj.segments[seg].f[i] * pow(t, 4);
      qdd[i] =
        2 * traj.segments[seg].c[i] +
        6 * traj.segments[seg].d[i] * t +
        12 * traj.segments[seg].e[i] * pow(t, 2) +
        20 * traj.segments[seg].f[i] * pow(t, 3);
    }
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

void JointTrajectoryController2::commandCB(const manipulation_msgs::SplineTrajConstPtr &msg)
{
  double time = last_time_;
  ROS_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf",
            time, msg->header.stamp.toSec());

  SpecifiedTrajectory new_traj;

  // Makes sure the realtime process is using the latest trajectory
  while (incoming_trajectory_.has_next())
    usleep(1000);
  SpecifiedTrajectory &prev_traj = incoming_trajectory_.next();

  // Copies over the segments from the previous trajectory that are still useful.
  for (size_t i = 0; i < prev_traj.start_times.size(); ++i)
  {
    // If this segment lasts beyond the current time and starts before the new trajectory.
    if (prev_traj.start_times[i] + prev_traj.segments[i].duration.toSec() > time &&
        prev_traj.start_times[i] < msg->header.stamp.toSec())
    {
      new_traj.start_times.push_back(prev_traj.start_times[i]);
      new_traj.segments.push_back(prev_traj.segments[i]);

      ROS_DEBUG("Saving segment %2d: %.3lf for %.3lf",
                i, prev_traj.start_times[i], prev_traj.segments[i].duration.toSec());
    }
    else
    {
      ROS_DEBUG("Removing segment %2d: %.3lf for %.3lf",
                i, prev_traj.start_times[i], prev_traj.segments[i].duration.toSec());
    }
  }

  // Tacks on the new segments
  double start_time = msg->header.stamp.toSec();
  for (size_t i = 0; i < msg->segments.size(); ++i)
  {
    new_traj.start_times.push_back(start_time);
    new_traj.segments.push_back(msg->segments[i]);

    // Makes sure the trajectory is quintic
    int n = new_traj.segments.size() - 1;
    if (new_traj.segments[n].a.size() != joints_.size())
      new_traj.segments[n].a.resize(joints_.size(), 0.0);
    if (new_traj.segments[n].b.size() != joints_.size())
      new_traj.segments[n].b.resize(joints_.size(), 0.0);
    if (new_traj.segments[n].c.size() != joints_.size())
      new_traj.segments[n].c.resize(joints_.size(), 0.0);
    if (new_traj.segments[n].d.size() != joints_.size())
      new_traj.segments[n].d.resize(joints_.size(), 0.0);
    if (new_traj.segments[n].e.size() != joints_.size())
      new_traj.segments[n].e.resize(joints_.size(), 0.0);
    if (new_traj.segments[n].f.size() != joints_.size())
      new_traj.segments[n].f.resize(joints_.size(), 0.0);

    start_time += msg->segments[i].duration.toSec();
  }

  // Sends the new set of segments to the realtime process
  incoming_trajectory_.set(new_traj);

  ROS_DEBUG("The new trajectory has %d segments", new_traj.segments.size());
}

}
