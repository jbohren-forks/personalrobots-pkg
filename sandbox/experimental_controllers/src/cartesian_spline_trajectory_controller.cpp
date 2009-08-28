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

// Original version: Sachin Chitta <sachinc@willowgarage.com>, Mrinal Kalakrishnan (kalakris@willowgarage.com)

#include "experimental_controllers/cartesian_spline_trajectory_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(CartesianSplineTrajectoryController);

CartesianSplineTrajectoryController::CartesianSplineTrajectoryController():num_joints_(0),num_controllers_(0)
{
  pthread_mutex_init(&spline_cmd_lock_, NULL);
}

CartesianSplineTrajectoryController::~CartesianSplineTrajectoryController()
{
}

bool CartesianSplineTrajectoryController::init(mechanism::RobotState *robot_state, const ros::NodeHandle &n)
{
  ROS_INFO("Trying to initialize the controller");
  node_ = n;
  robot_state_ = robot_state;

  // get a pointer to the cartesian controller
  string output;
  if (!node_.getParam("output", output)){
    ROS_ERROR("No output name found on parameter server (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  ros::Duration timeout(20);
  ros::Time start_time = ros::Time::now();
  bool found_controller = false;
  do
  {
    found_controller = getController<ChildController>(output, AFTER_ME, cart_controller_);

    if (!found_controller)
    {
      ros::Duration(0.5).sleep();
    }
  }
  while (!found_controller && ((start_time + timeout) > ros::Time::now()));

  if (!found_controller)
  {
    ROS_ERROR("Could not connect to cartesian controller \"%s\"", output.c_str());
    return false;
  }

  num_joints_ = 7;
  joint_names_.resize(7);
  joint_names_[0] = "x";
  joint_names_[1] = "y";
  joint_names_[2] = "z";
  joint_names_[3] = "qx";
  joint_names_[4] = "qy";
  joint_names_[5] = "qz";
  joint_names_[6] = "qw";

  goal_.positions.resize(joint_names_.size());
  goal_.velocities.resize(joint_names_.size());
  goal_.accelerations.resize(joint_names_.size());

  cmd_.positions.resize(num_joints_);
  cmd_.velocities.resize(num_joints_);
  cmd_.accelerations.resize(num_joints_);

  current_position_cmd_.set_segments_size(1);
  current_position_cmd_.segments[0].set_a_size(num_joints_);
  current_position_cmd_.segments[0].set_b_size(num_joints_);
  current_position_cmd_.segments[0].set_c_size(num_joints_);
  current_position_cmd_.segments[0].set_d_size(num_joints_);
  current_position_cmd_.segments[0].set_e_size(num_joints_);
  current_position_cmd_.segments[0].set_f_size(num_joints_);

  node_.param("goal_reached_position_threshold", goal_reached_position_threshold_, 0.005);
  node_.param("goal_reached_angle_threshold", goal_reached_angle_threshold_, 0.005);

  trajectory_set_service_ = node_.advertiseService("SetSplineTrajectory", &CartesianSplineTrajectoryController::setSplineTraj, this);
  trajectory_query_service_ = node_.advertiseService("QuerySplineTrajectory", &CartesianSplineTrajectoryController::querySplineTraj, this);
  trajectory_cancel_service_ = node_.advertiseService("CancelSplineTrajectory", &CartesianSplineTrajectoryController::cancelSplineTraj, this);

  ROS_INFO("Initialized cartesian spline trajectory controller.");
  return true;
}

bool CartesianSplineTrajectoryController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  return init(robot, ros::NodeHandle(config->Attribute("name")));
}


bool CartesianSplineTrajectoryController::starting()
{
  trajectory_id_ = 0;
  spline_rt_ = setCmdToCurrent();
  spline_time_ = ros::Duration(0);
  spline_index_= 0;
  spline_num_segments_ = 1;
  new_cmd_available_ = false;
  trajectory_status_ = manipulation_srvs::QuerySplineTraj::Response::State_Done;
  spline_done_ = false;
  last_time_ = robot_state_->getTime();
  goal_ = getCommand(spline_rt_.segments.back(), spline_rt_.segments.back().duration.toSec());
      spline_time_ = spline_rt_.segments.back().duration;
  setGoalPose(goal_);

  ROS_INFO("Started trajectory controller");
  return true;
}

void CartesianSplineTrajectoryController::update()
{
  spline_time_ += (robot_state_->getTime() - last_time_);

  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&spline_cmd_lock_))
    {
      spline_rt_ = spline_cmd_;
      spline_time_ = ros::Duration(0.0);
      spline_index_ = 0;
      spline_num_segments_ = spline_rt_.segments.size();
      spline_done_ = false;
      trajectory_status_ = manipulation_srvs::QuerySplineTraj::Response::State_Active;
      trajectory_id_++;
      pthread_mutex_unlock(&spline_cmd_lock_);
      new_cmd_available_ = false;
      goal_ = getCommand(spline_rt_.segments.back(), spline_rt_.segments.back().duration.toSec());
      setGoalPose(goal_);
      resetControllers();
    }
  }

  if(spline_time_ > (spline_rt_.segments[spline_index_].duration))
  {
    spline_time_ -= (spline_rt_.segments[spline_index_].duration);
    if(spline_index_ < spline_num_segments_-1)
    {
      spline_index_++;
    }
    else
    {
      spline_done_ = true;
      if(goalReached())
      {
        trajectory_status_ = manipulation_srvs::QuerySplineTraj::Response::State_Done;
      }
      spline_index_ = spline_num_segments_-1;
      spline_time_ = spline_rt_.segments.back().duration;
    }
  }
  manipulation_msgs::Waypoint joint_cmd = getCommand(spline_rt_.segments[spline_index_],spline_time_.toSec());

  setCommand(joint_cmd);

  last_time_ = robot_state_->getTime();
}

manipulation_msgs::Waypoint CartesianSplineTrajectoryController::getCommand(const manipulation_msgs::SplineTrajSegment &spline, const double t)
{
  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t3*t;
  double t5 = t4*t;

  for(int i=0; i < (int) num_joints_; i++)
  {
    cmd_.positions[i] = spline.a[i] + spline.b[i]*t + spline.c[i]*t2 + spline.d[i]*t3 + spline.e[i]*t4 + spline.f[i]*t5;
    cmd_.velocities[i]= spline.b[i] + 2*spline.c[i]*t + 3*spline.d[i]*t2 + 4*spline.e[i]*t3 + 5*spline.f[i]*t4;
    cmd_.accelerations[i]= 2*spline.c[i] + 6*spline.d[i]*t + 12*spline.e[i]*t2 + 20*spline.f[i]*t3;
  }
  return cmd_;
}

void CartesianSplineTrajectoryController::setCommand(const manipulation_msgs::Waypoint &wp)
{
  waypointToPoseTwist(wp, cart_controller_->pose_desi_, cart_controller_->twist_desi_);
}

void CartesianSplineTrajectoryController::resetControllers()
{
  /*for(int i=0; i < (int) num_joints_; i++)
  {
    joint_controllers_[i]->reset();
  }*/
}

manipulation_msgs::SplineTraj CartesianSplineTrajectoryController::setCmdToCurrent(void)
{
  for(int i=0; i < (int) num_joints_; i++)
  {
    //current_position_cmd_.segments[0].a[i] = joint_controllers_[i]->joint_state_->position_;
    current_position_cmd_.segments[0].b[i] = 0.0;
    current_position_cmd_.segments[0].c[i] = 0.0;
    current_position_cmd_.segments[0].d[i] = 0.0;
    current_position_cmd_.segments[0].e[i] = 0.0;
    current_position_cmd_.segments[0].f[i] = 0.0;
  }
  current_position_cmd_.segments[0].a[0] = cart_controller_->pose_desi_.p(0);
  current_position_cmd_.segments[0].a[1] = cart_controller_->pose_desi_.p(1);
  current_position_cmd_.segments[0].a[2] = cart_controller_->pose_desi_.p(2);
  double x, y, z, w;
  cart_controller_->pose_desi_.M.GetQuaternion(x, y, z, w);
  current_position_cmd_.segments[0].a[3] = x;
  current_position_cmd_.segments[0].a[4] = y;
  current_position_cmd_.segments[0].a[5] = z;
  current_position_cmd_.segments[0].a[6] = w;
  return current_position_cmd_;
}

bool CartesianSplineTrajectoryController::setSplineTraj(manipulation_srvs::SetSplineTraj::Request &req,
                                         manipulation_srvs::SetSplineTraj::Response &resp)
{
  pthread_mutex_lock(&spline_cmd_lock_);
  if(req.spline.segments.size() > 0)
  {
    spline_cmd_ = req.spline;
  }
  else
  {
    spline_cmd_ = setCmdToCurrent();
  }
  pthread_mutex_unlock(&spline_cmd_lock_);
  new_cmd_available_ = true;
  ros::Duration new_cmd_wait(0.002);
  while(new_cmd_available_)
  {
    new_cmd_wait.sleep();
  }
  resp.trajectory_id = trajectory_id_;
  return true;
}

bool CartesianSplineTrajectoryController::querySplineTraj(manipulation_srvs::QuerySplineTraj::Request &req,
                                         manipulation_srvs::QuerySplineTraj::Response &resp)
{
  if(req.trajectory_id == trajectory_id_)
  {
    resp.trajectory_status = trajectory_status_;
  }
  else if (req.trajectory_id == manipulation_srvs::QuerySplineTraj::Request::Query_Joint_Names)
  {
    resp.joint_names.resize(num_joints_);
    resp.joint_positions.resize(num_joints_);
    manipulation_msgs::Waypoint wp;
    poseToWaypoint(cart_controller_->pose_meas_, wp);
    for(int i=0; i< (int) num_joints_;i++)
    {
      resp.joint_names[i] = joint_names_[i];
      resp.joint_positions[i] = wp.positions[i];
    }
  }
  else if (req.trajectory_id > trajectory_id_)
  {
    resp.trajectory_status = manipulation_srvs::QuerySplineTraj::Response::State_Does_Not_Exist;
  }
  else
  {
    resp.trajectory_status = manipulation_srvs::QuerySplineTraj::Response::State_Done;
  }
  return true;
}

bool CartesianSplineTrajectoryController::cancelSplineTraj(manipulation_srvs::CancelSplineTraj::Request &req,
                                            manipulation_srvs::CancelSplineTraj::Response &resp)
{
  ROS_INFO("Received cancel request with id: %d",req.trajectory_id);
  ROS_INFO("Current trajectory id: %d",trajectory_id_);
  if(req.trajectory_id == trajectory_id_)
  {
    ROS_INFO("Cancel id: %d",req.trajectory_id);
    pthread_mutex_lock(&spline_cmd_lock_);
    spline_cmd_ = setCmdToCurrent();
    pthread_mutex_unlock(&spline_cmd_lock_);
    new_cmd_available_ = true;
  }
  return true;
}

bool CartesianSplineTrajectoryController::goalReached()
{
  bool return_val = true;
  double error(0.0);
  KDL::Twist tw = diff(goal_pose_,cart_controller_->pose_meas_);
  for(int i=0; i < 6; i++)
  {
    double thresh = goal_reached_position_threshold_;
    if (i>=3)
      thresh = goal_reached_angle_threshold_;
    error = fabs(tw(i));
    return_val = return_val && (error <= thresh);
  }
  return return_val;
}

void CartesianSplineTrajectoryController::setGoalPose(manipulation_msgs::Waypoint& wp)
{
  goal_pose_.p(0) = wp.positions[0];
  goal_pose_.p(1) = wp.positions[1];
  goal_pose_.p(2) = wp.positions[2];

  goal_pose_.M = KDL::Rotation::Quaternion(
      wp.positions[3],
      wp.positions[4],
      wp.positions[5],
      wp.positions[6]);
}

void CartesianSplineTrajectoryController::poseToWaypoint(const KDL::Frame& frame, manipulation_msgs::Waypoint& wp) const
{
  wp.positions.resize(7);
  wp.positions[0] = frame.p(0);
  wp.positions[1] = frame.p(1);
  wp.positions[2] = frame.p(2);
  KDL::Rotation tmp = frame.M;
  tmp.GetQuaternion(
      wp.positions[3],
      wp.positions[4],
      wp.positions[5],
      wp.positions[6]
      );
}

void CartesianSplineTrajectoryController::waypointToPoseTwist(const manipulation_msgs::Waypoint& wp, KDL::Frame& pose, KDL::Twist& twist) const
{
  // normalize the quaternion:
  double quat[4];
  double quat_sum=0.0;
  for (int i=0; i<4; i++)
  {
    quat[i] = wp.positions[i+3];
    quat_sum += quat[i] * quat[i];
  }
  quat_sum = sqrt(quat_sum);
  for (int i=0; i<4; i++)
    quat[i] /= quat_sum;

  // set the desired pose and twist:
  pose.p(0) = wp.positions[0];
  pose.p(1) = wp.positions[1];
  pose.p(2) = wp.positions[2];
  pose.M = KDL::Rotation::Quaternion(quat[0], quat[1], quat[2], quat[3]);

  twist(0) = wp.velocities[0];
  twist(1) = wp.velocities[1];
  twist(2) = wp.velocities[2];
  for (int i=3; i<6; i++)
    twist(i) = 0.0;
}
