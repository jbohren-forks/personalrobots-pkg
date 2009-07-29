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

#include "experimental_controllers/trajectory_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(TrajectoryController);

TrajectoryController::TrajectoryController():num_joints_(0),num_controllers_(0)
{
  pthread_mutex_init(&spline_cmd_lock_, NULL);
}

TrajectoryController::~TrajectoryController()
{
}

bool TrajectoryController::init(mechanism::RobotState *robot_state, const ros::NodeHandle &n)
{
  ROS_INFO("Trying to initialize the controller");
  node_ = n;
  robot_state_ = robot_state;
  std::string joint_names_string;
  node_.param<std::string>("joint_names",joint_names_string,"");
  std::stringstream ss(joint_names_string);
  std::string tmp;
  while(ss >> tmp)
  {
    if(!robot_state_->getJointState(tmp))
    {
      ROS_ERROR("Could not find joint %s in the robot state",tmp.c_str());
      return false;
    }
    joint_names_.push_back(tmp);
    num_joints_++;
  }
  joint_controllers_.resize(joint_names_.size());

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

  goal_reached_threshold_.resize(joint_names_.size());

  for(int i=0; i < (int)joint_names_.size(); i++)
  {
    joint_controllers_[i].reset(new controller::PIDPositionVelocityController());

    num_controllers_++;
    if(!joint_controllers_[i]->init(robot_state_,ros::NodeHandle(n.getNamespace()+"/" + joint_names_[i])))
    {
      ROS_ERROR("Could not load joint controller for joint: %s",joint_names_[i].c_str());
      return false;
    }
    else
    {
      double threshold;
      node_.param<double>(joint_names_[i]+"/"+"goal_reached_threshold",threshold,0.1);
      goal_reached_threshold_[i] = threshold;
    }
  }
  trajectory_set_service_ = node_.advertiseService("SetSplineTrajectory", &TrajectoryController::setSplineTraj, this);
  trajectory_query_service_ = node_.advertiseService("QuerySplineTrajectory", &TrajectoryController::querySplineTraj, this);
  trajectory_cancel_service_ = node_.advertiseService("CancelSplineTrajectory", &TrajectoryController::cancelSplineTraj, this);

  ROS_INFO("Initialized trajectory controller with %d joints.",num_joints_);
  return true;
}

bool TrajectoryController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  return init(robot, ros::NodeHandle(config->Attribute("name")));
}


bool TrajectoryController::starting()
{
  for(int i=0; i < num_joints_; i++)
  {
    if(!joint_controllers_[i]->joint_state_->calibrated_)
    {
      return false;
    }
  }
  trajectory_id_ = 0;
  spline_rt_ = setCmdToCurrent();
  spline_time_ = 0;
  spline_index_= 0;
  spline_num_segments_ = 1;
  new_cmd_available_ = false;
  trajectory_status_ = manipulation_srvs::QuerySplineTraj::Response::State_Done;
  spline_done_ = false;
  last_time_ = robot_state_->hw_->current_time_;
  goal_ = getCommand(spline_rt_.segments.back(), spline_rt_.segments.back().duration.toSec());
      spline_time_ = spline_rt_.segments.back().duration.toSec();

  ROS_INFO("Started trajectory controller");
  return true;
}

void TrajectoryController::update()
{
  spline_time_ += (robot_state_->hw_->current_time_ - last_time_);

  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&spline_cmd_lock_))
    {
      spline_rt_ = spline_cmd_;
      spline_time_ = 0.0;
      spline_index_ = 0;
      spline_num_segments_ = spline_rt_.segments.size();
      spline_done_ = false;
      trajectory_status_ = manipulation_srvs::QuerySplineTraj::Response::State_Active;
      trajectory_id_++;
      pthread_mutex_unlock(&spline_cmd_lock_);
      new_cmd_available_ = false;
      goal_ = getCommand(spline_rt_.segments.back(), spline_rt_.segments.back().duration.toSec());
      resetControllers();
    }
  }

  if(spline_time_ > (spline_rt_.segments[spline_index_].duration.toSec()))
  {
    spline_time_ -= (spline_rt_.segments[spline_index_].duration.toSec());
    if(spline_index_ < spline_num_segments_-1)
    {
      spline_index_++;
    }
    else
    {
      if(!spline_done_ && goalReached())
      {
        spline_done_ = true;
        trajectory_status_ = manipulation_srvs::QuerySplineTraj::Response::State_Done;
      }
      spline_index_ = spline_num_segments_-1;
      spline_time_ = spline_rt_.segments.back().duration.toSec();
    }
  }
  manipulation_msgs::Waypoint joint_cmd = getCommand(spline_rt_.segments[spline_index_],spline_time_);

  setCommand(joint_cmd);

  last_time_ = robot_state_->hw_->current_time_;
}

manipulation_msgs::Waypoint TrajectoryController::getCommand(const manipulation_msgs::SplineTrajSegment &spline, const double t)
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

void TrajectoryController::setCommand(const manipulation_msgs::Waypoint &wp)
{
  for(int i=0; i < (int) num_joints_; i++)
  {
    joint_controllers_[i]->setCommand(wp.positions[i],wp.velocities[i]);
    joint_controllers_[i]->update();
  }
}

void TrajectoryController::resetControllers()
{
  for(int i=0; i < (int) num_joints_; i++)
  {
    joint_controllers_[i]->reset();
  }
}

manipulation_msgs::SplineTraj TrajectoryController::setCmdToCurrent(void)
{
  for(int i=0; i < (int) num_joints_; i++)
  {
    current_position_cmd_.segments[0].a[i] = joint_controllers_[i]->joint_state_->position_;
    current_position_cmd_.segments[0].b[i] = 0.0;
    current_position_cmd_.segments[0].c[i] = 0.0;
    current_position_cmd_.segments[0].d[i] = 0.0;
    current_position_cmd_.segments[0].e[i] = 0.0;
    current_position_cmd_.segments[0].f[i] = 0.0;
  }
  return current_position_cmd_;
}

bool TrajectoryController::setSplineTraj(manipulation_srvs::SetSplineTraj::Request &req,
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

bool TrajectoryController::querySplineTraj(manipulation_srvs::QuerySplineTraj::Request &req,
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
    for(int i=0; i< (int) num_joints_;i++)
    {
      resp.joint_names[i] = joint_names_[i];
      resp.joint_positions[i] = joint_controllers_[i]->joint_state_->position_;
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

bool TrajectoryController::cancelSplineTraj(manipulation_srvs::CancelSplineTraj::Request &req,
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


bool TrajectoryController::goalReached()
{
  bool return_val = true;
  double error(0.0);
  for(int i=0;i < num_joints_;++i)
  {
    if(joint_controllers_[i]->joint_state_->joint_->type_ == mechanism::JOINT_CONTINUOUS || joint_controllers_[i]->joint_state_->joint_->type_ == mechanism::JOINT_ROTARY)
    {
      error = fabs(angles::shortest_angular_distance(goal_.positions[i], joint_controllers_[i]->joint_state_->position_));
      ROS_DEBUG("Joint: %d position error: %f",i,error);
    }
    else //prismatic
    {
      error = fabs(joint_controllers_[i]->joint_state_->position_ - goal_.positions[i]);
    }
    return_val = return_val && (error <= goal_reached_threshold_[i]);
  }
  return return_val;
}
