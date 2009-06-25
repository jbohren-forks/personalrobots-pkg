/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
/*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#include <pr2_mechanism_controllers/pr2_base_controller.h>

using namespace controller;
ROS_REGISTER_CONTROLLER(Pr2BaseController)

Pr2BaseController::Pr2BaseController()
{
  //init variables
  cmd_vel_.vel.vx = 0;
  cmd_vel_.vel.vy = 0;
  cmd_vel_.ang_vel.vz = 0;

  desired_vel_.vel.vx = 0;
  desired_vel_.vel.vy = 0;
  desired_vel_.ang_vel.vz = 0;

  cmd_vel_t_.vel.vx = 0;
  cmd_vel_t_.vel.vy = 0;
  cmd_vel_t_.ang_vel.vz = 0;

  new_cmd_available_ = false;

  state_publisher_ = NULL;

  pthread_mutex_init(&pr2_base_controller_lock_, NULL);
}

Pr2BaseController::~Pr2BaseController()
{
  //Destruct things (publishers)
  if(state_publisher_)
  {
    state_publisher_->stop();
    delete state_publisher_;
  }
}

bool Pr2BaseController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ROS_INFO("starting Pr2BaseController::initXml");
  base_kin_.initXml(robot, config);
  if(state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete state_publisher_;
  state_publisher_ = new realtime_tools::RealtimePublisher<pr2_msgs::BaseControllerState>(base_kin_.name_ + "/state", 1);
  state_publisher_->msg_.set_joint_name_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_stall_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_speed_error_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_speed_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_speed_filtered_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_commanded_effort_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_applied_effort_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  ros::Node::instance()->subscribe("cmd_vel", baseVelMsg, &Pr2BaseController::CmdBaseVelReceived, this, 1);
  //Get params from param server
  ros::Node::instance()->param<double> (base_kin_.name_ + "/max_vel_.vel.vx", max_vel_.vel.vx, .5);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/max_vel_.vel.vy", max_vel_.vel.vy, .5);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/max_vel_.ang_vel.vz", max_vel_.ang_vel.vz, 10.0); //0.5
  ros::Node::instance()->param<double> (base_kin_.name_ + "/max_accel_.acc.ax", max_accel_.acc.ax, .2);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/max_accel_.acc.ay", max_accel_.acc.ay, .2);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/max_accel_.ang_acc.az", max_accel_.ang_acc.az, 10.0); //0.2
  ros::Node::instance()->param<double> (base_kin_.name_ + "/caster_speed_threshold", caster_speed_threshold_, 0.2);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/caster_position_error_threshold", caster_position_error_threshold_, 0.05);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/wheel_speed_threshold", wheel_speed_threshold_, 0.2);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/caster_effort_threshold", caster_effort_threshold_, 3.45);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/wheel_effort_threshold", wheel_effort_threshold_, 3.45);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/kp_wheel_steer_", kp_wheel_steer_, 2.0);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/alpha_stall", alpha_stall_, 0.5);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/kp_caster_steer_", kp_caster_steer_, 40.0);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/timeout", timeout_, 1.0);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/eps", eps_, 1e-5);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/cmd_vel_trans_eps", cmd_vel_trans_eps_, 1e-5);
  ros::Node::instance()->param<double> (base_kin_.name_ + "/cmd_vel_rot_eps", cmd_vel_rot_eps_, 1e-5);

  /*double kp_;
   double ki_;
   double kd_;
   double ki_clamp;*/
  //casters
  ROS_INFO("resizing caster_controller_");
  caster_controller_.resize(base_kin_.num_casters_);
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    control_toolbox::Pid p_i_d;
    state_publisher_->msg_.joint_name[i] = base_kin_.caster_[i].name_ + "joint";
    /*ros::Node::instance()->param<double>(base_kin_.caster_[i].name_ + "/p",kp_,3.0);
     ros::Node::instance()->param<double>(base_kin_.caster_[i].name_ + "/i",ki_,0.1);
     ros::Node::instance()->param<double>(base_kin_.caster_[i].name_ + "/d",kd_,0.0);
     ros::Node::instance()->param<double>(base_kin_.caster_[i].name_ + "/i_clamp",ki_clamp,4.0);*/
    //tmp.init(base_kin_.robot_state_, base_kin_.caster_[i].name_ + "joint", control_toolbox::Pid(kp_,ki_,kd_,ki_clamp));
    //caster_controller_.push_back(tmp);//TODO::make this not copy!!!!!!!
    ROS_INFO("making initParam");
    p_i_d.initParam(base_kin_.name_ + "/" + base_kin_.caster_[i].name_ + "/");
    caster_controller_[i] = new JointVelocityController();
    ROS_INFO("initing controller");
    caster_controller_[i]->init(base_kin_.robot_state_, base_kin_.caster_[i].name_ + "joint", p_i_d);
  }
  //wheels
  wheel_controller_.resize(base_kin_.num_wheels_);
  for(int j = 0; j < base_kin_.num_wheels_; j++)
  {
    control_toolbox::Pid p_i_d;
    state_publisher_->msg_.joint_name[j + base_kin_.num_casters_] = base_kin_.wheel_[j].name_ + "joint";
    /*ros::Node::instance()->param<double>(base_kin_.wheel_[j].name_ + "/kp",kp_,2.0);
     ros::Node::instance()->param<double>(base_kin_.wheel_[j].name_ + "/ki",ki_,0.01);
     ros::Node::instance()->param<double>(base_kin_.wheel_[j].name_ + "/kd",kd_,0.0);
     ros::Node::instance()->param<double>(base_kin_.wheel_[j].name_ + "/i_clamp",ki_clamp,0.4);*/
    //tmp.init(base_kin_.robot_state_, base_kin_.wheel_[j].name_ + "joint", control_toolbox::Pid(kp_,ki_,kd_,ki_clamp));
    //wheel_controller_.push_back(tmp);//TODO::make this not copy!!!!!!!
    p_i_d.initParam(base_kin_.name_ + "/" + base_kin_.wheel_[j].name_ + "/");
    wheel_controller_[j] = new JointVelocityController();
    wheel_controller_[j]->init(base_kin_.robot_state_, base_kin_.wheel_[j].name_ + "joint", p_i_d);
  }
  ROS_INFO("ending Pr2BaseController::initXml");
  return true;
}

// Set the base velocity command
void Pr2BaseController::setCommand(robot_msgs::PoseDot cmd_vel)
{
  double vel_mag = sqrt(cmd_vel.vel.vx * cmd_vel.vel.vx + cmd_vel.vel.vy * cmd_vel.vel.vy);
  double clamped_vel_mag = filters::clamp(vel_mag, -(max_vel_.vel.vx + max_vel_.vel.vy) / 2.0, (max_vel_.vel.vx + max_vel_.vel.vy) / 2.0);
  if(vel_mag > eps_)
  {
    cmd_vel_t_.vel.vx = cmd_vel.vel.vx * clamped_vel_mag / vel_mag;
    cmd_vel_t_.vel.vy = cmd_vel.vel.vy * clamped_vel_mag / vel_mag;
  }
  else
  {
    cmd_vel_t_.vel.vx = 0.0;
    cmd_vel_t_.vel.vy = 0.0;
  }
  cmd_vel_t_.ang_vel.vz = filters::clamp(cmd_vel.ang_vel.vz, -max_vel_.ang_vel.vz, max_vel_.ang_vel.vz);
  cmd_received_timestamp_ = base_kin_.robot_state_->hw_->current_time_;

#if 0
  ROS_INFO("BaseController:: command received: %f %f %f",cmd_vel.vel.vx,cmd_vel.vel.vy,cmd_vel.ang_vel.vz);
  ROS_INFO("BaseController:: command current: %f %f %f", cmd_vel_.vel.vx,cmd_vel_.vel.vy,cmd_vel_.ang_vel.vz);
  ROS_INFO("BaseController:: clamped vel: %f", clamped_vel_mag);
  ROS_INFO("BaseController:: vel: %f", vel_mag);

  for(int i=0; i < (int) base_kin_.num_wheels_; i++)
  {
    ROS_INFO("BaseController:: wheel speed cmd:: %d %f",i,(base_kin_.wheel_[i].direction_multiplier_*base_kin_.wheel_[i].wheel_speed_cmd_));
  }
  for(int i=0; i < (int) base_kin_.num_casters_; i++)
  {
    ROS_INFO("BaseController:: caster speed cmd:: %d %f",i,(base_kin_.caster_[i].steer_velocity_desired_));
  }
#endif
  new_cmd_available_ = true;
}

robot_msgs::PoseDot Pr2BaseController::interpolateCommand(robot_msgs::PoseDot start, robot_msgs::PoseDot end, robot_msgs::PoseDDot max_rate, double dT)
{
  robot_msgs::PoseDot result;
  robot_msgs::PoseDDot alpha;
  double delta(0), max_delta(0);

  delta = end.vel.vx - start.vel.vx;
  max_delta = max_rate.acc.ax * dT;
  if(fabs(delta) <= max_delta || max_delta < eps_)
    alpha.acc.ax = 1;
  else
    alpha.acc.ax = max_delta / fabs(delta);

  delta = end.vel.vy - start.vel.vy;
  max_delta = max_rate.acc.ay * dT;
  if(fabs(delta) <= max_delta || max_delta < eps_)
    alpha.acc.ay = 1;
  else
    alpha.acc.ay = max_delta / fabs(delta);

  delta = end.ang_vel.vz - start.ang_vel.vz;
  max_delta = max_rate.ang_acc.az * dT;
  if(fabs(delta) <= max_delta || max_delta < eps_)
    alpha.ang_acc.az = 1;
  else
    alpha.ang_acc.az = max_delta / fabs(delta);

  double alpha_min = alpha.acc.ax;
  if(alpha.acc.ay < alpha_min)
    alpha_min = alpha.acc.ay;
  if(alpha.ang_acc.az < alpha_min)
    alpha_min = alpha.ang_acc.az;

  result.vel.vx = start.vel.vx + alpha_min * (end.vel.vx - start.vel.vx);
  result.vel.vy = start.vel.vy + alpha_min * (end.vel.vy - start.vel.vy);
  result.ang_vel.vz = start.ang_vel.vz + alpha_min * (end.ang_vel.vz - start.ang_vel.vz);
  return result;
}

robot_msgs::PoseDot Pr2BaseController::getCommand()// Return the current velocity command
{
  robot_msgs::PoseDot cmd_vel;
  pthread_mutex_lock(&pr2_base_controller_lock_);
  cmd_vel.vel.vx = cmd_vel_.vel.vx;
  cmd_vel.vel.vy = cmd_vel_.vel.vy;
  cmd_vel.ang_vel.vz = cmd_vel_.ang_vel.vz;
  pthread_mutex_unlock(&pr2_base_controller_lock_);
  return cmd_vel;
}

bool Pr2BaseController::starting()
{
  last_time_ = base_kin_.robot_state_->hw_->current_time_;
  cmd_received_timestamp_ = base_kin_.robot_state_->hw_->current_time_;
  return true;
}

void Pr2BaseController::update()
{
  for(int i = 0; i < base_kin_.num_casters_; ++i)
  {
    if(!base_kin_.caster_[i].joint_->calibrated_)
    {
      return; // Casters are not calibrated
    }
  }

  double current_time = base_kin_.robot_state_->hw_->current_time_;
  double dT = std::min<double>(current_time - last_time_, base_kin_.MAX_DT_);

  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&pr2_base_controller_lock_) == 0)
    {
      desired_vel_.vel.vx = cmd_vel_t_.vel.vx;
      desired_vel_.vel.vy = cmd_vel_t_.vel.vy;
      desired_vel_.ang_vel.vz = cmd_vel_t_.ang_vel.vz;
      new_cmd_available_ = false;
      pthread_mutex_unlock(&pr2_base_controller_lock_);
    }
  }

  if((current_time - cmd_received_timestamp_) > timeout_)
  {
    cmd_vel_.vel.vx = 0;
    cmd_vel_.vel.vy = 0;
    cmd_vel_.ang_vel.vz = 0;
  }
  else
    cmd_vel_ = interpolateCommand(cmd_vel_, desired_vel_, max_accel_, dT);

  computeJointCommands();

  setJointCommands();

  updateJointControllers();

  computeStall();

  last_time_ = current_time;

  if(state_publisher_->trylock())
  {
    state_publisher_->msg_.command_vx = cmd_vel_.vel.vx;
    state_publisher_->msg_.command_vy = cmd_vel_.vel.vy;
    state_publisher_->msg_.command_vw = cmd_vel_.ang_vel.vz;
    for(int i = 0; i < base_kin_.num_casters_; i++)
    {
      state_publisher_->msg_.joint_speed[i] = base_kin_.caster_[i].caster_speed_;
      state_publisher_->msg_.joint_speed_filtered[i] = base_kin_.caster_[i].caster_speed_filtered_;
      state_publisher_->msg_.joint_speed_error[i] = base_kin_.caster_[i].caster_speed_error_;
      state_publisher_->msg_.joint_stall[i] = base_kin_.caster_[i].caster_stuck_;
      state_publisher_->msg_.joint_commanded_effort[i] = base_kin_.caster_[i].joint_->commanded_effort_;
      state_publisher_->msg_.joint_applied_effort[i] = base_kin_.caster_[i].joint_->applied_effort_;
    }
    for(int i = 0; i < base_kin_.num_wheels_; i++)
    {
      state_publisher_->msg_.joint_speed[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_speed_actual_;
      state_publisher_->msg_.joint_speed_filtered[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_speed_filtered_;
      state_publisher_->msg_.joint_speed_error[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_speed_error_;
      state_publisher_->msg_.joint_stall[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_stuck_;
      state_publisher_->msg_.joint_commanded_effort[i + base_kin_.num_casters_] = base_kin_.wheel_[i].joint_->commanded_effort_;
      state_publisher_->msg_.joint_applied_effort[i + base_kin_.num_casters_] = base_kin_.wheel_[i].joint_->applied_effort_;
    }

    state_publisher_->unlockAndPublish();
  }
}

void Pr2BaseController::computeJointCommands()
{
  base_kin_.computeWheelPositions();

  computeDesiredCasterSteer();

  computeDesiredWheelSpeeds();
}

void Pr2BaseController::setJointCommands()
{
  setDesiredCasterSteer();

  setDesiredWheelSpeeds();
}

void Pr2BaseController::computeDesiredCasterSteer()
{
  robot_msgs::PoseDot result;

  double steer_angle_desired(0.0), steer_angle_desired_m_pi(0.0);
  double error_steer(0.0), error_steer_m_pi(0.0);
  double trans_vel = sqrt(cmd_vel_.vel.vx * cmd_vel_.vel.vx + cmd_vel_.vel.vy * cmd_vel_.vel.vy);
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    result = base_kin_.pointVel2D(base_kin_.caster_[i].offset_, cmd_vel_);
    if(trans_vel < cmd_vel_trans_eps_ && fabs(cmd_vel_.ang_vel.vz) < cmd_vel_rot_eps_)
    {
      steer_angle_desired = base_kin_.caster_[i].steer_angle_stored_;
    }
    else
    {
      steer_angle_desired = atan2(result.vel.vy, result.vel.vx);
      base_kin_.caster_[i].steer_angle_stored_ = steer_angle_desired;
    }
    steer_angle_desired_m_pi = angles::normalize_angle(steer_angle_desired + M_PI);
    error_steer = angles::shortest_angular_distance(steer_angle_desired, base_kin_.caster_[i].joint_->position_);
    error_steer_m_pi = angles::shortest_angular_distance(steer_angle_desired_m_pi, base_kin_.caster_[i].joint_->position_);

    if(fabs(error_steer_m_pi) < fabs(error_steer))
    {
      error_steer = error_steer_m_pi;
      steer_angle_desired = steer_angle_desired_m_pi;
    }
    base_kin_.caster_[i].steer_velocity_desired_ = -kp_caster_steer_ * error_steer;
    base_kin_.caster_[i].caster_position_error_ = error_steer;
  }
}

void Pr2BaseController::setDesiredCasterSteer()
{
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    caster_controller_[i]->setCommand(base_kin_.caster_[i].steer_velocity_desired_);
  }
}

void Pr2BaseController::computeDesiredWheelSpeeds()
{
  robot_msgs::PoseDot wheel_point_velocity;
  robot_msgs::PoseDot wheel_point_velocity_projected;
  robot_msgs::PoseDot wheel_caster_steer_component;
  robot_msgs::PoseDot caster_2d_velocity;

  caster_2d_velocity.vel.vx = 0;
  caster_2d_velocity.vel.vy = 0;
  caster_2d_velocity.ang_vel.vz = 0;

  double steer_angle_actual = 0;
  for(int i = 0; i < (int) base_kin_.num_wheels_; i++)
  {
    base_kin_.wheel_[i].updatePosition();
    caster_2d_velocity.ang_vel.vz = kp_wheel_steer_ * base_kin_.wheel_[i].parent_->steer_velocity_desired_;
    steer_angle_actual = base_kin_.wheel_[i].parent_->joint_->position_;
    wheel_point_velocity = base_kin_.pointVel2D(base_kin_.wheel_[i].position_, cmd_vel_);
    wheel_caster_steer_component = base_kin_.pointVel2D(base_kin_.wheel_[i].offset_, caster_2d_velocity);

    double costh = cos(-steer_angle_actual);
    double sinth = sin(-steer_angle_actual);

    wheel_point_velocity_projected.vel.vx = costh * wheel_point_velocity.vel.vx - sinth * wheel_point_velocity.vel.vy;
    wheel_point_velocity_projected.vel.vy = sinth * wheel_point_velocity.vel.vx + costh * wheel_point_velocity.vel.vy;
    base_kin_.wheel_[i].wheel_speed_cmd_ = (wheel_point_velocity_projected.vel.vx + wheel_caster_steer_component.vel.vx) / (base_kin_.wheel_radius_ * base_kin_.wheel_[i].wheel_radius_scaler_);
  }
}

void Pr2BaseController::setDesiredWheelSpeeds()
{
  for(int i = 0; i < (int) base_kin_.num_wheels_; i++)
  {
    wheel_controller_[i]->setCommand(base_kin_.wheel_[i].direction_multiplier_ * base_kin_.wheel_[i].wheel_speed_cmd_);
  }
}

void Pr2BaseController::updateJointControllers()
{
  for(int i = 0; i < base_kin_.num_wheels_; i++)
    wheel_controller_[i]->update();
  for(int i = 0; i < base_kin_.num_casters_; i++)
    caster_controller_[i]->update();
}

void Pr2BaseController::computeStall()
{

  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    base_kin_.caster_[i].caster_speed_error_ = fabs(base_kin_.caster_[i].joint_->velocity_ - base_kin_.caster_[i].steer_velocity_desired_);
    base_kin_.caster_[i].caster_speed_filtered_ = alpha_stall_ * base_kin_.caster_[i].caster_speed_filtered_ + (1 - alpha_stall_) * base_kin_.caster_[i].joint_->velocity_;//low pass filter
    base_kin_.caster_[i].caster_speed_ = base_kin_.caster_[i].joint_->velocity_;

    if(fabs(base_kin_.caster_[i].caster_speed_) < caster_speed_threshold_ && fabs(base_kin_.caster_[i].caster_position_error_) > caster_position_error_threshold_ && fabs(base_kin_.caster_[i].joint_->applied_effort_) > caster_effort_threshold_)
    {
      base_kin_.caster_[i].caster_stuck_ = 1;
    }
    else
    {
      base_kin_.caster_[i].caster_stuck_ = 0;
    }
  }

  for(int j = 0; j < base_kin_.num_wheels_; j++)
  {
    base_kin_.wheel_[j].wheel_speed_error_ = fabs(base_kin_.wheel_[j].joint_->velocity_ - base_kin_.wheel_[j].wheel_speed_cmd_);
    base_kin_.wheel_[j].wheel_speed_filtered_ = alpha_stall_ * base_kin_.wheel_[j].wheel_speed_filtered_ + (1 - alpha_stall_) * base_kin_.wheel_[j].wheel_speed_actual_;
    if(fabs(base_kin_.wheel_[j].wheel_speed_filtered_) < wheel_speed_threshold_ && fabs(base_kin_.wheel_[j].joint_->applied_effort_) > wheel_effort_threshold_)
    {
      base_kin_.wheel_[j].wheel_stuck_ = 1;
    }
    else
    {
      base_kin_.wheel_[j].wheel_stuck_ = 0;
    }
  }
}

void Pr2BaseController::CmdBaseVelReceived()
{
  pthread_mutex_lock(&pr2_base_controller_lock_);
  this->setCommand(baseVelMsg);
  pthread_mutex_unlock(&pr2_base_controller_lock_);
}
