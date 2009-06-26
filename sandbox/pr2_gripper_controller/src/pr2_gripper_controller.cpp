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
 
#include <pr2_gripper_controller/pr2_gripper_controller.h>

using namespace controller;
ROS_REGISTER_CONTROLLER(Pr2GripperController)

bool Pr2GripperController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  grasp_cmd_.cmd = "move";
  grasp_cmd_.val = 0.0;
  new_cmd_available_ = false;

  robot_state_ = robot_state;
  name_ = config->Attribute("name"); //"l_gripper" or "r_gripper" expected
  mechanism::Link *link = robot_state_->model_->getLink(name_ + "_link");
  joint_ = robot_state->getJointState(link->joint_name_);
  joint_controller_.init(robot_state_, name_ + "_joint");
  ros::Node::instance()->param<double>(name_ + "/default_speed",default_speed_,20);
  ros::Node::instance()->param<double>(name_ + "/timeout", timeout_, 0.0);
  ros::Node::instance()->subscribe(name_+"_cmd", grasp_cmd, &Pr2GripperController::command_callback, this,1);
  return true;
}

void Pr2GripperController::update()
{
  //do nothing if the joint is not calibrated
  if (joint_->calibrated_)
  {
    return;  // motor's not calibrated
  }
  
  double current_time = robot_state_->hw_->current_time_;
  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&pr2_gripper_controller_lock_) == 0) //the callback is not writing to grasp_cmd_
    {
      grasp_cmd_desired_.cmd = grasp_cmd_.cmd;
      grasp_cmd_desired_.start = grasp_cmd_.start;
      grasp_cmd_desired_.end = grasp_cmd_.end;
      grasp_cmd_desired_.time = grasp_cmd_.time;
      grasp_cmd_desired_.val = grasp_cmd_.val;
      
      pthread_mutex_unlock(&pr2_gripper_controller_lock_);
    }
  }
  
  //check for timeout
  if((current_time - cmd_received_timestamp_) <= timeout_ || timeout_ == 0.0) //continue with what you were doing
  {
    joint_controller_.command_ = parseMessage(grasp_cmd_desired_); //set value
    joint_controller_.update(); //update value
  }
  else //if timed out, don't do anything
  {      
    //stop motor
    //set value
    joint_controller_.command_ = 0.0;
    //update value
    joint_controller_.update();
  }
  last_time_ = current_time;
}

bool Pr2GripperController::starting()
{
	last_time_ = robot_state_->hw_->current_time_;
	cmd_received_timestamp_ = robot_state_->hw_->current_time_;
	return true;
}

double Pr2GripperController::rampMove(double start_force, double end_force, double time)
{
  double del_force = end_force - start_force;
  double del_time = robot_state_->hw_->current_time_ - cmd_received_timestamp_;
  if(del_time > time)
  	return 0.0;
  return start_force + del_time/time*del_force;
}

double Pr2GripperController::stepMove(double step_size)
{
  return joint_controller_.command_ + step_size;
}

double Pr2GripperController::grasp()
{
  //TODO::make a closed loop grasp
  return 0.0;
  
  //TODO::get position and velocity from the gripper motor
  //use joint_->position and joint_->velocity
  
  //TODO::get info from tactile sensors
  
  
}

double Pr2GripperController::parseMessage(pr2_gripper_controller::GripperControllerCmd desired_msg)
{
  //TODO::parse the different messages into a double val
  if(desired_msg.cmd.compare("grasp"))
  {
    return grasp();
  }
  else if(desired_msg.cmd.compare("move"))
  {
    return desired_msg.val;
  }
  else if(desired_msg.cmd.compare("step"))
  {
    return stepMove(desired_msg.val);
  }
  else if(desired_msg.cmd.compare("ramp"))
  {
    return rampMove(desired_msg.start, desired_msg.end, desired_msg.time);
  }
  return 0.0;
}

void Pr2GripperController::command_callback()
{
  pthread_mutex_lock(&pr2_gripper_controller_lock_);
  //TODO::set command
  grasp_cmd_.cmd = grasp_cmd.cmd;
  grasp_cmd_.start = grasp_cmd.start;
  grasp_cmd_.end = grasp_cmd.end;
  grasp_cmd_.time = grasp_cmd.time;
  grasp_cmd_.val = grasp_cmd.val;
  cmd_received_timestamp_ = robot_state_->hw_->current_time_;
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
}




