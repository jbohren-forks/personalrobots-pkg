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

#include <pr2_mechanism_controllers/pr2_gripper_controller.h>

using namespace controller;
ROS_REGISTER_CONTROLLER(Pr2GripperController)

Pr2GripperController::Pr2GripperController()
{
  state_publisher_ = NULL;
}

bool Pr2GripperController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  pthread_mutex_init(&pr2_gripper_controller_lock_,NULL);
  grasp_cmd_.cmd = "move";
  grasp_cmd_.val = 0.0;
  new_cmd_available_ = false;
  robot_state_ = robot_state;
  name_ = config->Attribute("name"); //"l_gripper" or "r_gripper" expected
//  mechanism::Link *link = robot_state_->model_->getLink(name_ + "_link");
  joint_ = robot_state->getJointState(name_ + "_joint");
  joint_controller_.init(robot_state_, name_ + "_joint");
  ros::Node::instance()->param<double>(name_ + "/default_speed",default_speed_,joint_->joint_->effort_limit_);
  ros::Node::instance()->param<double>(name_ + "/timeout", timeout_, 0.0);
  ros::Node::instance()->subscribe(name_+"_cmd", grasp_cmd, &Pr2GripperController::command_callback, this,1);
  if (state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete state_publisher_;
  state_publisher_ = new realtime_tools::RealtimePublisher <pr2_msgs::GripperControllerState> (name_ + "/state", 1);
  return true;
}

void Pr2GripperController::update()
{
  //do nothing if the joint is not calibrated
  if (!joint_->calibrated_)
  {
    ROS_INFO("gripper not calibrated!");
    return;  // motor's not calibrated
  }

  double current_time = robot_state_->hw_->current_time_;
  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&pr2_gripper_controller_lock_) == 0) //the callback is not writing to grasp_cmd_
    {
      if(grasp_cmd_desired_.cmd.compare("Event")) //If the incoming message is an event, don't copy the command
      {
        //TODO::copy command
        grasp_cmd_desired_.cmd = grasp_cmd_.cmd;
        grasp_cmd_desired_.start = grasp_cmd_.start;
        grasp_cmd_desired_.end = grasp_cmd_.end;
        grasp_cmd_desired_.time = grasp_cmd_.time;
        grasp_cmd_desired_.val = grasp_cmd_.val;
      }
      new_cmd_available_ = false;
      pthread_mutex_unlock(&pr2_gripper_controller_lock_);
      if(grasp_cmd_desired_.cmd.compare("step") == 0)
        joint_controller_.command_ = effortLimit(stepMove(grasp_cmd_desired_.val));
    }
  }

  //check for timeout
  if((current_time - cmd_received_timestamp_) <= timeout_ || timeout_ == 0.0) //continue with what you were doing
  {
    joint_controller_.command_ = effortLimit(parseMessage(grasp_cmd_desired_)); //set value
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

  //Publish state
  if(state_publisher_->trylock())
  {
    state_publisher_->msg_.joint_commanded_effort = joint_->commanded_effort_;
    state_publisher_->msg_.joint_applied_effort = joint_->applied_effort_;
    state_publisher_->msg_.joint_name = joint_->joint_->name_;
    state_publisher_->msg_.joint_velocity = joint_->velocity_;
    state_publisher_->msg_.joint_position = joint_->position_;
    state_publisher_->unlockAndPublish() ;
  }

  last_time_ = current_time;
}

bool Pr2GripperController::starting()
{
  last_time_ = robot_state_->hw_->current_time_;
  cmd_received_timestamp_ = robot_state_->hw_->current_time_;
  return true;
}

bool Pr2GripperController::stopping()
{
  if(state_publisher_)
  {
    state_publisher_->stop();
    delete state_publisher_;
  }
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

double Pr2GripperController::parseMessage(pr2_mechanism_controllers::GripperControllerCmd desired_msg)
{
  //TODO::parse the different messages into a double val
  if(desired_msg.cmd.compare("grasp") == 0)
  {
    return grasp();
  }
  else if(desired_msg.cmd.compare("move") == 0)
  {
    return desired_msg.val;
  }
  else if(desired_msg.cmd.compare("step") == 0)
  {
    return joint_controller_.command_;
  }
  else if(desired_msg.cmd.compare("ramp") == 0)
  {
    return rampMove(desired_msg.start, desired_msg.end, desired_msg.time);
  }
  else if(desired_msg.cmd.compare("open") == 0)
  {
    return default_speed_;
  }
  else if(desired_msg.cmd.compare("close") == 0)
  {
    return -1.0*default_speed_;
  }
  return 0.0;
}

double Pr2GripperController::effortLimit(double desiredEffort)
{
  if(desiredEffort > joint_->joint_->effort_limit_)
    return joint_->joint_->effort_limit_;
  if(desiredEffort < joint_->joint_->effort_limit_*-1.0)
    return joint_->joint_->effort_limit_*-1.0;
  return desiredEffort;
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
  new_cmd_available_ = true;
  pthread_mutex_unlock(&pr2_gripper_controller_lock_);
}




