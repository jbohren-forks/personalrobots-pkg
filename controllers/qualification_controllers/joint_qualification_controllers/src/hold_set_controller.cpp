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

/* Author: Kevin Watts */

#include <joint_qualification_controllers/hold_set_controller.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(HoldSetController)

HoldSetController::HoldSetController()
: robot_(NULL), 
  initial_time_(0.0), 
  start_time_(0.0), 
  lift_min_(0.0),
  lift_max_(0.0),
  flex_min_(0.0),
  flex_max_(0.0),
  flex_delta_(0.0)
{
  dither_time_ = 0.0;
  lift_cmd_ = 0.0;
  lift_delta_ = 0.0;
  flex_cmd_ = 0.0;
  timeout_ = 120;
  lift_index_ = 0;
  flex_index_ = 0;

  hold_set_data_.arg_name.resize(10);
  hold_set_data_.arg_value.resize(10);
  hold_set_data_.arg_name[0] = "Settle Time";
  hold_set_data_.arg_name[1] = "Start Time";
  hold_set_data_.arg_name[2] = "Dither Time";
  hold_set_data_.arg_name[3] = "Timeout";
  hold_set_data_.arg_name[4] = "Lift Min";
  hold_set_data_.arg_name[5] = "Lift Max";
  hold_set_data_.arg_name[6] = "Lift Delta";
  hold_set_data_.arg_name[7] = "Flex Min";
  hold_set_data_.arg_name[8] = "Flex Max";
  hold_set_data_.arg_name[9] = "Flex Delta";

  ///\todo Need PID's for lift, flex

  state_ = STARTING;
}

HoldSetController::~HoldSetController()
{
}

bool HoldSetController::init(mechanism::RobotState *robot, const ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;
 
  // Lift joint
  std::string lift_name;
  if (!n.getParam("lift_name", lift_name)){
    ROS_ERROR("CounterbalanceTestController: No lift joint name found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(lift_state_ = robot->getJointState(lift_name)))
  {
    ROS_ERROR("CounterbalanceTestController could not find lift joint named \"%s\"\n", lift_name.c_str());
    return false;
  }
  hold_set_data_.lift_name = lift_name;

  lift_controller_ = new JointPositionController();
  if (!lift_controller_->init(robot, ros::NodeHandle(n, "lift"))) return false;

  // Flex joint
  std::string flex_name;
  if (!n.getParam("flex_name", flex_name)){
    ROS_ERROR("CounterbalanceTestController: No flex joint name found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(flex_state_ = robot->getJointState(flex_name)))
  {
    ROS_ERROR("CounterbalanceTestController could not find flex joint named \"%s\"\n", flex_name.c_str());
    return false;
  }
  hold_set_data_.flex_name = flex_name;

  flex_controller_ = new JointPositionController();
  if (!flex_controller_->init(robot, ros::NodeHandle(n, "flex"))) return false;

  // Initialize dithers
  lift_dither_ = new control_toolbox::Dither(100.0);
  if (!lift_dither_->init(ros::NodeHandle(n, "lift")))
    return false;

  flex_dither_ = new control_toolbox::Dither(200.0);
  if (!flex_dither_->init(ros::NodeHandle(n, "flex")))
    return false;

  // Lift range
  if (!n.getParam("lift/min", lift_min_)){
    ROS_ERROR("CounterbalanceTestController: No min lift position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("lift/max", lift_max_)){
    ROS_ERROR("CounterbalanceTestController: No max lift position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("lift/delta", lift_delta_)){
    ROS_ERROR("CounterbalanceTestController: No lift delta found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  
  // Flex range
  if (!n.getParam("flex/min", flex_max_)){
    ROS_ERROR("CounterbalanceTestController: No min flex position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("flex/max", flex_min_)){
    ROS_ERROR("CounterbalanceTestController: No max flex position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("flex/delta", flex_delta_)){
    ROS_ERROR("CounterbalanceTestController: No flex delta found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  // Setting controller defaults
  if (!n.getParam("settle_time", settle_time_)){
    ROS_ERROR("CounterbalanceTestController: No settle time found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("dither_time", dither_time_)){
    ROS_ERROR("CounterbalanceTestController: No dither time found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("timeout", timeout_)){
    ROS_ERROR("CounterbalanceTestController: No timeout found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  lift_cmd_ = lift_min_;
  flex_cmd_ = flex_min_ - flex_delta_;

  return true;
}

bool HoldSetController::starting()
{
  initial_time_ = robot_->hw_->current_time_;
  return true;
}

void HoldSetController::update()
{
  // wait until the joints are calibrated to start
  if (!flex_state_->calibrated_ || !lift_state_->calibrated_)
    return;
      
  double time = robot_->hw_->current_time_;
  
  if (time - initial_time_ > timeout_ && state_ != DONE) 
  {
    ROS_ERROR("CounterbalanceTestController timed out during test. Timeout: %f.", timeout_);
    state_ = DONE;
  }

  lift_controller_->update();
  flex_controller_->update();
  
  switch (state_)
  {
  case STARTING:
    {
      ROS_INFO("Starting");

      // Set the flex and lift commands
      // If 
      flex_cmd_ += flex_delta_;
      flex_index_++;
      // Move to next lift position, reset flex
      if (flex_cmd_ > flex_max_)
      {
        flex_cmd_ = flex_min_ - flex_delta_;
        lift_cmd_ += lift_delta_;
        lift_index_++;

        // We're done after we finished the lifts
        if (lift_cmd_ > lift_max_)
        {
          state_ = DONE;
          break;
        }
        hold_set_data_.lift_data.resize(lift_index_ + 1);   
        hold_set_data_.lift_data[lift_index_].lift_position = lift_cmd_;
      }
      else
      {
        hold_set_data_.lift_data[lift_index_].flex_data.resize(flex_index_ + 1);
        hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].flex_position = flex_cmd_;
      }
      
      // Set controllers
      lift_controller_->setCommand(lift_cmd_);
      flex_controller_->setCommand(flex_cmd_);

      start_time_ = time;
      state_ = SETTLING;
      break;
    }
  case SETTLING:
    {
      if (time - start_time_ > settle_time_)
      {
        state_ = DITHERING;
        start_time_ = time;
        ROS_INFO("Dithering!");
      }
      
      break;
    }
  case DITHERING:
    {
      // Add dither
      lift_state_->commanded_effort_ += lift_dither_->update();
      flex_state_->commanded_effort_ += flex_dither_->update();
      
      // Lift
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.time.push_back(time - start_time_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.position.push_back(lift_state_->position_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.velocity.push_back(lift_state_->velocity_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.cmd.push_back(lift_state_->commanded_effort_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.effort.push_back(lift_state_->applied_effort_);
      
      // Flex
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.time.push_back(time - start_time_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.position.push_back(flex_state_->position_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.velocity.push_back(flex_state_->velocity_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.cmd.push_back(flex_state_->commanded_effort_);
      hold_set_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.effort.push_back(flex_state_->applied_effort_);
        
      if (time - start_time_ > dither_time_)
      {
        state_ = STARTING;
      }
      break;
    }
  case DONE:
    {
      if (!data_sent_)
        data_sent_ = sendData();
            
      break;
    }
    
  }
}

bool HoldSetController::sendData()
{
  if (call_service_->trylock())
  {
    ROS_INFO("Calling results service!");
    
    // Copy data and send
    joint_qualification_controllers::HoldSetData::Request *out = &call_service_->srv_req_;
    
    out->lift_name = hold_set_data_.lift_name;
    out->flex_name = hold_set_data_.flex_name;
    out->lift_amplitude = hold_set_data_.lift_amplitude;
    out->flex_amplitude = hold_set_data_.flex_amplitude;

    out->arg_name = hold_set_data_.arg_name;
    out->arg_value = hold_set_data_.arg_value;

    out->lift_data = hold_set_data_.lift_data;
    call_service_->unlockAndCall();
    return true;
  }
  return false;
}

