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

#include <joint_qualification_controllers/checkout_controller.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(CheckoutController)

CheckoutController::CheckoutController():
  robot_(NULL)
{
  robot_data_.test_time = 0;
  robot_data_.num_joints = 0;
  robot_data_.num_actuators = 0;

  state_          = STARTING;
  joint_count_    = 0;
  actuator_count_ = 0;
  
  timeout_        = 30.0; 
}

CheckoutController::~CheckoutController()
{
}

void CheckoutController::init( double timeout, mechanism::RobotState *robot)
{
  assert(robot);
  robot_ = robot;
  
  joint_count_ = robot_->joint_states_.size();
  robot_data_.num_joints = joint_count_;

  robot_data_.joint_data.resize(joint_count_);

  for (int i = 0; i < joint_count_; i++)
  {
    mechanism::Joint *joint = robot_->joint_states_[i].joint_;

    robot_data_.joint_data[i].index = i;
    robot_data_.joint_data[i].name = joint->name_;
    robot_data_.joint_data[i].is_cal = 0;
    robot_data_.joint_data[i].has_safety = joint->has_safety_limits_;

    // Assign type strings based on type
    switch (joint->type_)
    {
    case (mechanism::JOINT_NONE):
      robot_data_.joint_data[i].type = "None";
      break;
    case (mechanism::JOINT_ROTARY):
      robot_data_.joint_data[i].type = "Rotary";
      break;
    case (mechanism::JOINT_CONTINUOUS):
      robot_data_.joint_data[i].type = "Continuous";
      break;
    case (mechanism::JOINT_PRISMATIC):
      robot_data_.joint_data[i].type = "Prismatic";
      break;
    case (mechanism::JOINT_FIXED):
      robot_data_.joint_data[i].type = "Fixed";
      break;
    case (mechanism::JOINT_PLANAR):
      robot_data_.joint_data[i].type = "Planar";
      break;
    case (mechanism::JOINT_TYPES_MAX):
      robot_data_.joint_data[i].type = "Types max";
      break;
    default:
      robot_data_.joint_data[i].type = "No type given!";
      break;
    }
  }

  // Assign actuators
  actuator_count_ = robot_->hw_->actuators_.size();
  robot_data_.num_actuators = actuator_count_;
  robot_data_.actuator_data.resize(actuator_count_);
  
  for (int i = 0; i < actuator_count_; i++)
  {
    Actuator *actuator = robot_->hw_->actuators_[i];
   
    robot_data_.actuator_data[i].index = i;
    robot_data_.actuator_data[i].name = actuator->name_;
    robot_data_.actuator_data[i].id = actuator->state_.device_id_;
    robot_data_.actuator_data[i].enabled = 0;
  }

  timeout_ = timeout;

  initial_time_ = robot_->hw_->current_time_;
}

bool CheckoutController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  
  TiXmlElement *cd = config->FirstChildElement("controller_defaults");
  if (cd)
  {
    // Pull timeout attribute if it exists
    const char *time_char = cd->Attribute("timeout");
    double timeout = time_char ? atof(cd->Attribute("timeout")) : timeout_;

    init(timeout, robot);
  }
  else
  {
    init(timeout_, robot);
  }
  return true;
}

void CheckoutController::update()
{
  double time = robot_->hw_->current_time_;
  bool cal = false;
  bool enabled = false;

  // Timeout check.
  if (time - initial_time_ > timeout_ && state_ != ANALYZING && state_ != DONE) 
  {
    analysis(0);
    state_ = DONE;
  }
  
  switch (state_)
  {
  case STARTING:
    initial_time_ = robot_->hw_->current_time_;
    state_ = LISTENING;
    break;
  case LISTENING:
    {
    cal = true;
    for (int i = 0; i < joint_count_; i++)
    {
      // Check for caster wheel joints and fingers
      // Wheel joints and fingers don't calibrate, so don't wait for them
      if (cal && (robot_->joint_states_[i].joint_->name_.find("wheel_joint") != string::npos))
      {     
        //cal = true;
        continue;
      }

      if (cal && (robot_->joint_states_[i].joint_->name_.find("finger") != string::npos))
      {
        //cal = true;
        continue;
      }

      // Base joint is a dummy joint used to set up visualization
      if (cal && robot_->joint_states_[i].joint_->name_ == "base_joint")
      {
        //cal = true;
        continue;
      }

      //if (cal && robot_->joint_states_[i].calibrated_)
      //  cal = true;
      //else
      //  cal = false;
      // break;
      
      if (!robot_->joint_states_[i].calibrated_)
      {     
        cal = false;
        break;
      }
    }

    enabled = true;
    for (int i = 0; i < actuator_count_; i++)
    {
      if (robot_->hw_->actuators_[i]->state_.is_enabled_ && enabled)
        enabled = true;
      else
        enabled = false;
        break;
    }

    if (cal && enabled)
      state_ = ANALYZING;

    break;
    }
  case ANALYZING:
    analysis(time - initial_time_);
    state_ = DONE;
    break;
  case DONE:
    break;
  }

}

void CheckoutController::analysis(double time)
{
  robot_data_.test_time = time;

  for (int i = 0; i < joint_count_; i++)
    robot_data_.joint_data[i].is_cal = robot_->joint_states_[i].calibrated_;

  for (int i = 0; i < actuator_count_; i++)
    robot_data_.actuator_data[i].enabled = robot_->hw_->actuators_[i]->state_.is_enabled_;
    
  return; 
}

ROS_REGISTER_CONTROLLER(CheckoutControllerNode)
CheckoutControllerNode::CheckoutControllerNode()
: data_sent_(false), last_publish_time_(0), call_service_("/robot_checkout")
{
  c_ = new CheckoutController();
}

CheckoutControllerNode::~CheckoutControllerNode()
{
  delete c_;
}

void CheckoutControllerNode::update()
{
  c_->update();

  if (c_->done())
  {
    if(!data_sent_)
    {
      if (call_service_.trylock())
      {
        joint_qualification_controllers::RobotData::Request *out = &call_service_.srv_req_;
        out->test_time     = c_->robot_data_.test_time;
        out->num_joints    = c_->robot_data_.num_joints;
        out->num_actuators = c_->robot_data_.num_actuators;
        
        out->joint_data.resize(c_->robot_data_.num_joints);
        out->actuator_data.resize(c_->robot_data_.num_actuators);

        for (int i = 0; i < c_->joint_count_; i++)
        {
          out->joint_data[i].index      = c_->robot_data_.joint_data[i].index;
          out->joint_data[i].name       = c_->robot_data_.joint_data[i].name;
          out->joint_data[i].is_cal     = c_->robot_data_.joint_data[i].is_cal;
          out->joint_data[i].has_safety = c_->robot_data_.joint_data[i].has_safety;
          out->joint_data[i].type       = c_->robot_data_.joint_data[i].type;
        }

        for (int i = 0; i < c_->actuator_count_; i++)
        {
          out->actuator_data[i].index    = c_->robot_data_.actuator_data[i].index;
          out->actuator_data[i].name     = c_->robot_data_.actuator_data[i].name;
          out->actuator_data[i].id       = c_->robot_data_.actuator_data[i].id;
          out->actuator_data[i].enabled  = c_->robot_data_.actuator_data[i].enabled;
        }
          
        call_service_.unlockAndCall();
        data_sent_ = true;
      }
    }
  }
}

bool CheckoutControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
  
  if (!c_->initXml(robot, config))
    return false;
    
  return true;
}

