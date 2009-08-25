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
#include <joint_qualification_controllers/hysteresis_controller.h>

#define MAX_DATA_POINTS 120000

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(HysteresisController)

HysteresisController::HysteresisController()
: joint_(NULL), 
  robot_(NULL),
  data_sent_(false), 
  call_service_(NULL)
{
  test_data_.test_name ="hysteresis";
  test_data_.joint_name = "default joint";
  test_data_.time.resize(MAX_DATA_POINTS);
  test_data_.cmd.resize(MAX_DATA_POINTS);
  test_data_.effort.resize(MAX_DATA_POINTS);
  test_data_.position.resize(MAX_DATA_POINTS);
  test_data_.velocity.resize(MAX_DATA_POINTS);

  test_data_.arg_name.resize(11);
  test_data_.arg_value.resize(11);
  test_data_.arg_name[0] = "Min. Expected Effort";
  test_data_.arg_name[1] = "Max. Expected Effort";
  test_data_.arg_name[2] = "Minimum Position";
  test_data_.arg_name[3] = "Minimum Position";
  test_data_.arg_name[4] = "Velocity";
  test_data_.arg_name[5] = "Timeout";
  test_data_.arg_name[6] = "Max. Allowed Effort";
  test_data_.arg_name[7] = "P Gain";
  test_data_.arg_name[8] = "I Gain";
  test_data_.arg_name[9] = "D Gain";
  test_data_.arg_name[10] = "I-Clamp";

  state_         = STOPPED;
  starting_count = 0;
  velocity_      = 0;
  initial_time_  = 0;
  max_effort_    = 0;
  complete       = false;
  start          = true;
  loop_count_    = 0;
  count_         = 0;
  
  // Assume 1KHz update rate
  timeout_ = MAX_DATA_POINTS / 1000;
}

HysteresisController::~HysteresisController()
{
}

bool HysteresisController::init( mechanism::RobotState *robot, const ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;

  std::string name;
  if (!n.getParam("velocity_controller/joint", name)){
    ROS_ERROR("Hysteresis Controller: No joint name found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(joint_ = robot->getJointState(name)))
  {
    ROS_ERROR("HysteresisController could not find joint named \"%s\"\n", name.c_str());
    return false;
  }

  if (!n.getParam("velocity", velocity_)){
    ROS_ERROR("Hysteresis Controller: No velocity found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("max_effort", max_effort_)){
    ROS_ERROR("Hysteresis Controller: No max effort found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  double min_expected, max_expected, max_pos, min_pos;
  
    if (!n.getParam("min_expected", min_expected)){
    ROS_ERROR("Hysteresis Controller: No min expected effort found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("max_expected", max_expected)){
    ROS_ERROR("Hysteresis Controller: No max expected effort found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("max_position", max_pos)){
    ROS_ERROR("Hysteresis Controller: No max position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("min_position", min_pos)){
    ROS_ERROR("Hysteresis Controller: No min position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("timeout", timeout_)){
    ROS_ERROR("Hysteresis Controller: No timeout found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  initial_time_ = robot_->getTime();
  initial_position_ = joint_->position_;

  // Set values in test data output
  test_data_.joint_name = name;
  test_data_.arg_value[0] = min_expected;
  test_data_.arg_value[1] = max_expected;
  test_data_.arg_value[2] = min_pos;
  test_data_.arg_value[3] = max_pos;
  test_data_.arg_value[4] = velocity_;
  test_data_.arg_value[5] = timeout_;
  test_data_.arg_value[6] = max_effort_;

  
  velocity_controller_ = new JointVelocityController();
  if (!velocity_controller_->init(robot, ros::NodeHandle(n, "velocity_controller"))) return false;

  // Get the gains, add them to test data
  double p, i, d, iClamp, imin;
  velocity_controller_->getGains(p, i, d, iClamp, imin);

  test_data_.arg_value[7] = p;
  test_data_.arg_value[8] = i;
  test_data_.arg_value[9] = d;
  test_data_.arg_value[10] = iClamp;

  call_service_.reset(new realtime_tools::RealtimeSrvCall<joint_qualification_controllers::TestData::Request, joint_qualification_controllers::TestData::Response>(n, "/test_data"));

  return true;
}

bool HysteresisController::starting()
{
  velocity_controller_->starting();
  
  initial_time_ = robot_->getTime();
  initial_position_ = joint_->position_;

  count_ = 0;
  return true;
}

void HysteresisController::update()
{
  // wait until the joint is calibrated if its not a wheel
  if(!joint_->calibrated_ && joint_->joint_->name_.find("wheel_joint") != string::npos)
  {
    return;
  }

  double time = robot_->getTime();
  velocity_controller_->update();
  
  if (state_ == STOPPED || state_ == STARTING || state_ == MOVING)
  {
    if(state_!=DONE && count_ < MAX_DATA_POINTS && loop_count_ > 1)
    {
      double cmd;
      velocity_controller_->getCommand(cmd);
      
      test_data_.time[count_] = time;
      test_data_.cmd[count_] = cmd;
      test_data_.effort[count_] = joint_->applied_effort_;
      test_data_.position[count_] = joint_->position_;
      test_data_.velocity[count_] = joint_->velocity_;
      count_++;
    }
  }

  // Timeout check.
  if (time - initial_time_ > timeout_ && state_ != ANALYZING && state_ != DONE) 
  {
    state_ = ANALYZING;
    test_data_.arg_value[5] = 0;
  }

  switch (state_)
  {
  case STOPPED:
    velocity_controller_->setCommand(velocity_);
    velocity_ *= -1.0;
    ++loop_count_;
    starting_count = 0;
    state_ = STARTING;
    break;
  case STARTING:
    
    ++starting_count;
    if (starting_count > 100)
      state_ = MOVING;
    break;
  case MOVING:
    if (fabs(joint_->velocity_) < 0.001 && fabs(joint_->commanded_effort_) > max_effort_ && joint_->joint_->type_!=mechanism::JOINT_CONTINUOUS)
    {
      velocity_controller_->setCommand(0.0);
      if (loop_count_ < 3)
        state_ = STOPPED;
      else
        state_ = ANALYZING;
    }
    else if(fabs(joint_->position_-initial_position_)>6.28 && joint_->joint_->type_==mechanism::JOINT_CONTINUOUS) 
    {
   
      velocity_controller_->setCommand(0.0);
      initial_position_=joint_->position_;
      if (loop_count_ < 3)
        state_ = STOPPED;
      else
        state_ = ANALYZING;
    }
    break;
  case ANALYZING:
    velocity_controller_->setCommand(0.0);
    analysis();
    state_ = DONE;
    break;
  case DONE:
    velocity_controller_->setCommand(0.0);
    if (!data_sent_)
      data_sent_ = sendData();
    break;
  }

}

void HysteresisController::analysis()
{
  //test done
  if (count_ == 0)
    count_ = 1; // Resize if no points

  test_data_.time.resize(count_);
  test_data_.cmd.resize(count_);
  test_data_.effort.resize(count_);
  test_data_.position.resize(count_);
  test_data_.velocity.resize(count_);
  
  return; 
}

bool HysteresisController::sendData()
{
  if (call_service_->trylock())
  {
    joint_qualification_controllers::TestData::Request *out = &call_service_->srv_req_;
    out->test_name = test_data_.test_name;
    out->joint_name = test_data_.joint_name;
    
    out->time = test_data_.time;
    out->cmd = test_data_.cmd;
    out->effort = test_data_.effort;
    out->position = test_data_.position;
    out->velocity = test_data_.velocity;
    
    out->arg_name = test_data_.arg_name;
    out->arg_value = test_data_.arg_value;
    call_service_->unlockAndCall();
    return true;
  }
  return false;
}



