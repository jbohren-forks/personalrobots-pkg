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

#include "joint_qualification_controllers/sine_sweep_controller.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(SineSweepController, controller::SineSweepController, controller::Controller)

#define MAX_DATA_POINTS 120000

using namespace std;
using namespace control_toolbox;

namespace controller {

SineSweepController::SineSweepController():
joint_state_(NULL), robot_(NULL), sweep_(NULL), data_sent_(false)
{
  test_data_.test_name = "sinesweep";
  test_data_.joint_name = "default joint";
  test_data_.time.resize(MAX_DATA_POINTS);
  test_data_.cmd.resize(MAX_DATA_POINTS);
  test_data_.effort.resize(MAX_DATA_POINTS);
  test_data_.position.resize(MAX_DATA_POINTS);
  test_data_.velocity.resize(MAX_DATA_POINTS);
  test_data_.arg_name.resize(7);
  test_data_.arg_value.resize(7);
  test_data_.arg_name[0] = "First Mode";
  test_data_.arg_name[1] = "Second Mode";
  test_data_.arg_name[2] = "Error Tolerance";
  test_data_.arg_name[3] = "Start Freq";
  test_data_.arg_name[4] = "End Freq";
  test_data_.arg_name[5] = "Amplitude";
  test_data_.arg_name[6] = "Duration";

  duration_     = 0.0;
  initial_time_ = ros::Time(0);
  count_        = 0;
  done_         = 0;
}

SineSweepController::~SineSweepController()
{
  delete sweep_;
}

bool SineSweepController::init(pr2_mechanism::RobotState *robot, const ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;

  std::string name;
  if (!n.getParam("joint_name", name)){
    ROS_ERROR("SineSweepController: No 'joint name' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(joint_state_ = robot->getJointState(name)))
  {
    ROS_ERROR("SineSweepController could not find joint named \"%s\"\n", name.c_str());
    return false;
  }

  test_data_.joint_name = name;

  double first_mode, second_mode, error_tolerance;
  if (!n.getParam("first_mode", first_mode)){
    ROS_ERROR("SineSweepController: No first mode found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("second_mode", second_mode)){
    ROS_ERROR("SineSweepController: No second mode found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("error_tolerance", error_tolerance)){
    ROS_ERROR("SineSweepController: No error tolerance found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }


  double start_freq, end_freq, amplitude;

  if (!n.getParam("start_freq", start_freq)){
    ROS_ERROR("SineSweepController: No start frequency found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("end_freq", end_freq)){
    ROS_ERROR("SineSweepController: No end frequency found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("amplitude", amplitude)){
    ROS_ERROR("SineSweepController: No amplitude found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("duration", duration_)){
    ROS_ERROR("SineSweepController: No duration found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  sweep_ = new SineSweep;
  if (!sweep_->init(start_freq, end_freq, duration_, amplitude))
    return false;

  test_data_.arg_value[0] = first_mode;
  test_data_.arg_value[1] = second_mode;
  test_data_.arg_value[2] = error_tolerance;
  test_data_.arg_value[3] = start_freq;
  test_data_.arg_value[4] = end_freq;
  test_data_.arg_value[5] = amplitude;
  test_data_.arg_value[6] = duration_;

  initial_time_ = robot_->getTime();

  call_service_.reset(new realtime_tools::RealtimeSrvCall<joint_qualification_controllers::TestData::Request, joint_qualification_controllers::TestData::Response>(n, "/test_data"));

  return true;
}

bool SineSweepController::starting()
{
  initial_time_ = robot_->getTime();
  count_ = 0;

  return true;
}

void SineSweepController::update()
{
  ros::Time time = robot_->getTime();
  // wait until the joint is calibrated if it isn't a wheel
  if(!joint_state_->calibrated_ && joint_state_->joint_->name_.find("wheel_joint") != string::npos)
  {
    initial_time_ = time;
    return;
  }

  if((time - initial_time_).toSec() <= duration_)
  {
    joint_state_->commanded_effort_ = sweep_->update(time - initial_time_);

    if (count_ < MAX_DATA_POINTS && !done_)
    {
      test_data_.time[count_] = time.toSec();
      test_data_.cmd[count_] = joint_state_->commanded_effort_;
      test_data_.effort[count_] = joint_state_->applied_effort_;
      test_data_.position[count_] = joint_state_->position_;
      test_data_.velocity[count_] = joint_state_->velocity_;
      count_++;
    }
  }
  else if(!done_)
  {
    joint_state_->commanded_effort_ = 0;
    analysis();
    done_ = true;
  }
  else // Done
  {
    joint_state_->commanded_effort_ = 0;
    if (!data_sent_)
      data_sent_ = sendData();
  }
}

void SineSweepController::analysis()
{
  count_=count_-1;
  //test done
  assert(count_>0);
  test_data_.time.resize(count_);
  test_data_.cmd.resize(count_);
  test_data_.effort.resize(count_);
  test_data_.position.resize(count_);
  test_data_.velocity.resize(count_);

}

bool SineSweepController::sendData()
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

}
