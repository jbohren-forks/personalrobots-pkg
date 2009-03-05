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

#include <dynamic_verification_controllers/freq_resp_controller.h>

#define MAX_DATA_POINTS 80000
#define PI 3.14159265

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(FrequencyResponseController)

FrequencyResponseController::FrequencyResponseController():
  joint_state_(NULL), robot_(NULL)
{
  dynamic_data_.test_name = "frequency_response";
  dynamic_data_.joint_name = "default";
  dynamic_data_.time.resize(MAX_DATA_POINTS);
  dynamic_data_.cmd.resize(MAX_DATA_POINTS);
  dynamic_data_.effort.resize(MAX_DATA_POINTS);
  dynamic_data_.position.resize(MAX_DATA_POINTS);
  dynamic_data_.velocity.resize(MAX_DATA_POINTS);
  starting_count = 0;
  effort_ = 0;
  frequency_ = 0;
  settle_time_ = 0;
  
  initial_time_ = 0;
  initial_position_ = 0;
  complete = false;
  start = true;
  test_duration_ = 5;
  count_ = 0;
}

FrequencyResponseController::~FrequencyResponseController()
{
}

void FrequencyResponseController::init( double effort, double test_duration, double frequency, double settle_time, double time, std::string name ,mechanism::RobotState *robot)
{
  assert(robot);
  robot_       = robot;
  joint_state_ = robot->getJointState(name);
  if(name=="r_gripper_joint" || name=="l_gripper_joint")
  {
    joint_state_ -> calibrated_ = true;
  }

  dynamic_data_.joint_name = name;

  effort_           = effort;
  frequency_        = frequency;
  settle_time_      = settle_time;
  test_duration_    = test_duration;
  initial_time_     = time;
  initial_position_ = joint_state_->position_;
}

bool FrequencyResponseController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  
  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "FrequencyResponseController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_state_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_state_)
  {
    fprintf(stderr, "FrequencyResponseController could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  {
    double effort = atof(cd->Attribute("effort"));
    double duration = atof(cd->Attribute("duration"));
    double settle_time = atof(cd->Attribute("settle_time"));
    double frequency = atof(cd->Attribute("frequency"));

    init(effort, duration, frequency, settle_time, robot->hw_->current_time_, j->Attribute("name"), robot);
  }
  else
  {
    fprintf(stderr, "FrequencyResponseController's config did not specify the default control parameters.\n");
    return false;
  }
  return true;
}

void FrequencyResponseController::update()
{
  // wait until the joint is calibrated if it has limits
  if(!joint_state_->calibrated_ && joint_state_->joint_->type_!=mechanism::JOINT_CONTINUOUS)
  {
    return;
  }

  double time = robot_->hw_->current_time_;
  double dt = time - initial_time_;
  
  double cmd = effort_ * sin(frequency_ * 2 * PI * dt);

  // Returns true if joint has stopped moving at joint limit
  // can use to remove problem points.
  bool stopped = (fabs(joint_state_->velocity_) < 0.01) && joint_state_->joint_->type_!=mechanism::JOINT_CONTINUOUS && count_ > 100;

  // Log data 
  if (dt <= settle_time_)
  {
    joint_state_->commanded_effort_ = cmd;
    // Don't log anything, wait for it to settle
  }
  if (dt <= (test_duration_ + settle_time_) && !stopped)
  {
    joint_state_->commanded_effort_ = cmd;

    if (count_ < MAX_DATA_POINTS && !done_)
    {
      dynamic_data_.time[count_]     = dt;
      dynamic_data_.cmd[count_]      = joint_state_->commanded_effort_;
      dynamic_data_.effort[count_]   = joint_state_->applied_effort_;
      dynamic_data_.position[count_] = joint_state_->position_;
      dynamic_data_.velocity[count_] = joint_state_->velocity_;
      
      count_++;
    }
  }
  else if(!done_)
  {
    joint_state_->commanded_effort_ = 0;
    analysis();
    done_ = 1;
  }
  else
  {
    joint_state_->commanded_effort_ = 0;
  }
}


void FrequencyResponseController::analysis()
{
  diagnostic_message_.set_status_size(1);
  robot_msgs::DiagnosticStatus *status = &diagnostic_message_.status[0];
  status->name = "FrequencyResponse";
  count_=count_-1;
  //test done
  assert(count_>0);
  status->level = 0;
  status->message = "OK: Done.";

  dynamic_data_.time.resize(count_);
  dynamic_data_.cmd.resize(count_);
  dynamic_data_.effort.resize(count_);
  dynamic_data_.position.resize(count_);
  dynamic_data_.velocity.resize(count_);
  
  return; 
}

ROS_REGISTER_CONTROLLER(FrequencyResponseControllerNode)
FrequencyResponseControllerNode::FrequencyResponseControllerNode()
: data_sent_(false), last_publish_time_(0), call_service_("/dynamic_response_data"),pub_diagnostics_("/diagnostics", 1)
{
  c_ = new FrequencyResponseController();
}

FrequencyResponseControllerNode::~FrequencyResponseControllerNode()
{
  delete c_;
}

void FrequencyResponseControllerNode::update()
{
  c_->update();
  if (c_->done())
  {
    if(!data_sent_)
    {
      if (call_service_.trylock())
      {
        robot_srvs::DynamicResponseData::Request *out = &call_service_.srv_req_;
        out->test_name = c_->dynamic_data_.test_name;
        out->time = c_->dynamic_data_.time;
        out->cmd = c_->dynamic_data_.cmd;
        out->effort = c_->dynamic_data_.effort;
        out->position = c_->dynamic_data_.position;
        out->velocity = c_->dynamic_data_.velocity;
        call_service_.unlockAndCall();
        data_sent_ = true;
      }
    }
    if (last_publish_time_ + 0.5 < robot_->hw_->current_time_)
    {
      if (pub_diagnostics_.trylock())
      {
        last_publish_time_ = robot_->hw_->current_time_;
        
        robot_msgs::DiagnosticStatus *out = &pub_diagnostics_.msg_.status[0];
        out->name = c_->diagnostic_message_.status[0].name;
        out->level = c_->diagnostic_message_.status[0].level;
        out->message = c_->diagnostic_message_.status[0].message;
        pub_diagnostics_.unlockAndPublish();
      }  
    }
  }
}

bool FrequencyResponseControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
  
  if (!c_->initXml(robot, config))
    return false;
    
  pub_diagnostics_.msg_.set_status_size(1);
  return true;
}

