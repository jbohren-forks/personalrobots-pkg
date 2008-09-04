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
#include <generic_controllers/ramp_effort_controller.h>

using namespace std;

namespace controller {

ROS_REGISTER_CONTROLLER(RampEffortController)

RampEffortController::RampEffortController()
: joint_state_(NULL), robot_(NULL)
{
  input_start_ = 0;
  input_end_ = 0;
  duration_ = 0;
  initial_time_ = 0;
}

RampEffortController::~RampEffortController()
{
}

void RampEffortController::init(double input_start, double input_end, double duration, double time,std::string name,mechanism::RobotState *robot)
{
  robot_ = robot;
  int index = robot->model_->getJointIndex(name);
  joint_state_ = &robot->joint_states_[index];

  input_start_=input_start;
  input_end_=input_end;
  duration_=duration;
  initial_time_=time;
}

bool RampEffortController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "RampEffortController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_state_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_state_)
  {
    fprintf(stderr, "RampEffortController could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  {
    double input_start = atof(cd->Attribute("start"));
    double input_end = atof(cd->Attribute("end"));
    double duration = atof(cd->Attribute("duration"));
    init(input_start, input_end, duration,robot->hw_->current_time_,j->Attribute("name"), robot);
  }
  else
    fprintf(stderr, "RampEffortController's config did not specify the default control parameters.\n");
  return true;
}

// Return the current position command
double RampEffortController::getCommand()
{
  return joint_state_->commanded_effort_;
}

// Return the measured joint position
double RampEffortController::getMeasuredEffort()
{
  return joint_state_->applied_effort_;
}

// Return the measured joint position
double RampEffortController::getVelocity()
{
  return joint_state_->velocity_;
}

double RampEffortController::getTime()
{
  return robot_->hw_->current_time_;
}

void RampEffortController::update()
{
  double time = robot_->hw_->current_time_;

  joint_state_->commanded_effort_ = input_start_+(input_end_-input_start_)*(time-initial_time_)/(duration_);
}

ROS_REGISTER_CONTROLLER(RampEffortControllerNode)
RampEffortControllerNode::RampEffortControllerNode()
{
  c_ = new RampEffortController();
}

RampEffortControllerNode::~RampEffortControllerNode()
{
  delete c_;
}

void RampEffortControllerNode::update()
{
  c_->update();
}



bool RampEffortControllerNode::getActual(
  generic_controllers::GetActual::request &req,
  generic_controllers::GetActual::response &resp)
{
  resp.command = c_->getMeasuredEffort();
  resp.time = c_->getTime();
  return true;
}

bool RampEffortControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();
  string prefix = config->Attribute("name");

  c_->initXml(robot, config);

  node->advertise_service(prefix + "/get_actual", &RampEffortControllerNode::getActual, this);
  return true;
}

}
