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
#include <algorithm>
#include <generic_controllers/ramp_input_controller.h>


using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(RampInputController)

RampInputController::RampInputController()
{

}

RampInputController::~RampInputController()
{
}

void RampInputController::init(double input_start, double input_end, double duration, double time,std::string name,mechanism::Robot *robot)
{
  robot_ = robot;
  joint_ = robot->getJoint(name);
  input_start_=input_start;
  input_end_=input_end;
  duration_=duration;
  initial_time_=time;
}



void RampInputController::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  
  TiXmlElement *elt = config->FirstChildElement("joint");
  if (elt) 
  {
    double input_start = atof(elt->FirstChildElement("start")->GetText());
    double input_end = atof(elt->FirstChildElement("end")->GetText());
    double duration = atof(elt->FirstChildElement("duration")->GetText());
    init(input_start, input_end, duration,robot->hw_->current_time_,elt->Attribute("name"), robot);
  }
    
}

// Return the current position command
double RampInputController::getCommand()
{
  return joint_->commanded_effort_;
}

// Return the measured joint position
double RampInputController::getActual()
{
  return joint_->applied_effort_;
}

// Return the measured joint position
double RampInputController::getVelocity()
{
  return joint_->velocity_;
}

double RampInputController::getTime()
{
  return robot_->hw_->current_time_;
}

void RampInputController::update()
{
  double effort_cmd(0);
  double time = robot_->hw_->current_time_;
  
  effort_cmd=input_start_+(input_end_-input_start_)*(time-initial_time_)/(duration_);
  
  setJointEffort(effort_cmd);
}

void RampInputController::setJointEffort(double effort)
{
  joint_->commanded_effort_ = min(max(effort, -joint_->effort_limit_), joint_->effort_limit_);
}

ROS_REGISTER_CONTROLLER(RampInputControllerNode)
RampInputControllerNode::RampInputControllerNode() 
{
  c_ = new RampInputController();
}

RampInputControllerNode::~RampInputControllerNode()
{
  delete c_;
}

void RampInputControllerNode::update()
{
  c_->update();
}



bool RampInputControllerNode::getActual(
  generic_controllers::GetActual::request &req,
  generic_controllers::GetActual::response &resp)
{
  resp.command = c_->getActual();
  resp.time = c_->getTime();
  return true;
}

void RampInputControllerNode::init(double input_start, double input_end, double duration, double time,std::string name,mechanism::Robot *robot)
{
  assert(false); // temporary fix for lack of xml
}

void RampInputControllerNode::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();
  string prefix = config->Attribute("name");
  
  c_->initXml(robot, config);

  node->advertise_service(prefix + "/get_actual", &RampInputControllerNode::getActual, this);
}

