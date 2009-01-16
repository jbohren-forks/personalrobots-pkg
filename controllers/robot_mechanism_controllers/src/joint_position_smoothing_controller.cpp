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

#include <robot_mechanism_controllers/joint_position_smoothing_controller.h>
#include <angles/angles.h>

using namespace std;
using namespace controller;


ROS_REGISTER_CONTROLLER(JointPositionSmoothController)

JointPositionSmoothController::JointPositionSmoothController()
: joint_state_(NULL), robot_(NULL), last_time_(0), command_(0),smoothed_error_(0), smoothing_factor_(1)
{
}

JointPositionSmoothController::~JointPositionSmoothController()
{
}

bool JointPositionSmoothController::init(mechanism::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->hw_->current_time_;

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    fprintf(stderr, "JointPositionSmoothController could not find joint named \"%s\"\n",
            joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;
}

bool JointPositionSmoothController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointPositionSmoothController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  TiXmlElement *p = j->FirstChildElement("pid");
  control_toolbox::Pid pid;
  if (p)
    pid.initXml(p);
  else
    fprintf(stderr, "JointPositionSmoothController's config did not specify the default pid parameters.\n");

  TiXmlElement *s = config->FirstChildElement("filter");
  if(s)
  {
    if(s->QueryDoubleAttribute("smoothing_factor", & smoothing_factor_)!=TIXML_SUCCESS)
    {
      std::cerr<<"You specified a filter option but not the smoothing_factor parameter\n";
      return false;	    }
    else
      std::cout<<"Smoothing factor: "<<smoothing_factor_<<std::endl;
   }

  return init(robot, joint_name, pid);
}

void JointPositionSmoothController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void JointPositionSmoothController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointPositionSmoothController::getJointName()
{
  return joint_state_->joint_->name_;
}

// Set the joint position command
void JointPositionSmoothController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current position command
void JointPositionSmoothController::getCommand(double & cmd)
{
  cmd = command_;
}

void JointPositionSmoothController::update()
{
  if (!joint_state_->calibrated_)
    return;

  assert(robot_ != NULL);
  double error(0);
  double time = robot_->hw_->current_time_;

  assert(joint_state_->joint_);

  if(joint_state_->joint_->type_ == mechanism::JOINT_ROTARY)
  {
    angles::shortest_angular_distance_with_limits(command_, joint_state_->position_, joint_state_->joint_->joint_limit_min_, joint_state_->joint_->joint_limit_max_,error);

  }
  else if(joint_state_->joint_->type_ == mechanism::JOINT_CONTINUOUS)
  {
    error = angles::shortest_angular_distance(command_, joint_state_->position_);
  }
  else //prismatic
  {
    error = joint_state_->position_ - command_;
  }
  
  smoothed_error_ = smoothing_factor_*error + (1-smoothing_factor_)*smoothed_error_;
  joint_state_->commanded_effort_ = pid_controller_.updatePid(smoothed_error_, time - last_time_);
  last_time_ = time;
}

//------ Joint Position controller node --------
ROS_REGISTER_CONTROLLER(JointPositionSmoothControllerNode)

JointPositionSmoothControllerNode::JointPositionSmoothControllerNode(): node_(ros::Node::instance())
{
  c_ = new JointPositionSmoothController();
}

JointPositionSmoothControllerNode::~JointPositionSmoothControllerNode()
{
  delete c_;
}

void JointPositionSmoothControllerNode::update()
{
  c_->update();
}

bool JointPositionSmoothControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(node_);
  service_prefix_ = config->Attribute("name");

  // Parses subcontroller configuration
  if (!c_->initXml(robot, config))
    return false;
  //subscriptions
  node_->subscribe(service_prefix_ + "/set_command", cmd_, &JointPositionSmoothControllerNode::setCommand, this, 1);
  guard_set_command_.set(service_prefix_ + "/set_command");
  //services
  node_->advertise_service(service_prefix_ + "/get_command", &JointPositionSmoothControllerNode::getCommand, this);
  guard_get_command_.set(service_prefix_ + "/get_command");

  return true;
}

void JointPositionSmoothControllerNode::setCommand()
{
  c_->setCommand(cmd_.data);
}

bool JointPositionSmoothControllerNode::getCommand(robot_srvs::GetValue::request &req,
                                             robot_srvs::GetValue::response &resp)
{
  double cmd;
  c_->getCommand(cmd);
  resp.v = cmd;
  return true;
}




