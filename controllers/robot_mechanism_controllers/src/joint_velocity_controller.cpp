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

#include <robot_mechanism_controllers/joint_velocity_controller.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(JointVelocityController)

JointVelocityController::JointVelocityController()
: joint_state_(NULL), robot_(NULL), last_time_(0), command_(0)
{
  controller_state_publisher_ = NULL;
}

JointVelocityController::~JointVelocityController()
{
}

bool JointVelocityController::init(mechanism::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->hw_->current_time_;

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    fprintf(stderr, "JointVelocityController could not find joint named \"%s\"\n",
            joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;
}

bool JointVelocityController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointVelocityController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  TiXmlElement *p = j->FirstChildElement("pid");
  control_toolbox::Pid pid;
  if (p)
  {
    controller_state_publisher_->msg_.p = p->Attribute("p") ? atof(p->Attribute("p")) : 0.0;
    controller_state_publisher_->msg_.i = p->Attribute("i") ? atof(p->Attribute("i")) : 0.0;
    controller_state_publisher_->msg_.d = p->Attribute("d") ? atof(p->Attribute("d")) : 0.0;
    controller_state_publisher_->msg_.i_clamp = p->Attribute("iClamp") ? atof(p->Attribute("iClamp")) : 0.0;
    pid.initXml(p);
  }
  else
    fprintf(stderr, "JointVelocityController's config did not specify the default pid parameters.\n");

  return init(robot, joint_name, pid);
}

void JointVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
  controller_state_publisher_->msg_.p = p; 
  controller_state_publisher_->msg_.i = i; 
  controller_state_publisher_->msg_.d = d; 
  controller_state_publisher_->msg_.i_clamp = i_max; 
}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointVelocityController::getJointName()
{
  return joint_state_->joint_->name_;
}

// Set the joint velocity command
void JointVelocityController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void JointVelocityController::getCommand(double  & cmd)
{
  cmd = command_;
}

void JointVelocityController::update()
{
  assert(robot_ != NULL);
  double error(0);
  double time = robot_->hw_->current_time_;
  static int count = 0;

  error =joint_state_->velocity_ - command_;

  joint_state_->commanded_effort_ = pid_controller_.updatePid(error, time - last_time_);
  
  if(count % 10 == 0)
  {
    if(controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.set_point = command_; 
      controller_state_publisher_->msg_.process_value = joint_state_->velocity_; 
      controller_state_publisher_->msg_.error = error; 
      controller_state_publisher_->msg_.time_step = time - last_time_; 
      controller_state_publisher_->unlockAndPublish();      
    }
  }
  count++;
  last_time_ = time;
}

//------ Joint Velocity controller node --------
ROS_REGISTER_CONTROLLER(JointVelocityControllerNode)

JointVelocityControllerNode::JointVelocityControllerNode(): node_(ros::Node::instance())
{
  c_ = new JointVelocityController();
}

JointVelocityControllerNode::~JointVelocityControllerNode()
{
  c_->controller_state_publisher_->stop();
  delete c_->controller_state_publisher_;
  delete c_;
}

void JointVelocityControllerNode::update()
{
  c_->update();
}

bool JointVelocityControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(node_);
  service_prefix_ = config->Attribute("name");
  //publishers
  if (c_->controller_state_publisher_ != NULL)
    delete c_->controller_state_publisher_ ;
  c_->controller_state_publisher_ = new realtime_tools::RealtimePublisher <robot_mechanism_controllers::JointControllerState> (service_prefix_+"/state", 1) ;
  
  
  // Parses subcontroller configuration
  if (!c_->initXml(robot, config))
    return false;
  //subscriptions
  node_->subscribe(service_prefix_ + "/set_command", cmd_, &JointVelocityControllerNode::setCommand, this, 1);
  guard_set_command_.set(service_prefix_ + "/set_command");
  //services
  node_->advertiseService(service_prefix_ + "/get_command", &JointVelocityControllerNode::getCommand, this);
  guard_get_command_.set(service_prefix_ + "/get_command");

  return true;
}
void JointVelocityControllerNode::setCommand()
{
  c_->setCommand(cmd_.data);
}

bool JointVelocityControllerNode::getCommand(robot_srvs::GetValue::Request &req,
                                             robot_srvs::GetValue::Response &resp)
{
  double cmd;
  c_->getCommand(cmd);
  resp.v = cmd;
  return true;
}
