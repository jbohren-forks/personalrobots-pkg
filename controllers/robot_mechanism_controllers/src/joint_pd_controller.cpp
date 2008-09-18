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

#include <robot_mechanism_controllers/joint_pd_controller.h>
#include <math_utils/angles.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(JointPDController)

JointPDController::JointPDController()
{
  robot_=NULL;
  joint_=NULL;

  // Initialize PID class
  command_ = 0;
  command_dot_ = 0;
  last_time_=0;
  pthread_mutex_init(&joint_pd_controller_lock_,NULL);
}

JointPDController::~JointPDController()
{
}

void JointPDController::init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::RobotState *robot)

{
  robot_ = robot;
  joint_ = robot->getJointState(name);

  pid_controller_.initPid(p_gain, i_gain, d_gain, windup, -windup);
  command_= 0;
  command_dot_ = 0;
  last_time_= time;

}

bool JointPDController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->hw_->current_time_;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointPDController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "JointPDController could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  TiXmlElement *p = j->FirstChildElement("pid");
  if (p)
    pid_controller_.initXml(p);
  else
    fprintf(stderr, "JointPDController's config did not specify the default pid parameters.\n");

  return true;
}

// Set the joint velocity command
void JointPDController::setPDCommand(double command, double command_dot)
{
  pthread_mutex_lock(&joint_pd_controller_lock_);
  command_t_ = command;
  command_dot_t_ = command_dot;
  pthread_mutex_unlock(&joint_pd_controller_lock_);
}

// Return the current velocity command
void JointPDController::getPDCommand(double &command, double &command_dot)
{
  pthread_mutex_lock(&joint_pd_controller_lock_);
  command = command_t_;
  command_dot = command_dot_t_;
  pthread_mutex_lock(&joint_pd_controller_lock_);
}

double JointPDController::getTime()
{
  return robot_->hw_->current_time_;
}

// Return the measured joint velocity
double JointPDController::getMeasuredVelocity()
{
  return joint_->velocity_;
}

// Return the measured joint velocity
double JointPDController::getMeasuredPosition()
{
  return joint_->position_;
}


std::string JointPDController::getJointName()
{
  return(joint_->joint_->name_);
}

void JointPDController::update()
{
  double error(0), error_dot(0);
  double time = robot_->hw_->current_time_;

  if(pthread_mutex_trylock(&joint_pd_controller_lock_) == 0)
  {
    command_ = command_t_;
    command_dot_ = command_dot_t_;
    pthread_mutex_unlock(&joint_pd_controller_lock_);
  }

  error_dot = joint_->velocity_ - command_dot_;
  error = joint_->position_ - command_;
  joint_->commanded_effort_ = pid_controller_.updatePid(error, error_dot, time - last_time_);
  last_time_ = time;
}

void JointPDController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void JointPDController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

ROS_REGISTER_CONTROLLER(JointPDControllerNode)
JointPDControllerNode::JointPDControllerNode()
{
  c_ = new JointPDController();
}

JointPDControllerNode::~JointPDControllerNode()
{
  delete c_;
}

void JointPDControllerNode::update()
{
  c_->update();
}

bool JointPDControllerNode::setPDCommand(
  robot_mechanism_controllers::SetPDCommand::request &req,
  robot_mechanism_controllers::SetPDCommand::response &resp)
{
  c_->setPDCommand(req.command,req.command_dot);
  c_->getPDCommand(resp.command,resp.command_dot);

  return true;
}

bool JointPDControllerNode::getPDActual(
  robot_mechanism_controllers::GetPDActual::request &req,
  robot_mechanism_controllers::GetPDActual::response &resp)
{
  resp.state_dot = c_->getMeasuredVelocity();
  resp.state = c_->getMeasuredPosition();
  resp.time = c_->getTime();

  return true;
}
bool JointPDControllerNode::getPDCommand(
  robot_mechanism_controllers::GetPDCommand::request &req,
  robot_mechanism_controllers::GetPDCommand::response &resp)
{
  double command, command_dot;
  c_->getPDCommand(command, command_dot);
  resp.command_dot = command_dot;
  resp.command     = command;
  resp.time = c_->getTime();

  return true;
}


void JointPDControllerNode::init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::RobotState *robot)
{
  ros::node *node = ros::node::instance();
  string prefix = name;

  c_->init(p_gain, i_gain, d_gain, windup, time,name, robot);
  node->advertise_service(prefix + "/set_command", &JointPDControllerNode::setPDCommand, this);
  node->advertise_service(prefix + "/get_actual", &JointPDControllerNode::getPDActual, this);
}

bool JointPDControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to JointPDControllerNode\n");
    return false;
  }

  if (!c_->initXml(robot, config))
    return false;
  node->advertise_service(topic + "/set_pd_command", &JointPDControllerNode::setPDCommand, this);
  node->advertise_service(topic + "/get_pd_command", &JointPDControllerNode::getPDCommand, this);
  node->advertise_service(topic + "/get_pd_actual", &JointPDControllerNode::getPDActual, this);
  return true;
}
