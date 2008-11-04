/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include "robot_mechanism_controllers/cartesian_velocity_controller.h"
#include <algorithm>
#include "tf/transform_datatypes.h"

namespace controller {

//ROS_REGISTER_CONTROLLER(CartesianVelocityController)

CartesianVelocityController::CartesianVelocityController()
  : command_(tf::Vector3(0,0,0)), robot_(NULL), last_time_(0)
{
}

CartesianVelocityController::~CartesianVelocityController()
{
}

bool CartesianVelocityController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  if (!effort_.initXml(robot, config))
    return false;

  tip_ = effort_.links_[effort_.links_.size() - 1];

  TiXmlElement *pid_el = config->FirstChildElement("pid");
  if (!pid_el)
  {
    fprintf(stderr, "Error: CartesianVelocityController requires a pid element\n");
    return false;
  }
  if (!pid_x_.initXml(pid_el))
    return false;
  pid_y_ = pid_x_;
  pid_z_ = pid_x_;

  last_time_ = robot_->hw_->current_time_;

  return true;
}

void CartesianVelocityController::update()
{
  for (unsigned int i = 0; i < effort_.joints_.size(); ++i)
  {
    if (!effort_.joints_[i]->calibrated_)
      return;
  }

  tf::Vector3 &command = command_.next();
  assert(tip_);
  double time = robot_->hw_->current_time_;

  tf::Vector3 tv;
  getTipVelocity(&tv);
  tf::Vector3 error = command - tv;
  effort_.command_[0] = -pid_x_.updatePid(error.x(), time - last_time_);
  effort_.command_[1] = -pid_y_.updatePid(error.y(), time - last_time_);
  effort_.command_[2] = -pid_z_.updatePid(error.z(), time - last_time_);

  effort_.update();

  last_time_ = time;
}

void CartesianVelocityController::getTipVelocity(tf::Vector3 *v)
{
  *v = tip_->abs_velocity_ + cross(tip_->abs_rot_velocity_,
                                   quatRotate(tip_->abs_orientation_, effort_.offset_));
}


ROS_REGISTER_CONTROLLER(CartesianVelocityControllerNode)

CartesianVelocityControllerNode::CartesianVelocityControllerNode()
: vel_publisher_(NULL)
{
}

CartesianVelocityControllerNode::~CartesianVelocityControllerNode()
{
  if (vel_publisher_)
    delete vel_publisher_;
}

bool CartesianVelocityControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to CartesianVelocityControllerNode\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

  node->advertise_service(topic + "/set_command",
                          &CartesianVelocityControllerNode::setCommand, this);
  guard_set_command_.set(topic + "/set_command");
  node->advertise_service(topic + "/get_actual",
                          &CartesianVelocityControllerNode::getActual, this);
  guard_get_actual_.set(topic + "/get_actual");

  node->subscribe(topic + "/command", command_msg_,
                  &CartesianVelocityControllerNode::command, this, 1);
  guard_command_.set(topic + "/command");

  vel_publisher_ = new misc_utils::RealtimePublisher<std_msgs::Vector3>(topic + "/velocity", 0);

  return true;
}

void CartesianVelocityControllerNode::update()
{
  if (++loop_count_ % 10 == 0)
  {
    if (vel_publisher_)
    {
      if (vel_publisher_->trylock())
      {
        tf::Vector3 v;
        c_.getTipVelocity(&v);
        tf::Vector3TFToMsg(v, vel_publisher_->msg_);
        vel_publisher_->unlockAndPublish();
      }
    }
  }
  c_.update();
}

bool CartesianVelocityControllerNode::setCommand(
  robot_srvs::SetVector::request &req,
  robot_srvs::SetVector::response &resp)
{
  tf::Vector3 command;
  tf::Vector3MsgToTF(req.v, command);
  c_.command_.set(command);
  return true;
}

bool CartesianVelocityControllerNode::getActual(
  robot_srvs::GetVector::request &req,
  robot_srvs::GetVector::response &resp)
{
  btVector3 v;
  c_.getTipVelocity(&v);
  tf::Vector3TFToMsg(v, resp.v);
  return true;
}

void CartesianVelocityControllerNode::command()
{
  tf::Vector3 command;
  tf::Vector3MsgToTF(command_msg_, command);
  c_.command_.set(command);
}

}
