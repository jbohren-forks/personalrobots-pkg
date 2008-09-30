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

#include "robot_mechanism_controllers/cartesian_position_controller.h"
#include <algorithm>
#include "tf/transform_datatypes.h"

namespace controller {

ROS_REGISTER_CONTROLLER(CartesianPositionController)

CartesianPositionController::CartesianPositionController()
: command_(0,0,0), robot_(NULL), last_time_(0), reset_(true)
{
}

CartesianPositionController::~CartesianPositionController()
{
}

bool CartesianPositionController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  if (!effort_.initXml(robot, config))
    return false;

  tip_ = effort_.links_[effort_.links_.size() - 1];

  TiXmlElement *pid_el = config->FirstChildElement("pid");
  if (!pid_el)
  {
    fprintf(stderr, "Error: CartesianPositionController requires a pid element\n");
    return false;
  }
  if (!pid_x_.initXml(pid_el))
    return false;
  pid_y_ = pid_x_;
  pid_z_ = pid_x_;

  last_time_ = robot_->hw_->current_time_;

  return true;
}

void CartesianPositionController::update()
{
  if (reset_) {
    reset_ = false;
    command_ = tip_->abs_position_ + effort_.offset_;
  }

  assert(tip_);
  double time = robot_->hw_->current_time_;

  btVector3 error = command_ - (tip_->abs_position_ + effort_.offset_);
  effort_.command_[0] = -pid_x_.updatePid(error.x(), time - last_time_);
  effort_.command_[1] = -pid_y_.updatePid(error.y(), time - last_time_);
  effort_.command_[2] = -pid_z_.updatePid(error.z(), time - last_time_);

  printf("    %.4lf %.4lf %.4lf\n", effort_.command_[0],effort_.command_[1],effort_.command_[2]);
  effort_.update();

  last_time_ = time;
}

void CartesianPositionController::getTipPosition(btVector3 *p)
{
  *p = tip_->abs_position_ + effort_.offset_;
}


ROS_REGISTER_CONTROLLER(CartesianPositionControllerNode)

CartesianPositionControllerNode::CartesianPositionControllerNode()
{
}

CartesianPositionControllerNode::~CartesianPositionControllerNode()
{
}

bool CartesianPositionControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to CartesianPositionControllerNode\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

  node->advertise_service(topic + "/set_command",
                          &CartesianPositionControllerNode::setCommand, this);
  guard_set_command_.set(topic + "/set_command");
  node->advertise_service(topic + "/get_actual",
                          &CartesianPositionControllerNode::getActual, this);
  guard_get_actual_.set(topic + "/get_actual");
  return true;
}

void CartesianPositionControllerNode::update()
{
  c_.update();
}

bool CartesianPositionControllerNode::setCommand(
  robot_mechanism_controllers::SetVectorCommand::request &req,
  robot_mechanism_controllers::SetVectorCommand::response &resp)
{
  c_.command_ = btVector3(req.x, req.y, req.z);
  return true;
}

bool CartesianPositionControllerNode::getActual(
  robot_mechanism_controllers::GetVector::request &req,
  robot_mechanism_controllers::GetVector::response &resp)
{
  btVector3 v;
  c_.getTipPosition(&v);
  tf::Vector3TFToMsg(v, resp.v);
  return true;
}

}
