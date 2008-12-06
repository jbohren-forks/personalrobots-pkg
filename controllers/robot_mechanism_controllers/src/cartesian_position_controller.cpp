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

//ROS_REGISTER_CONTROLLER(CartesianPositionController)

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
  for (unsigned int i = 0; i < effort_.joints_.size(); ++i)
  {
    if (!effort_.joints_[i]->calibrated_)
      return;
  }

  if (reset_) {
    reset_ = false;
    getTipPosition(&command_);
  }

  assert(tip_);
  double time = robot_->hw_->current_time_;

  tf::Vector3 tip_position;
  getTipPosition(&tip_position);

  tf::Vector3 error = command_ - tip_position;
  effort_.command_[0] = -pid_x_.updatePid(error.x(), time - last_time_);
  effort_.command_[1] = -pid_y_.updatePid(error.y(), time - last_time_);
  effort_.command_[2] = -pid_z_.updatePid(error.z(), time - last_time_);

  effort_.update();

  last_time_ = time;
}

void CartesianPositionController::getTipPosition(tf::Vector3 *p)
{
  *p = tip_->abs_position_ + quatRotate(tip_->abs_orientation_, effort_.offset_);
}

std::string CartesianPositionController::rootFrame()
{
  return effort_.links_[0]->link_->name_;
}


ROS_REGISTER_CONTROLLER(CartesianPositionControllerNode)

CartesianPositionControllerNode::CartesianPositionControllerNode()
: robot_(NULL), pos_publisher_(NULL), TF(*ros::node::instance(), false) , loop_count_(0)
{
  assert(ros::node::instance());
  TF.setExtrapolationLimit(ros::Duration(10000000));
}

CartesianPositionControllerNode::~CartesianPositionControllerNode()
{
  if (pos_publisher_)
    delete pos_publisher_;
}

bool CartesianPositionControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_ = robot;
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to CartesianPositionControllerNode\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

  /*
  node->advertise_service(topic + "/set_command",
                          &CartesianPositionControllerNode::setCommand, this);
  guard_set_command_.set(topic + "/set_command");
  node->advertise_service(topic + "/get_actual",
                          &CartesianPositionControllerNode::getActual, this);
  guard_get_actual_.set(topic + "/get_actual");

  node->subscribe(topic + "/command", command_msg_,
                  &CartesianPositionControllerNode::command, this, 0);
  guard_command_.set(topic + "/command");
  */





  node->subscribe(topic + "/set_command", command_msg_,
                  &CartesianPositionControllerNode::setCommand, this, 1);
  guard_set_command_.set(topic + "/set_command");

  //pos_publisher_ = new misc_utils::RealtimePublisher<std_msgs::Vector3>(topic + "/position", 0);
  pos_publisher_ = new misc_utils::RealtimePublisher<std_msgs::PointStamped>(topic + "/position", 1);

  return true;
}

void CartesianPositionControllerNode::update()
{
  if (++loop_count_ % 10 == 0)
  {
    if (pos_publisher_)
    {
      if (pos_publisher_->trylock())
      {
        pos_publisher_->msg_.header.stamp.fromSec(robot_->hw_->current_time_);
        pos_publisher_->msg_.header.frame_id = c_.rootFrame();

        tf::Point p;
        c_.getTipPosition(&p);
        tf::PointTFToMsg(p, pos_publisher_->msg_.point);

        pos_publisher_->unlockAndPublish();
      }
    }
  }

  c_.update();
}

void CartesianPositionControllerNode::setCommand()
{
  using namespace tf;

  // Transforms the command into the root frame of the chain
  Stamped<tf::Point> point, out;
  PointStampedMsgToTF(command_msg_, point);
  try
  {
    TF.transformPoint(c_.rootFrame(), point, out);
    c_.command_ = out;
  }
  catch (tf::ExtrapolationException ex)
  {
    fprintf(stderr, "CartesianPositionController extrapolated too far: %s\n", ex.what());
  }
  catch (tf::ConnectivityException ex)
  {
    fprintf(stderr, "CartesianPositionController cannot act in frame: %s\n", ex.what());
  }

}

bool CartesianPositionControllerNode::getActual(
  robot_srvs::GetVector::request &req,
  robot_srvs::GetVector::response &resp)
{
  tf::Vector3 v;
  c_.getTipPosition(&v);
  tf::Vector3TFToMsg(v, resp.v);
  return true;
}

}
