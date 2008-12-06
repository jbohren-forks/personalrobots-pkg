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

#include "robot_mechanism_controllers/cartesian_orientation_controller.h"
#include <algorithm>
#include "tf/transform_datatypes.h"

namespace controller {

//ROS_REGISTER_CONTROLLER(CartesianOrientationController)

CartesianOrientationController::CartesianOrientationController()
: command_(0,0,0), robot_(NULL), last_time_(0), reset_(true)
{
}

CartesianOrientationController::~CartesianOrientationController()
{
}

bool CartesianOrientationController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  if (!torque_.initXml(robot, config))
    return false;

  tip_ = torque_.links_[torque_.links_.size() - 1];

  TiXmlElement *pid_el = config->FirstChildElement("pid");
  if (!pid_el)
  {
    fprintf(stderr, "Error: CartesianOrientationController requires a pid element\n");
    return false;
  }
  if (!pid_x_.initXml(pid_el))
    return false;
  pid_y_ = pid_x_;
  pid_z_ = pid_x_;

  last_time_ = robot_->hw_->current_time_;

  return true;
}

void CartesianOrientationController::update()
{
  for (unsigned int i = 0; i < torque_.joints_.size(); ++i)
  {
    if (!torque_.joints_[i]->calibrated_)
      return;
  }

  if (reset_) {
    reset_ = false;
    getTipOrientation(&command_);
  }

  assert(tip_);
  double time = robot_->hw_->current_time_;

  tf::Quaternion tip_orientation;
  getTipOrientation(&tip_orientation);

  tf::Quaternion q_diff = (command_ - tip_orientation).normalized();
  tf::Vector3 error = q_diff.getAxis() * q_diff.getAngle();
  torque_.command_[0] = -pid_x_.updatePid(error.x(), time - last_time_);
  torque_.command_[1] = -pid_y_.updatePid(error.y(), time - last_time_);
  torque_.command_[2] = -pid_z_.updatePid(error.z(), time - last_time_);

  torque_.update();

  last_time_ = time;
}

void CartesianOrientationController::getTipOrientation(tf::Quaternion *q)
{
  *q = tip_->abs_orientation_;
}

std::string CartesianOrientationController::rootFrame()
{
  return torque_.links_[0]->link_->name_;
}


ROS_REGISTER_CONTROLLER(CartesianOrientationControllerNode)

CartesianOrientationControllerNode::CartesianOrientationControllerNode()
: robot_(NULL), pos_publisher_(NULL), TF(*ros::node::instance(), false) , loop_count_(0)
{
  assert(ros::node::instance());
  TF.setExtrapolationLimit(ros::Duration().fromSec(10.0e-3));
}

CartesianOrientationControllerNode::~CartesianOrientationControllerNode()
{
  if (pos_publisher_)
    delete pos_publisher_;
}

bool CartesianOrientationControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_ = robot;
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to CartesianOrientationControllerNode\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

  /*
  node->advertise_service(topic + "/set_command",
                          &CartesianOrientationControllerNode::setCommand, this);
  guard_set_command_.set(topic + "/set_command");
  node->advertise_service(topic + "/get_actual",
                          &CartesianOrientationControllerNode::getActual, this);
  guard_get_actual_.set(topic + "/get_actual");

  node->subscribe(topic + "/command", command_msg_,
                  &CartesianOrientationControllerNode::command, this, 0);
  guard_command_.set(topic + "/command");
  */





  node->subscribe(topic + "/set_command", command_msg_,
                  &CartesianOrientationControllerNode::setCommand, this, 1);
  guard_set_command_.set(topic + "/set_command");

  pos_publisher_ = new misc_utils::RealtimePublisher<std_msgs::QuaternionStamped>(topic + "/position", 1);

  return true;
}

void CartesianOrientationControllerNode::update()
{
  if (++loop_count_ % 10 == 0)
  {
    if (pos_publisher_)
    {
      if (pos_publisher_->trylock())
      {
        pos_publisher_->msg_.header.stamp.fromSec(robot_->hw_->current_time_);
        pos_publisher_->msg_.header.frame_id = c_.rootFrame();

        tf::Quaternion q;
        c_.getTipOrientation(&q);
        tf::QuaternionTFToMsg(q, pos_publisher_->msg_.quaternion);

        pos_publisher_->unlockAndPublish();
      }
    }
  }

  c_.update();
}

void CartesianOrientationControllerNode::setCommand()
{
  using namespace tf;

  // Transforms the command into the root frame of the chain
  Stamped<Quaternion> quat, out;
  QuaternionStampedMsgToTF(command_msg_, quat);
  try
  {
    TF.transformQuaternion(c_.rootFrame(), quat, out);
    c_.command_ = out;
  }
  catch (tf::ExtrapolationException ex)
  {
    fprintf(stderr, "CartesianOrientationController extrapolated too far: %s\n", ex.what());
  }
  catch (tf::ConnectivityException ex)
  {
    fprintf(stderr, "CartesianOrientationController cannot act in frame: %s\n", ex.what());
  }

}

bool CartesianOrientationControllerNode::getActual(
  robot_srvs::GetQuaternion::request &req,
  robot_srvs::GetQuaternion::response &resp)
{
  tf::Quaternion q;
  c_.getTipOrientation(&q);
  tf::QuaternionTFToMsg(q, resp.q);
  return true;
}

}
