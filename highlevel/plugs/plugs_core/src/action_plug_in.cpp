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
 *   * Neither the name of Willow Garage nor the names of its
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
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING INeco
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <plugs_core/action_plug_in.h>


namespace plugs_core {

const double MIN_STANDOFF = 0.035;
const double SUCCESS_THRESHOLD = 0.025;
enum {MEASURING, MOVING, INSERTING, FORCING, HOLDING};

PlugInAction::PlugInAction(ros::Node& node) :
  robot_actions::Action<std_msgs::Empty, std_msgs::Empty>("plug_in"),
  action_name_("plug_in"),
  node_(node),
  arm_controller_("r_arm_hybrid_controller"),
  detector_(NULL)
{
  node_.setParam("~roi_policy", "LastImageLocation");
  node_.setParam("~display", 0);
  node_.setParam("~square_size", 0.0042);
  node_.setParam("~board_width", 3);
  node_.setParam("~board_height",4);

  node_.param(action_name_ + "/arm_controller", arm_controller_, arm_controller_);

  if(arm_controller_ == "" )
  {
    ROS_ERROR("%s: Aborted, arm controller param was not set.", action_name_.c_str());
    terminate();
    return;
  }

  node_.advertise<robot_msgs::TaskFrameFormalism>(arm_controller_ + "/command", 2);

  //detector_ = new PlugTracker::PlugTracker(node);
  //detector_->deactivate();

  TF_.reset(new tf::TransformListener(node));
  notifier_.reset(new tf::MessageNotifier<robot_msgs::PoseStamped>(
                    TF_.get(), &node,
                    boost::bind(&PlugInAction::plugMeasurementCallback, this, _1),
                    "/plug_detector/plug_pose", "outlet_pose", 100));

  tff_msg_.header.frame_id = "outlet_pose";

  node_.advertise<plugs_core::PlugInState>(action_name_ + "/state", 10);
};

PlugInAction::~PlugInAction()
{
  if(detector_) delete detector_;
};

robot_actions::ResultStatus PlugInAction::execute(const std_msgs::Empty& empty, std_msgs::Empty& feedback)
{
  reset();
  if (detector_)
    detector_->activate();

  return waitForDeactivation(feedback);
}

void PlugInAction::reset()
{
  last_standoff_ = 1.0e10;
  g_state_ = MEASURING;
  g_started_inserting_ = ros::Time::now();
  g_started_forcing_ = ros::Time::now();
  g_stopped_forcing_ = ros::Time::now();
}

void PoseTFToMsg(const tf::Pose &p, robot_msgs::Twist &t)
{
  t.vel.x = p.getOrigin().x();
  t.vel.y = p.getOrigin().y();
  t.vel.z = p.getOrigin().z();
  btMatrix3x3(p.getRotation()).getEulerYPR(t.rot.x, t.rot.y, t.rot.z);
}

void PlugInAction::plugMeasurementCallback(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr &msg)
{
  plugs_core::PlugInState state_msg;

  //ROS_INFO("recieved plug_pose Msg in callback");

  if (!isActive())
    return;

  if (isPreemptRequested()){
    deactivate(robot_actions::PREEMPTED, empty_);
    if (detector_)
      detector_->deactivate();
    return;
  }
  tff_msg_.header.stamp = msg->header.stamp;
  // Both are transforms from the outlet to the estimated plug pose
  robot_msgs::PoseStamped viz_offset_msg;
  usleep(10000);
  try {
    TF_->transformPose("outlet_pose", *(msg.get()), viz_offset_msg);
    TF_->lookupTransform("outlet_pose", arm_controller_ + "/tool_frame", msg->header.stamp, mech_offset_);
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s: Error, transform exception: %s", action_name_.c_str(), ex.what());
    return;
  }

  tf::Pose viz_offset;
  tf::PoseMsgToTF(viz_offset_msg.pose, viz_offset);
  PoseTFToMsg(viz_offset, state_msg.viz_offset);
  double standoff = std::max(MIN_STANDOFF, viz_offset.getOrigin().length()  * 2.5/4.0);

  // Computes the offset for movement
  tf::Pose viz_offset_desi;
  viz_offset_desi.setIdentity();
  viz_offset_desi.getOrigin().setX(-standoff);
  viz_offset_desi.getOrigin().setY(-0.003);
  viz_offset_desi.getOrigin().setZ(-0.004);
  PoseTFToMsg(viz_offset.inverse() * viz_offset_desi, state_msg.viz_error);
  mech_offset_desi_ = viz_offset.inverse() * viz_offset_desi * mech_offset_;

  prev_state_ = g_state_;

  switch (g_state_) {
    case MEASURING:
    {
#if 0
      if (viz_offset.getOrigin().length() > 0.5 ||
          viz_offset.getRotation().getAngle() > (M_PI/6.0))
      {
        double ypr[3];
        btMatrix3x3(viz_offset.getRotation()).getEulerYPR(ypr[2], ypr[1], ypr[0]);
        ROS_ERROR("%s: Error, Crazy vision offset (%lf, (%lf, %lf, %lf))!!!.",
                  action_name_.c_str(), viz_offset.getOrigin().length(), ypr[0], ypr[1], ypr[2]);
        g_state_ = MEASURING;
        break;
      }
#endif

      tf::Transform diff = viz_offset * prev_viz_offset_.inverse();
      if (diff.getOrigin().length() > 0.002 ||
          diff.getRotation().getAngle() > 0.035)
      {
        ROS_WARN("Vision estimate of the plug wasn't stable: %lf, %lf",
                 diff.getOrigin().length(), diff.getRotation().getAngle());
        g_state_ = MEASURING;
        break;
      }

      double error = sqrt(pow(viz_offset.getOrigin().y() - viz_offset_desi.getOrigin().y(), 2) +
                          pow(viz_offset.getOrigin().z() - viz_offset_desi.getOrigin().z(), 2));
      ROS_DEBUG("%s: Error = %0.6lf.", action_name_.c_str(), error);
      if (error < 0.002 && last_standoff_ < MIN_STANDOFF + 0.002)
        g_state_ = INSERTING;
      else
        g_state_ = MOVING;

      break;
    }

    case MOVING: {
      g_state_ = MEASURING;
      break;
    }

    case INSERTING:
    {
      tf::Vector3 offset = viz_offset.getOrigin() - viz_offset_desi.getOrigin();
      ROS_DEBUG("%s: Offset: (% 0.3lf, % 0.3lf, % 0.3lf)", action_name_.c_str(), offset.x(), offset.y(), offset.z());
      if (g_started_inserting_ + ros::Duration(5.0) < ros::Time::now())
      {
        if (offset.x() > SUCCESS_THRESHOLD) // if (in_outlet)
        {
          g_state_ = FORCING;
        }
        else
        {
          g_state_ = MEASURING;
        }
      }
      break;
    }
    case FORCING:
    {
      if (ros::Time::now() > g_started_forcing_ + ros::Duration(1.0))
        g_state_ = HOLDING;
      break;
    }
    case HOLDING:
    {
      break;
    }
  }

  state_msg.state = prev_state_;
  state_msg.next_state = g_state_;

  if (g_state_ != prev_state_)
  {
    switch (g_state_) {
      case MEASURING:
      {
        ROS_DEBUG("MEASURING");
        if (prev_state_ == INSERTING || prev_state_ == FORCING)
        {
          measure();
        }
        break;
      }
      case MOVING:
      {
        ROS_DEBUG("MOVING");
        move();
        break;
      }
      case INSERTING:
      {
        ROS_DEBUG("INSERTING");
        insert();
        break;
      }
      case FORCING:
      {
        ROS_DEBUG("FORCING");
        force();
        break;
      }
      case HOLDING:
      {
        ROS_DEBUG("HOLDING");
        hold();
        deactivate(robot_actions::SUCCESS, empty_);
        if (detector_)
          detector_->deactivate();
        break;
      }
    }
  }


  last_standoff_ = standoff;
  prev_viz_offset_ = viz_offset;
  return;
}

void PlugInAction::measure()
{
  if (!isActive())
    return;

  tff_msg_.mode.vel.x = 3;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = mech_offset_.getOrigin().x() - 0.05;  // backs off 5cm
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  mech_offset_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  node_.publish(arm_controller_ + "/command", tff_msg_);
  g_stopped_forcing_ = ros::Time::now();
  last_standoff_ = 0.05;
  return;
}

void PlugInAction::move()
{
  if (!isActive())
    return;

  tff_msg_.mode.vel.x = 3;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = mech_offset_desi_.getOrigin().x();
  tff_msg_.value.vel.y = mech_offset_desi_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_desi_.getOrigin().z();
  mech_offset_desi_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  ROS_DEBUG("plublishing command to hybrid controller to move");
  node_.publish(arm_controller_ + "/command", tff_msg_);
  return;

}

void PlugInAction::insert()
{
  if (!isActive())
    return;
  g_started_inserting_ = ros::Time::now();

  tff_msg_.mode.vel.x = 2;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = 0.3;
  tff_msg_.value.vel.y = mech_offset_desi_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_desi_.getOrigin().z();
  mech_offset_desi_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  node_.publish(arm_controller_ + "/command", tff_msg_);

  return;
}

void PlugInAction::force()
{
  if (!isActive())
    return;

  g_started_forcing_ = ros::Time::now();

  tff_msg_.mode.vel.x = 1;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 2;
  tff_msg_.mode.rot.y = 2;
  tff_msg_.mode.rot.z = 2;
  tff_msg_.value.vel.x = 50;
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  tff_msg_.value.rot.x = 0.0;
  tff_msg_.value.rot.y = 0.0;
  tff_msg_.value.rot.z = 0.0;
  tff_msg_.mode.vel.y = 2;
  tff_msg_.mode.vel.z = 2;
  tff_msg_.value.vel.y = 0.0;
  tff_msg_.value.vel.z = 0.0;


  tff_msg_.mode.vel.x = 1;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = 30;
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  mech_offset_desi_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);

  node_.publish(arm_controller_ + "/command", tff_msg_);
  return;
}

void PlugInAction::hold()
{
  if (!isActive())
    return;

  tff_msg_.mode.vel.x = 1;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = 4;
  tff_msg_.value.vel.y = mech_offset_.getOrigin().y();
  tff_msg_.value.vel.z = mech_offset_.getOrigin().z();
  mech_offset_.getBasis().getEulerZYX(tff_msg_.value.rot.z, tff_msg_.value.rot.y, tff_msg_.value.rot.x);
  node_.publish(arm_controller_ + "/command", tff_msg_);
  return;
}

}
