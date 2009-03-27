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

#include "boost/scoped_ptr.hpp"
#include "ros/node.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"

#include "robot_msgs/PoseStamped.h"
#include "robot_msgs/CartesianState.h"
#include "robot_msgs/TaskFrameFormalism.h"

const double MIN_STANDOFF = 0.035;
const double SUCCESS_THRESHOLD = 0.025;

enum {MEASURING, MOVING, PUSHING, FORCING, HOLDING};
int g_state = MEASURING;

ros::Time g_started_pushing = ros::Time::now(),
  g_started_forcing = ros::Time::now(),
  g_stopped_forcing = ros::Time::now(),
  g_started_holding = ros::Time::now();

boost::scoped_ptr<tf::MessageNotifier<robot_msgs::PoseStamped> > g_mn;
boost::scoped_ptr<tf::TransformListener> TF;

void printTransform(const tf::Transform &t, const std::string &prefix = "")
{
  printf("%s (% .3lf, % .3lf, % .3lf)  <% .2lf, % .2lf, % .2lf, % .2lf>\n",
         prefix.c_str(),
         t.getOrigin().x(),
         t.getOrigin().y(),
         t.getOrigin().z(),
         t.getRotation().x(),
         t.getRotation().y(),
         t.getRotation().z(),
         t.getRotation().w());
}

void plug_cb(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr &msg)
{
  if (msg->header.stamp < g_stopped_forcing + ros::Duration(5.0)) {
    printf("too soon after forcing\n");
    return;
  }

  // Both are transforms from the outlet to the estimated plug pose
  robot_msgs::PoseStamped viz_offset_msg;
  tf::Stamped<tf::Transform> mech_offset;
  try {
    TF->transformPose("outlet_pose", *(msg.get()), viz_offset_msg);
    TF->lookupTransform("outlet_pose", "arm_hybrid/tool_frame", msg->header.stamp, mech_offset);
  }
  catch(tf::TransformException &ex)
  {
    //printf("FAIL: %s\n", ex.what());
    printf("FAIL: transform exception\n");
    return;
  }

  tf::Pose viz_offset;
  tf::PoseMsgToTF(viz_offset_msg.pose, viz_offset);

  static double last_standoff = 1.0e10;
  double standoff = std::max(MIN_STANDOFF, viz_offset.getOrigin().length()  * 2.5/4.0);

  // Computes the offset for movement
  tf::Pose viz_offset_desi, mech_offset_desi;
  viz_offset_desi.setIdentity();
  viz_offset_desi.getOrigin().setX(-standoff);
  viz_offset_desi.getOrigin().setZ(-0.004);
  mech_offset_desi = viz_offset.inverse() * viz_offset_desi * mech_offset;

  // Decides when to transition

  int prev_state = g_state;
  switch (g_state) {
  case MEASURING: {
    printf(">\n");
    printf("standoff = %.3lf\n", standoff);
    printTransform(mech_offset, "Mech offset: ");
    printTransform(viz_offset, "Viz offset: ");
    printTransform(mech_offset_desi, "Command: ");

    if (viz_offset.getOrigin().length() > 0.5 ||
        viz_offset.getRotation().getAngle() > (M_PI/4.0))
    {
      printf("WOAH!  Crazy vision offset!!!\n");
      g_state = MEASURING;
      break;
    }

    double error = sqrt(pow(viz_offset.getOrigin().y() - viz_offset_desi.getOrigin().y(), 2) +
                        pow(viz_offset.getOrigin().z() - viz_offset_desi.getOrigin().z(), 2));
    printf("error = %0.6lf\n", error);
    if (error < 0.002 && last_standoff < MIN_STANDOFF + 0.002)
      g_state = PUSHING;
    else
      g_state = MOVING;

    break;
  }

  case MOVING: {
    g_state = MEASURING;
    break;
  }

  case PUSHING: {
    tf::Vector3 offset = viz_offset.getOrigin() - viz_offset_desi.getOrigin();
    printf("Offset: (% 0.3lf, % 0.3lf, % 0.3lf)\n", offset.x(), offset.y(), offset.z());
    if (g_started_pushing + ros::Duration(5.0) < ros::Time::now())
    {
      if (offset.x() > SUCCESS_THRESHOLD) // if (in_outlet)
      {
        g_state = FORCING;
      }
      else
      {
        g_state = MEASURING;
      }
    }
    break;
  }

  case FORCING: {
    if (ros::Time::now() > g_started_forcing + ros::Duration(1.0))
      g_state = HOLDING;
    break;
  }

  case HOLDING: {
    //g_state = MEASURING;
    break;
  }
  }


  // Handles the transitions

  if (g_state != prev_state)
  {
    switch (g_state) {
    case MEASURING: {
      printf("enter MEASURING\n");
      if (prev_state == PUSHING || prev_state == FORCING)
      {
        robot_msgs::TaskFrameFormalism tff;
        tff.header.frame_id = "outlet_pose";
        tff.header.stamp = msg->header.stamp;
        tff.mode.vel.x = 3;
        tff.mode.vel.y = 3;
        tff.mode.vel.z = 3;
        tff.mode.rot.x = 3;
        tff.mode.rot.y = 3;
        tff.mode.rot.z = 3;
        tff.value.vel.x = mech_offset.getOrigin().x() - 0.05;  // backs off 5cm
        tff.value.vel.y = mech_offset.getOrigin().y() + 0.007;
        tff.value.vel.z = mech_offset.getOrigin().z() - 0.008;
        mech_offset.getBasis().getEulerZYX(tff.value.rot.z, tff.value.rot.y, tff.value.rot.x);
        ros::Node::instance()->publish("/arm_hybrid/command", tff);
        g_stopped_forcing = ros::Time::now();
        last_standoff = 0.05;
      }
      break;
    }

    case MOVING: {
      printf("enter MOVING\n");
      robot_msgs::TaskFrameFormalism tff;
      tff.header.frame_id = "outlet_pose";
      tff.header.stamp = msg->header.stamp;
      tff.mode.vel.x = 3;
      tff.mode.vel.y = 3;
      tff.mode.vel.z = 3;
      tff.mode.rot.x = 3;
      tff.mode.rot.y = 3;
      tff.mode.rot.z = 3;
      tff.value.vel.x = mech_offset_desi.getOrigin().x();
      tff.value.vel.y = mech_offset_desi.getOrigin().y();
      tff.value.vel.z = mech_offset_desi.getOrigin().z();
      mech_offset_desi.getBasis().getEulerZYX(tff.value.rot.z, tff.value.rot.y, tff.value.rot.x);
      ros::Node::instance()->publish("/arm_hybrid/command", tff);
      break;
    }

    case PUSHING: {
      printf("enter PUSHING\n");
      g_started_pushing = ros::Time::now();
      robot_msgs::TaskFrameFormalism tff;
      tff.header.frame_id = "outlet_pose";
      tff.header.stamp = msg->header.stamp;
      tff.mode.vel.x = 2;
      tff.mode.vel.y = 3;
      tff.mode.vel.z = 3;
      tff.mode.rot.x = 3;
      tff.mode.rot.y = 3;
      tff.mode.rot.z = 3;
      tff.value.vel.x = 0.3;
      tff.value.vel.y = mech_offset_desi.getOrigin().y();
      tff.value.vel.z = mech_offset_desi.getOrigin().z();
      mech_offset_desi.getBasis().getEulerZYX(tff.value.rot.z, tff.value.rot.y, tff.value.rot.x);
      ros::Node::instance()->publish("/arm_hybrid/command", tff);
      break;
    }

    case FORCING: {
      printf("enter FORCING\n");
      g_started_forcing = ros::Time::now();
      robot_msgs::TaskFrameFormalism tff;
      tff.header.frame_id = "outlet_pose";
      tff.header.stamp = msg->header.stamp;
      tff.mode.vel.x = 1;
      tff.mode.vel.y = 3;
      tff.mode.vel.z = 3;
      tff.mode.rot.x = 2;
      tff.mode.rot.y = 2;
      tff.mode.rot.z = 2;
      tff.value.vel.x = 50;
      tff.value.vel.y = mech_offset.getOrigin().y();
      tff.value.vel.z = mech_offset.getOrigin().z();
      tff.value.rot.x = 0.0;
      tff.value.rot.y = 0.0;
      tff.value.rot.z = 0.0;


      tff.mode.vel.y = 2;
      tff.mode.vel.z = 2;
      tff.value.vel.y = 0.0;
      tff.value.vel.z = 0.0;
      ros::Node::instance()->publish("/arm_hybrid/command", tff);
      break;
    }

    case HOLDING: {
      printf("enter HOLDING\n");
      g_started_holding = ros::Time::now();
      robot_msgs::TaskFrameFormalism tff;
      tff.header.frame_id = "outlet_pose";
      tff.header.stamp = msg->header.stamp;
      tff.mode.vel.x = 1;
      tff.mode.vel.y = 3;
      tff.mode.vel.z = 3;
      tff.mode.rot.x = 3;
      tff.mode.rot.y = 3;
      tff.mode.rot.z = 3;
      tff.value.vel.x = 4;
      tff.value.vel.y = mech_offset.getOrigin().y();
      tff.value.vel.z = mech_offset.getOrigin().z();
      mech_offset.getBasis().getEulerZYX(tff.value.rot.z, tff.value.rot.y, tff.value.rot.x);
      ros::Node::instance()->publish("/arm_hybrid/command", tff);
      break;
    }
    }
  }

  last_standoff = standoff;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv);
  ros::Node node("servo");

  node.advertise<robot_msgs::TaskFrameFormalism>("/arm_hybrid/command", 2);

  TF.reset(new tf::TransformListener(node));
  g_mn.reset(new tf::MessageNotifier<robot_msgs::PoseStamped>(
               TF.get(), &node, plug_cb, "/plug_detector/pose", "outlet_pose", 100));
  node.spin();

  return 0;
}
