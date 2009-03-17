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

enum {MEASURING, MOVING, PUSHING, FORCING};
int g_state = MEASURING;

ros::Time g_started_pushing, g_started_forcing;

bool g_settled = false;
robot_msgs::CartesianState g_hybrid_state;
void hybrid_state_cb()
{
  static double speed = 1.0;

  tf::Vector3 vel, rot;
  tf::PointMsgToTF(g_hybrid_state.last_twist_meas.vel, vel);
  tf::PointMsgToTF(g_hybrid_state.last_twist_meas.rot, rot);

  double s = vel.length() + 0.05 * rot.length();
  speed = s > speed ? s : (0.05 * s + 0.95 * speed);

  //printf("Speed: %lf\n", speed);
  if (!g_settled && speed < 0.4)
  {
    printf("Settled\n");
    g_settled = true;
  }
  if (g_settled && speed > 0.5)
  {
    printf("Moving again\n");
    g_settled = false;
  }
}

boost::scoped_ptr<tf::TransformListener> TF;

//robot_msgs::PoseStamped g_plug;
void plug_cb(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr &msg)
{
  if (!g_settled) {
    printf("Not yet settled\n");
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
    printf("FAIL: %s\n", ex.what());
    return;
  }

  tf::Pose viz_offset;
  tf::PoseMsgToTF(viz_offset_msg.pose, viz_offset);

  // The standoff is at least half the current distance
  double standoff = std::max(0.03, viz_offset.getOrigin().length() / 2.0);

  // Computes the offset for movement
  tf::Pose viz_offset_desi, mech_offset_desi;
  viz_offset_desi.setIdentity();
  viz_offset_desi.getOrigin().setX(-standoff);
  //mech_offset_desi = mech_offset + (viz_offset_desi - viz_offset);
  //mech_offset_desi = mech_offset * viz_offset.inverse() * viz_offset_desi;
  mech_offset_desi = viz_offset.inverse() * viz_offset_desi * mech_offset;


  switch (g_state) {
  case MEASURING: {
    printf("standoff = %.3lf\n", standoff);
    printf("Mech offset: (% .3lf, % .3lf, % .3lf)  <% .2lf, % .2lf, % .2lf, % .2lf>\n",
           mech_offset.getOrigin().x(),
           mech_offset.getOrigin().y(),
           mech_offset.getOrigin().z(),
           mech_offset.getRotation().x(),
           mech_offset.getRotation().y(),
           mech_offset.getRotation().z(),
           mech_offset.getRotation().w());
    printf("Viz offset: (% .3lf, % .3lf, % .3lf)  <% .2lf, % .2lf, % .2lf, % .2lf>\n",
           viz_offset.getOrigin().x(),
           viz_offset.getOrigin().y(),
           viz_offset.getOrigin().z(),
           viz_offset.getRotation().x(),
           viz_offset.getRotation().y(),
           viz_offset.getRotation().z(),
           viz_offset.getRotation().w());
    printf("Command: (% .3lf, % .3lf, % .3lf)  <% .2lf, % .2lf, % .2lf, % .2lf>\n",
           mech_offset_desi.getOrigin().x(),
           mech_offset_desi.getOrigin().y(),
           mech_offset_desi.getOrigin().z(),
           mech_offset_desi.getRotation().x(),
           mech_offset_desi.getRotation().y(),
           mech_offset_desi.getRotation().z(),
           mech_offset_desi.getRotation().w());

    if (viz_offset.getOrigin().length() > 0.5 ||
        viz_offset.getRotation().getAngle() > (M_PI/4.0))
    {
      printf("WOAH!  Crazy vision offset!!!\n");
      g_state = MEASURING;
      break;
    }

    // TODO: check (mechanism) velocities and throw out measurements taken while moving

    double error = sqrt(pow(viz_offset.getOrigin().y(), 2) + pow(viz_offset.getOrigin().z(), 2));
    printf("error = %0.6lf\n", error);
    if (error < 0.002)
    {
      printf("enter PUSHING\n");
      g_state = PUSHING;
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
      tff.value.vel.x = 0.6;
      tff.value.vel.y = mech_offset_desi.getOrigin().y();
      tff.value.vel.z = mech_offset_desi.getOrigin().z();
      mech_offset_desi.getBasis().getEulerZYX(tff.value.rot.z, tff.value.rot.y, tff.value.rot.x);
      ros::Node::instance()->publish("/arm_hybrid/command", tff);
    }
    else
    {
      g_state = MOVING;
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
    }

    break;
  }

  case MOVING: {
    // TODO: Wait to leave MOVING until settled
    g_state = MEASURING;
    break;
  }

  case PUSHING: {

    // TODO: When do we switch to FORCING?
    // - Push until we stop making forward progress
    // - Verify that we are in the outlet
    // But for now, we just make sure we've gone for a few seconds
    if (g_started_pushing + ros::Duration(3.0) > ros::Time::now())
    {
      if (true) // if (in_outlet)
      {
        printf("enter FORCING\n");
        g_state = FORCING;
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
        tff.value.vel.x = 40;
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
      }
      else
      {
        printf("enter MEASURING\n");
        g_state = MEASURING;
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
        tff.value.vel.y = mech_offset.getOrigin().y();
        tff.value.vel.z = mech_offset.getOrigin().z();
        mech_offset.getBasis().getEulerZYX(tff.value.rot.z, tff.value.rot.y, tff.value.rot.x);
        ros::Node::instance()->publish("/arm_hybrid/command", tff);
      }
    }

    break;
  }

  case FORCING: {

    // TODO: Get out of here if forcing is failing
    if (g_started_forcing + ros::Duration(5.0) > ros::Time::now())
    {
      printf("enter MEASURING\n");
      g_state = MEASURING;
      robot_msgs::TaskFrameFormalism tff;
      tff.header.frame_id = "outlet_pose";
      tff.header.stamp = msg->header.stamp;
      tff.mode.vel.x = 3;
      tff.mode.vel.y = 3;
      tff.mode.vel.z = 3;
      tff.mode.rot.x = 3;
      tff.mode.rot.y = 3;
      tff.mode.rot.z = 3;
      tff.value.vel.x = mech_offset.getOrigin().x() - 0.06;  // backs off
      tff.value.vel.y = mech_offset.getOrigin().y();
      tff.value.vel.z = mech_offset.getOrigin().z();
      mech_offset.getBasis().getEulerZYX(tff.value.rot.z, tff.value.rot.y, tff.value.rot.x);
      ros::Node::instance()->publish("/arm_hybrid/command", tff);
    }


    break;
  }
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv);
  ros::Node node("servo");

  node.advertise<robot_msgs::TaskFrameFormalism>("/arm_hybrid/command", 2);

  TF.reset(new tf::TransformListener(node));
  //tf::TransformListener TF(node);

  node.subscribe("/arm_hybrid/state", g_hybrid_state, hybrid_state_cb, 2);
  //node.subscribe("/plug_detector/pose", g_plug, plug_cb, 2);
  tf::MessageNotifier<robot_msgs::PoseStamped> mn(
    TF.get(), &node, plug_cb, "/plug_detector/pose", "outlet_pose", 5);
  node.spin();

  return 0;
}
