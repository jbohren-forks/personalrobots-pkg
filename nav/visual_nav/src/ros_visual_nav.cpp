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

/**
 * \mainpage
 *
 * \htmlinclude manifest.html
 *
 * \author Bhaskara Marthi
 *
 *
 **/



#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <ros/node.h>
#include <ros/time.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <visual_nav/visual_nav.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <robot_msgs/Planner2DGoal.h>
#include <robot_msgs/VisualizationMarker.h>


using std::string;
using robot_msgs::Planner2DGoal;
using deprecated_msgs::RobotBase2DOdom;
using ros::Duration;

namespace visual_nav
{



/************************************************************
 * Node class
 ************************************************************/

class RosVisualNavigator
{
public:
  
  // Initialize object, ros node
  RosVisualNavigator (double exit_point_radius, const string& roadmap_filename, NodeId goal);

  // Subscribe topics
  void setupTopics();

  // Spin
  void run();

  // Callback
  void odomCallback();

private:

  void publishMarker (double x, double y, double theta, int id);

  ros::Node node_;

  RoadmapPtr roadmap_;

  // has at least one odometry message been received?
  bool odom_received_;

  RobotBase2DOdom odom_message_;

  // the radius of the window used to find the exit point of the path 
  double exit_point_radius_;

  NodeId goal_id_;

  // Transform between the nav frame (used by the visual roadmap) and the odometry frame (used to give goals to local controller)
  Transform2D nav_odom_transform_;

};


/************************************************************
 * internal
 ************************************************************/

Pose poseFromMessage(const RobotBase2DOdom& odom_msg)
{
  return Pose(odom_msg.pos.x, odom_msg.pos.y, odom_msg.pos.th);
}

  
Planner2DGoal goalFromPose (const Pose& pose)
{
  Planner2DGoal m;
  m.goal.x = pose.x;
  m.goal.y = pose.y;
  m.goal.th = pose.theta;
  m.header.frame_id = "odom_combined_offset";
  m.enable = 1;
  return m;
}



/************************************************************
 * top level
 ************************************************************/


RosVisualNavigator::RosVisualNavigator (double exit_point_radius, const string& roadmap_filename, NodeId goal) : 
  node_("visual_navigator"), odom_received_(false), exit_point_radius_(exit_point_radius), goal_id_(goal)
{
  roadmap_=readRoadmapFromFile(roadmap_filename);
}

void RosVisualNavigator::setupTopics ()
{
  node_.subscribe("odom", odom_message_, &RosVisualNavigator::odomCallback, this, 1);
  node_.advertise<Planner2DGoal>("goal", 1);
  node_.advertise<robot_msgs::VisualizationMarker>( "visualizationMarker", 0 );
}

void RosVisualNavigator::publishMarker (double x, double y, double theta, int id) 
{
  robot_msgs::VisualizationMarker marker;
  marker.header.frame_id = "odom_combined_offset";
  marker.header.stamp = ros::Time((uint64_t)0ULL);
  marker.id = id;
  marker.type = robot_msgs::VisualizationMarker::SPHERE;
  marker.action = robot_msgs::VisualizationMarker::ADD;
  marker.x = x;
  marker.y = y;
  marker.z = 0;
  marker.yaw = theta;
  marker.pitch = 0.0;
  marker.roll = 0.0;
  marker.xScale = .3;
  marker.yScale = .3;
  marker.zScale = .3;
  marker.alpha = 255;
  marker.r = 0;
  marker.g = 255;
  marker.b = 0;
  node_.publish( "visualizationMarker", marker );
}



void RosVisualNavigator::run ()
{
  // Currently just plan once at the beginning
  PathPtr path = roadmap_->pathToGoal(goal_id_);
  
  Duration d(1);

  while (!odom_received_) {
    d.sleep();
    ROS_DEBUG_NAMED ("node", "Waiting for odometry message");
  }

  // Once we've received the initial odometry message, determine the transform between the frames
  // We're assuming we're stationary right now
  odom_message_.lock();
  Pose odom_pose=poseFromMessage(odom_message_);
  odom_message_.unlock();
  Pose pose_in_nav_frame = roadmap_->estimatedPose(path);
  nav_odom_transform_ = getTransformBetween(pose_in_nav_frame, odom_pose);

  // Command the base to the exit point
  Pose exit_point = roadmap_->pathExitPoint(path, exit_point_radius_);
  Planner2DGoal goal_message = goalFromPose(exit_point);
  ROS_DEBUG_NAMED ("node", "Publishing goal (%f, %f, %f) (in odometry frame)", goal_message.goal.x, goal_message.goal.y, goal_message.goal.th);

  while (true) {
    node_.publish("goal", goal_message);
    for (int i=1; i<=5; i++) {
      Pose p = transform (nav_odom_transform_, roadmap_->nodePose(i));
      
      ROS_DEBUG_NAMED ("node", "Publishing marker %f, %f, %f", p.x, p.y, p.theta);
      publishMarker (p.x, p.y, p.theta, i);
    }
    d.sleep();
  }
}



/************************************************************
 * callbacks
 ************************************************************/

void RosVisualNavigator::odomCallback ()
{
  odom_received_=true;
}



} // namespace visual_nav


using visual_nav::RosVisualNavigator;



int main(int argc, char** argv)
{
  ros::init (argc, argv);

  // Eventually, parse the command line arguments

   RosVisualNavigator nav(5.0, string("local/vis/roadmap.dat"), 5);

   nav.setupTopics();

   nav.run();

}


