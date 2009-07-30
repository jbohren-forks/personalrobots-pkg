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


#include <boost/shared_ptr.hpp>
#include <ros/node.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/time.h>
#include <people/PositionMeasurement.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/PointStamped.h>
#include <robot_msgs/PoseStamped.h>
#include "people_aware_nav/LookStraightAhead.h"
#include "people_aware_nav/GlanceAt.h"
#include "people_aware_nav/StartHeadTrack.h"

namespace people_aware_nav
{

using robot_msgs::Point;
using robot_msgs::PointStamped;
using robot_msgs::PoseStamped;
using ros::Node;
using ros::Time;
using ros::Duration;
using std::string;


// A node that publishes head control messages for various types of high-level head actions.
class HeadController
{
public:
  HeadController() : node_("head_controller")
  {

    node_.param("~head_control_type", head_control_type_, string("look_forward"));
    node_.param("~default_speed", default_speed_, 1.0);
    // look_forward, glance, look_at_goal

    ROS_INFO_STREAM("Head exec type is " << head_control_type_);

    do_continuous_ = false;
    have_goal_ = false;

    node_.advertise<PointStamped>("/head_controller/point_head",1);

    node_.advertiseService ("look_straight_ahead", &HeadController::lookStraightAhead, this);
    node_.advertiseService ("glance_at", &HeadController::glanceAt, this);
    node_.advertiseService ("start_head_track", &HeadController::startHeadTrack, this);

    if (head_control_type_ == "look_forward") {
      do_continuous_ = false;
      state_ = STRAIGHT;
      lookStraight();
    }
    else if (head_control_type_ == "look_at_goal") {
      do_continuous_ = true;
      state_ = TRACK;
      node_.subscribe("/hallway_move/goal", goal_pose_, &HeadController::goalCallback, this, 1); //goal
    }

      
  }

  // Service callbacks
  bool lookStraightAhead(people_aware_nav::LookStraightAhead::Request &req, people_aware_nav::LookStraightAhead::Response& resp) {

    do_continuous_ = false;
    state_ = STRAIGHT;
    lookStraight();
    return true;
  }

  bool glanceAt(people_aware_nav::GlanceAt::Request &req, people_aware_nav::GlanceAt::Response& resp) {

    do_continuous_ = false; // Turn off track point publishing
    node_.publish("/head_controller/point_head",req.point_stamped);
    usleep(2000000);
    if (state_== TRACK) {
      lookAtGoal(); // Look at the goal once with a reasonable speed before passing it back to the continuous publication.
      do_continuous_ = true;
    }
    else {
      lookStraight();
    }
    return true;
    
  }
 
  bool startHeadTrack(people_aware_nav::StartHeadTrack::Request &req, people_aware_nav::StartHeadTrack::Response& resp) {
    do_continuous_ = true;
    state_ = TRACK;
    node_.subscribe("goal", goal_pose_, &HeadController::goalCallback, this, 1); //goal                                                                              
    return true;
  }

  //void goalCallback(const GNotifier::MessagePtr& goal_message) 
  void goalCallback()
  {
    //   goal_pose_= *goal_message;
    have_goal_ = true;
  }

  void spin () 
  {

    while (node_.ok()) 
    {
      if (do_continuous_ && have_goal_)
      {
	lookAtGoal(-1);
	usleep(50000);	
      }
    }
  }


private:
  Node node_;
  string head_control_type_;
  double default_speed_;
  robot_msgs::PoseStamped goal_pose_;
  bool do_continuous_, have_goal_;

  enum {NONE=0, TRACK=1, STRAIGHT=2};
  int state_;

  void lookStraight() {
    lookStraight(default_speed_);
  }
 
  void lookStraight( double speed) {
    if (speed < 0.0) {
      // As fast as possible
      PointStamped p;
      p.header.frame_id = "base_link";
      p.point.x = 1;
      p.point.y = 0;
      p.point.z = 1.1;
      node_.publish("/head_controller/point_head",p);
    }
    else {
      // Change this to include the speed.
      PointStamped p;
      p.header.frame_id = "base_link";
      p.point.x = 1;
      p.point.y = 0;
      p.point.z = 1.1;
      node_.publish("/head_controller/point_head",p);
    }
  }

  void lookAtGoal() {
    lookAtGoal(default_speed_);
  }

  void lookAtGoal(double speed) 
  {
    if (speed < 0.0) {
      // As fast as possible
      PointStamped p;
      p.header.frame_id = goal_pose_.header.frame_id;
      p.point = goal_pose_.pose.position;
      node_.publish("/head_controller/head_track_point",p);
    }
    else {
      // Change this to include the speed.
      PointStamped p;
      p.header.frame_id = goal_pose_.header.frame_id;
      p.point = goal_pose_.pose.position;
      node_.publish("/head_controller/head_track_point",p);
    }
  }

}; // class
  
} // namespace


int main (int argc, char** argv)
{
  ros::init(argc, argv);
  people_aware_nav::HeadController hc;
  hc.spin();
}


  

  
