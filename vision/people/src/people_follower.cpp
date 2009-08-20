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

/* Author: Wim Meeussen */

#include "people_follower.h"

#include <visualization_msgs/Marker.h>

using namespace std;
using namespace ros;
using namespace tf;

namespace estimation
{
  // constructor
  PeopleFollower::PeopleFollower(const string& node_name)
    : node_name_(node_name),
      people_notifier_(NULL),
      initialized_(false)
  {
    // get parameters
	node_.param("~/follow_distance", follow_distance_, 1.0);
	node_.param("~/distance_threshold", distance_threshold_, 0.1);
	node_.param("~/fixed_frame", fixed_frame_, string("map"));
	node_.param("~/publish_rate", publish_rate_, 1.0);

	// advertise filter output
	filter_output_pub_ = node_.advertise<people::PositionMeasurement>("people_tracker_filter",10);

    // advertise visualization
	viz_pub_ = node_.advertise<sensor_msgs::PointCloud>("goal_pos",10);

    // register message sequencer
    people_notifier_ = new MessageNotifier<people::PositionMeasurement>(robot_state_, boost::bind(&PeopleFollower::callback, this, _1),
                                                               "people_tracker_filter", fixed_frame_, 10);
    // advertise robot poses
    goal_pub_ = node_.advertise<pr2_robot_actions::Pose2D>("/move_base_local/activate", 10);

    marker_pub_ = node_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    // initialize goal
    people_pos_.header.frame_id = fixed_frame_;
    people_pos_.header.stamp = ros::Time().fromSec(0);

    robot_pos_.header.frame_id = fixed_frame_;
    robot_pos_.header.stamp = ros::Time().fromSec(0);

    time_last_publish_ = Time::now();

    // visualization
    robot_goal_cloud_.points = vector<geometry_msgs::Point32>(1);
    robot_goal_cloud_.points[0].x = 0;
    robot_goal_cloud_.points[0].y = 0;
    robot_goal_cloud_.points[0].z = 0;
  }




  // destructor
  PeopleFollower::~PeopleFollower()
  {
    delete people_notifier_;
  };

  void PeopleFollower::visualizeGoalPose(const geometry_msgs::PoseStamped& poseStamped)
  {
      cout<<"Sending vizualization message!"<<endl;
    visualization_msgs::Marker marker;
    marker.header = poseStamped.header;
    marker.ns = "follower";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; //Add and update are the same enum
    marker.pose = poseStamped.pose;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.7;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.3;

    ROS_INFO("setting viz: Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)\n",
	     marker.pose.position.x,
	     marker.pose.position.y,
	     marker.pose.position.z,
	     marker.pose.orientation.x,
	     marker.pose.orientation.y,
	     marker.pose.orientation.z,
	     marker.pose.orientation.w);

    marker_pub_.publish( marker );
  }

  // callback for messages
  void PeopleFollower::callback(const MessageNotifier<people::PositionMeasurement>::MessagePtr& people_pos_msg)
  {
	ROS_DEBUG("Got a people position measurement callback!");

    // get people pos in fixed frame
    Stamped<tf::Vector3> people_pos_rel, people_pos_fixed_frame;
    people_pos_rel.setData(tf::Vector3(people_pos_msg->pos.x, people_pos_msg->pos.y, people_pos_msg->pos.z));
    people_pos_rel.stamp_    = people_pos_msg->header.stamp;
    people_pos_rel.frame_id_ = people_pos_msg->header.frame_id;
    robot_state_.transformPoint(fixed_frame_, people_pos_rel, people_pos_fixed_frame);

    // convert to planner 2D goal message
    people_pos_.x = people_pos_fixed_frame.x();
    people_pos_.y = people_pos_fixed_frame.y();
    people_pos_.th = 0.0;

    if (initialized_){
      // calculate distance and angle
      double dx = people_pos_.x - people_poses_.back().x;
      double dy = people_pos_.y - people_poses_.back().y;
      double length = sqrt(pow(dx,2) + pow(dy,2));
      people_pos_.th = atan2(dy, dx);

      // add to list buffer 
      if (length > distance_threshold_){
        distances_.push_back( distances_.back() + length);
        people_poses_.push_back(people_pos_);
      }

      // find next goal to send, which is follow_distance_ away from people_pos_
      while  (sqrt(pow(people_pos_.x-people_poses_.front().x,2) +
                   pow(people_pos_.y-people_poses_.front().y,2) ) > follow_distance_) {
        people_poses_.pop_front();
        distances_.pop_front();
      }
      robot_pos_ = people_poses_.front();
      dx = people_pos_.x - robot_pos_.x;
      dy = people_pos_.y - robot_pos_.y;
      robot_pos_.th = atan2(dy, dx);

      /*
      cout << "person pos " 
           << people_pos_.goal.x << " "  
           << people_pos_.goal.y << " " 
           << people_pos_.goal.th << endl;
      cout << "robot  pos " 
           << robot_pos_.goal.x << " "
           << robot_pos_.goal.y << " "
           << robot_pos_.goal.th << endl;
      cout << "distance between them "<< sqrt(pow(people_pos_.goal.x-robot_pos_.goal.x,2) +
                                                    pow(people_pos_.goal.y-robot_pos_.goal.y,2) ) << endl;
      */


      // send goal to planner
      if ((Time::now() - time_last_publish_).toSec() > 1.0/publish_rate_){
        //publish("goal", robot_pos_);
	    geometry_msgs::PoseStamped goal_pos_;
        goal_pos_.header = people_pos_.header;
        goal_pos_.pose.position.x = robot_pos_.x; //people_pose_ or robot_pos_?
        goal_pos_.pose.position.y = robot_pos_.y;
        goal_pos_.pose.position.z = robot_pos_.z;
        goal_pos_.pose.orientation.x = 1.0;

        visualizeGoalPose( goal_pos_ );
        goal_pub_.publish( goal_pos_);

        time_last_publish_ = Time::now();
      }

      // visualize goal
      robot_goal_cloud_.points[0].x = robot_pos_.x;
      robot_goal_cloud_.points[0].y = robot_pos_.y;
      robot_goal_cloud_.points[0].z = 0.0;
      robot_goal_cloud_.header.frame_id = fixed_frame_;
      viz_pub_.publish(robot_goal_cloud_);
    }
    

    // not initialized yet
    else{
      cout << "Initializing" << endl;
      people_poses_.push_back(people_pos_);
      distances_.push_back(0.0);
      initialized_ = true;
    }
  }
} // namespace



// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  std::string node_name = "people_follower";

  // Initialize ROS
  ros::init(argc, argv, node_name);

  // create people follower
  PeopleFollower my_people_follower(node_name);

  // wait for filter to finish
  ros::spin();

  return 0;
}
