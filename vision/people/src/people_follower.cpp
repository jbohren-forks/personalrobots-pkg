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


using namespace std;
using namespace ros;
using namespace tf;
using namespace robot_msgs;



namespace estimation
{
  // constructor
  PeopleFollower::PeopleFollower(const string& node_name)
    : ros::Node(node_name),
      node_name_(node_name),
      robot_state_(*this, true),
      people_notifier_(NULL),
      initialized_(false)
  {
    // get parameters
    param("~/follow_distance", follow_distance_, 1.0);
    param("~/distance_threshold", distance_threshold_, 0.1);
    param("~/fixed_frame", fixed_frame_, string("map"));
    param("~/publish_rate", publish_rate_, 1.0);

    // advertise filter output
    advertise<robot_msgs::PositionMeasurement>("people_tracker_filter",10);

    // advertise visualization
    advertise<robot_msgs::PointCloud>("goal_pos",10);

    // register message sequencer
    people_notifier_ = new MessageNotifier<PositionMeasurement>(&robot_state_, this,  boost::bind(&PeopleFollower::callback, this, _1), 
                                                               "people_tracker_filter", fixed_frame_, 10);
    // advertise robot poses
    advertise<robot_msgs::Planner2DGoal>("goal", 10);

    // initialize goal
    people_pos_.header.frame_id = fixed_frame_;
    people_pos_.header.stamp = ros::Time().fromSec(0);
    people_pos_.enable = true;
    people_pos_.timeout = 1000000;

    time_last_publish_ = Time::now();

    // visualization
    robot_goal_cloud_.pts = vector<robot_msgs::Point32>(1);
    robot_goal_cloud_.pts[0].x = 0;
    robot_goal_cloud_.pts[0].y = 0;
    robot_goal_cloud_.pts[0].z = 0;
  }




  // destructor
  PeopleFollower::~PeopleFollower()
  {
    delete people_notifier_;
  };




  // callback for messages
  void PeopleFollower::callback(const MessageNotifier<PositionMeasurement>::MessagePtr& people_pos_msg)
  {
    // get people pos in fixed frame
    Stamped<tf::Vector3> people_pos_rel, people_pos_fixed_frame;
    people_pos_rel.setData(tf::Vector3(people_pos_msg->pos.x, people_pos_msg->pos.y, people_pos_msg->pos.z));
    people_pos_rel.stamp_    = people_pos_msg->header.stamp;
    people_pos_rel.frame_id_ = people_pos_msg->header.frame_id;
    robot_state_.transformPoint(fixed_frame_, people_pos_rel, people_pos_fixed_frame);

    // convert to planner 2D goal message
    people_pos_.goal.x = people_pos_fixed_frame.x();
    people_pos_.goal.y = people_pos_fixed_frame.y();
    people_pos_.goal.th = 0.0;

    if (initialized_){
      // calculate distance and angle
      double dx = people_pos_.goal.x - people_poses_.back().goal.x;
      double dy = people_pos_.goal.y - people_poses_.back().goal.y;
      double length = sqrt(pow(dx,2) + pow(dy,2));
      people_pos_.goal.th = atan2(dy, dx);

      // add to list buffer 
      if (length > distance_threshold_){
        distances_.push_back( distances_.back() + length);
        people_poses_.push_back(people_pos_);
      }

      // find next goal to send, which is follow_distance_ away from people_pos_
      while  (sqrt(pow(people_pos_.goal.x-people_poses_.front().goal.x,2) +
                   pow(people_pos_.goal.y-people_poses_.front().goal.y,2) ) > follow_distance_) {
        people_poses_.pop_front();
        distances_.pop_front();
      }
      robot_pos_.goal = people_poses_.front().goal;
      dx = people_pos_.goal.x - robot_pos_.goal.x;
      dy = people_pos_.goal.y - robot_pos_.goal.y;
      robot_pos_.goal.th = atan2(dy, dx);

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


      // send goal to planner
      if ((Time::now() - time_last_publish_).toSec() > 1/publish_rate_){
        publish("goal", people_poses_.front());
        time_last_publish_ = Time::now();
      }

      // visualize goal
      robot_goal_cloud_.pts[0].x = robot_pos_.goal.x;
      robot_goal_cloud_.pts[0].y = robot_pos_.goal.y;
      robot_goal_cloud_.pts[0].z = 0.0;
      robot_goal_cloud_.header.frame_id = fixed_frame_;
      publish("goal_pos",robot_goal_cloud_);
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
  // Initialize ROS
  ros::init(argc, argv);

  // create people follower
  PeopleFollower my_people_follower("people_follower");

  // wait for filter to finish
  my_people_follower.spin();

  return 0;
}
