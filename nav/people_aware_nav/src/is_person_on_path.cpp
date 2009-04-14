/**********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>

#include <ros/node.h>
#include <robot_msgs/PositionMeasurement.h>
#include <robot_msgs/Polyline2D.h>
//#include <topic_synchronizer/topic_synchronizer.h>
#include "tf/transform_listener.h"
#include <tf/message_notifier.h>
#include <people_aware_nav/PersonOnPath.h>
#include <boost/thread/mutex.hpp>

namespace people_aware_nav
{

class IsPersonOnPath
{
public:
  ros::Node *node_;
  tf::TransformListener *tf_;
  tf::MessageNotifier<robot_msgs::PositionMeasurement>* message_notifier_person_;
  tf::MessageNotifier<robot_msgs::Polyline2D>* message_notifier_path_;
  robot_msgs::PositionMeasurement person_pos_;
  robot_msgs::Polyline2D path_;
  bool got_person_pos_, got_path_;
  std::string fixed_frame_;
  double total_dist_sqr_m_;
  boost::mutex path_mutex_, person_mutex_;


  IsPersonOnPath() 
  {
    node_ = ros::Node::instance();
    tf_ = new tf::TransformListener(*node_); 

    got_person_pos_ = false;
    got_path_ = false;

    double robot_radius_m, person_radius_m;
    node_->param("~/fixed_frame", fixed_frame_, std::string("map"));
    node_->param("~costmap_2d/circumscribed_radius", robot_radius_m, 0.46);
    node_->param("~/person_radius", person_radius_m, 0.5); 
    total_dist_sqr_m_ = robot_radius_m*robot_radius_m + person_radius_m*person_radius_m;

    message_notifier_person_ = new tf::MessageNotifier<robot_msgs::PositionMeasurement> (tf_, node_, boost::bind(&IsPersonOnPath::personPosCB, this, _1), "people_tracker_measurements", fixed_frame_, 1);
    message_notifier_path_ = new tf::MessageNotifier<robot_msgs::Polyline2D>(tf_, node_, boost::bind(&IsPersonOnPath::pathCB, this, _1), "gui_path", fixed_frame_, 1);

    node_->advertiseService ("is_person_on_path", &IsPersonOnPath::personOnPathCB, this);
    path_mutex_.lock();
    ROS_INFO("locked");
    path_mutex_.unlock();
    ROS_INFO ("Done initializing node");
    
  }

  ~IsPersonOnPath()
  {
  }

  // Person callback
  void personPosCB(const tf::MessageNotifier<robot_msgs::PositionMeasurement>::MessagePtr& person_pos_msg) 
  {
    boost::mutex::scoped_lock l2(person_mutex_);
    person_pos_ = *person_pos_msg;
    got_person_pos_ = true;
    ROS_INFO_STREAM ("Have seen person and got_person_ is " << got_person_pos_);
  }

  // Path callback
  void pathCB(const tf::MessageNotifier<robot_msgs::Polyline2D>::MessagePtr& gui_path_msg)
  {
    boost::mutex::scoped_lock l(path_mutex_);
    ROS_INFO_STREAM ("In path callback and got_path_ is " << got_path_);
    path_ = *gui_path_msg;
    got_path_ = true;
  }

  // Service callback
  bool personOnPathCB (people_aware_nav::PersonOnPath::Request& req, people_aware_nav::PersonOnPath::Response& resp)
  {
    boost::mutex::scoped_lock l(path_mutex_);
    boost::mutex::scoped_lock l2(person_mutex_);
    
    bool on_path;
    float dist;
    ros::Time current_time, person_stamp, path_stamp;
    bool ok = personPathDist(&on_path, &dist, &current_time, &person_stamp, &path_stamp);
    if (!ok) {
      ROS_WARN_STREAM ("Unable to tell if person was on path.  Ret vals: Current time " << current_time << " Person stamp : " << person_stamp << " Path stamp : " << path_stamp);
    }
    resp.value = on_path ? 1 : 0;
    ROS_DEBUG_STREAM_NAMED ("person_on_path", "Person on path value is " << (int) resp.value << " at distance " << dist);


    return true;


  }

  // Compute distance between the person and the path. Return true if distance was calculated, false otherwise (eg if the transforms didn't work).
  bool personPathDist(bool* is_on_path, float* dist, ros::Time* current_time, ros::Time* person_stamp, ros::Time* path_stamp)
  {
    *is_on_path = false;
    *dist = 0;
    *person_stamp = ros::Time().fromSec(0);
    *path_stamp = ros::Time().fromSec(0);
    *current_time = ros::Time().fromSec(0);
    
    // Check that we have both the person and path messages.
    if (!got_person_pos_ || !got_path_)
    {
      ROS_INFO_COND (!got_person_pos_, "Didn't have person");
      ROS_INFO_COND (!got_path_, "Didn't have path");
      std::cout << "\nGot path is " << got_path_;
      return false;
    }

    // Convert the path and the person to the current time. If either is stale (too old for TF), mark them as unusable and return false.
    (*current_time) = (person_pos_.header.stamp > path_.header.stamp) ? person_pos_.header.stamp : path_.header.stamp;
    tf::Point pt;
    tf::PointMsgToTF(person_pos_.pos, pt);
    tf::Stamped<tf::Point> t_person_tf_stamped_point(pt, person_pos_.header.stamp, person_pos_.header.frame_id);
    try {   
      tf_->transformPoint(fixed_frame_, *current_time, t_person_tf_stamped_point, fixed_frame_, t_person_tf_stamped_point);
    }
    catch (tf::TransformException& ex) {
      ROS_INFO_STREAM ("Unable to do person transformation" << ex.what());
      ROS_INFO_STREAM ("Image time : " << person_pos_.header.stamp << ".  Current time : " << *current_time << ". Rostime " << ros::Time::now());
      got_person_pos_ = false;
      return false;
    }

    ROS_DEBUG_STREAM_NAMED ("person_on_path", "Attempting to compute person-path dist to point " << t_person_tf_stamped_point[0] << ", " << t_person_tf_stamped_point[1]);



    // Go through each line segment on the path and get the distance to the person.
    double min_dist = -1.0;
    for (uint i=0; i<path_.points.size()-1; i++) {


      // Two points on the line segment
      tf::Point tpt1;
      tpt1[0] = path_.points[i].x;
      tpt1[1] = path_.points[i].y;
      tpt1[2] = 0.0;
      tf::Stamped<tf::Point> t_path_point1(tpt1, path_.header.stamp, path_.header.frame_id);
      tf::Point tpt2;
      tpt2[0] = path_.points[i+1].x;
      tpt2[1] = path_.points[i+1].y;
      tpt2[2] = 0.0;
      tf::Stamped<tf::Point> t_path_point2(tpt2, path_.header.stamp, path_.header.frame_id);
      
      // Try to transform the line endpoints to the current time
      try {
	tf_->transformPoint(fixed_frame_, *current_time, t_path_point1, fixed_frame_, t_path_point1);
      }
      catch (tf::TransformException& ex) {
        ROS_INFO_STREAM ("Unable to do path transformation " << ex.what());
	return false;
      }      

      try {
	tf_->transformPoint(fixed_frame_, *current_time, t_path_point2, fixed_frame_, t_path_point2);
      }
      catch (tf::TransformException& ex) {
        ROS_INFO_STREAM ("Unable to do path transformation: " << ex.what());
	return false;
      }

      
      // Distance from point to line:
      tf::Vector3 s, d;
      double dotss, dotdd;
      s = t_path_point2 - t_path_point1;

      // Check that the length of the line isn't 0.
      dotss = dot(s,s);
      if (dotss < 1E-10) {
	ROS_DEBUG_STREAM_NAMED("person_on_path","Line segment length is too small, skipping.");
	continue;
      }

      d = t_person_tf_stamped_point - t_path_point1;
      dotdd = dot(d,d);
      if (dotdd < 1E-10) {
	ROS_DEBUG_STREAM_NAMED("person_on_path", "Person is (almost) at line segment point 1. Returning distance of 0.");
	min_dist = 0.0;
	break;
      }

      tf::Vector3 c;
      c = cross(s,d);
      double sqr_dist = dot(c,c) / dotss;

      if (min_dist < 0.0 || sqr_dist < min_dist) {

	// Check that the point actually projects onto the line segment. If not, the closest distance is really to an endpoint.
	double k = ( dot( t_person_tf_stamped_point, s) - dot(t_path_point1,s) ) / dotss;
	if (k < 0) {
          ROS_DEBUG_STREAM_NAMED ("person_on_path", "endpoint1");
	  sqr_dist = dotdd;
	}
	else if (k > 1) {
          ROS_DEBUG_STREAM_NAMED ("person_on_path", "endpoint2");
	  d = t_person_tf_stamped_point - t_path_point2;
	  dotdd = dot(d,d);
	  if (dotdd < 1E-10) {
	    ROS_DEBUG_STREAM_NAMED("person_on_path", "Person is (almost) at line segment point 2. Returning distance of 0.");
	    min_dist = 0.0;
	    break;
	  }
	  sqr_dist = dotdd;
	}
	
	// If this is the closest line segment to the person, set the min distance.
	if (min_dist < 0.0 || sqr_dist < min_dist) {
	  min_dist = sqr_dist;
	}
	
      }

      ROS_DEBUG_STREAM_NAMED ("person_on_path", "Sq. distance from " << t_path_point1[0] << ", " << t_path_point1[1] << " to " << t_path_point2[0] << ", " << t_path_point2[1] << " is " << sqr_dist);


    }

    // The person is on the path if their min distance to one of the line segments is less than their radius plus the robot's radius.
    if (min_dist >= 0.0 && min_dist <= total_dist_sqr_m_) {
      *is_on_path = true;
      *dist = sqrt(min_dist);
    }
    else {
      *is_on_path = false;
    }


    *person_stamp = person_pos_.header.stamp;
    *path_stamp = path_.header.stamp;

    return true;
  }

}; // class

}; // namespace


int
main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node q("is_person_on_path");
  people_aware_nav::IsPersonOnPath qt;
  
  qt.path_mutex_.lock();
  ROS_INFO ("locked in main");
  qt.path_mutex_.unlock();

  q.spin ();
  return (0);
}
