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
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Wim Meeussen */

#include <door_handle_detector/DoorsDetectorCloud.h>
#include <door_handle_detector/DoorsDetector.h>
#include <door_functions/door_functions.h>
#include <door_msgs/Door.h>
#include <boost/thread/thread.hpp>
#include <point_cloud_assembler/BuildCloudAngle.h>
#include "doors_core/action_detect_handle_no_camera.h"

using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace door_functions;


static const string fixed_frame = "odom_combined";
static const double scan_speed  = 0.1; // [m/sec]
static const double scan_height = 0.4; //[m]
static const unsigned int max_retries = 5;

DetectHandleNoCameraAction::DetectHandleNoCameraAction(Node& node, tf::TransformListener& tf): 
  robot_actions::Action<door_msgs::Door, door_msgs::Door>("detect_handle_no_camera"),
  node_(node),
  tf_(tf)
{
  node_.advertise<robot_msgs::PointStamped>("head_controller/head_track_point",10);
};


DetectHandleNoCameraAction::~DetectHandleNoCameraAction()
{
  node_.unadvertise("head_controller/head_track_point");
};



robot_actions::ResultStatus DetectHandleNoCameraAction::execute(const door_msgs::Door& goal, door_msgs::Door& feedback)
{
  ROS_INFO("DetectHandleNoCameraAction: execute");

  // default feedback
  feedback = goal;

  // transform door message to time fixed frame
  door_msgs::Door goal_tr;
  transformTo(tf_, fixed_frame, goal, goal_tr, fixed_frame);

  // try to detect handle 
  for (unsigned int nr_tries=0; nr_tries<max_retries; nr_tries++){

    // check for preemption
    if (isPreemptRequested()){
      ROS_INFO("DetectHandleNoCameraAction: Preempted");
      return robot_actions::PREEMPTED;
    }

    boost::thread* thread_laser,* thread_camera;
    bool success_laser, success_camera;
    door_msgs::Door result_laser, result_camera;
    thread_laser =  new boost::thread(boost::bind(&DetectHandleNoCameraAction::laserDetectionFunction, this, goal_tr, &result_laser, &success_laser));
    thread_camera = new boost::thread(boost::bind(&DetectHandleNoCameraAction::cameraDetectionFunction, this, goal_tr, &result_camera, &success_camera));

    thread_laser->join();
    thread_camera->join();
    delete thread_laser;
    delete thread_camera;
    ROS_INFO("Laser success %i, Camera success %i", success_laser, success_camera);

    success_camera = true;
    result_camera = result_laser;

    if (!success_laser && !success_camera){
      ROS_ERROR("Laser and Camera detection failed");
      continue;
    }
    if (!success_laser){
      ROS_ERROR("Laser detection failed");
      continue;
    }
    if (!success_camera){
      ROS_ERROR("Camera detection failed");
      continue;
    }

    // transform laser data
    if (!transformTo(tf_, fixed_frame, result_laser, result_laser, fixed_frame)){
      ROS_ERROR ("Could not transform laser door message from frame %s to frame %s.",
		 result_laser.header.frame_id.c_str (), fixed_frame.c_str ());
      return robot_actions::ABORTED;
    }
    ROS_INFO("detected handle position transformed to '%s'", fixed_frame.c_str());

    // transform camera data
    if (!transformTo(tf_, fixed_frame, result_camera, result_camera, fixed_frame)){
      ROS_ERROR ("Could not transform camera door message from frame %s to frame %s.",
		 result_camera.header.frame_id.c_str (), fixed_frame.c_str ());
      return robot_actions::ABORTED;
    }
    ROS_INFO("detected handle position transformed to '%s'", fixed_frame.c_str());

    result_camera = result_laser;
    // compare laser and camera results
    double  error = sqrt(pow(result_laser.handle.x - result_camera.handle.x,2) +
			 pow(result_laser.handle.y - result_camera.handle.y,2) +
			 pow(result_laser.handle.z - result_camera.handle.z,2));
    ROS_INFO("difference between laser and camera result = %f", error);
    if (error < 0.1){
      // store handle position
      feedback = result_laser;
      feedback.handle.x = (result_laser.handle.x + result_camera.handle.x)/2.0;
      feedback.handle.y = (result_laser.handle.y + result_camera.handle.y)/2.0;
      feedback.handle.z = (result_laser.handle.z + result_camera.handle.z)/2.0;
      
      ROS_INFO("Found handle in %i tries", nr_tries+1);
      return robot_actions::SUCCESS;
    }
  }
  ROS_ERROR("Did not find hanlde in %i tries", max_retries);
  return robot_actions::ABORTED;
}



void DetectHandleNoCameraAction::laserDetectionFunction(const door_msgs::Door& door_in,
						door_msgs::Door* door_out, bool* success)
{
  *success = laserDetection(door_in, *door_out);
}

bool DetectHandleNoCameraAction::laserDetection(const door_msgs::Door& door_in,
                                        door_msgs::Door& door_out)
{
  // check where robot is relative to the door
  if (isPreemptRequested()) return false;
  if (!tf_.canTransform("base_footprint", "laser_tilt_link", ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("could not get transform from 'base_footprint' to 'laser_tilt_link'");
    return false;
  }
  tf::Stamped<tf::Transform> tilt_stage;
  tf_.lookupTransform("base_footprint", "laser_tilt_link", ros::Time(), tilt_stage);
  ROS_INFO("handle activate");
  double laser_height = tilt_stage.getOrigin()[2];
  tf::Stamped<tf::Vector3> handlepoint(tf::Vector3((door_in.door_p1.x+door_in.door_p2.x)/2.0,
						   (door_in.door_p1.y+door_in.door_p2.y)/2.0,
						   0.9),
				       ros::Time(), door_in.header.frame_id);
  if (!tf_.canTransform("base_footprint", handlepoint.frame_id_, ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("could not get transform from 'base_footprint' to '%s'", handlepoint.frame_id_.c_str());
    return false;
  }
  tf_.transformPoint("base_footprint", handlepoint, handlepoint);
  double dist = handlepoint[0];
  double handle_bottom = handlepoint[2]-(scan_height/2.0);
  double handle_top = handlepoint[2]+(scan_height/2.0);
  ROS_INFO("tilt laser is at height %f, and door at distance %f", laser_height, dist);
  
  // gets a point cloud from the point_cloud_srv
  if (isPreemptRequested()) return false;
  ROS_INFO("get a point cloud from the door");
  point_cloud_assembler::BuildCloudAngle::Request req_pointcloud;
  point_cloud_assembler::BuildCloudAngle::Response res_pointcloud;
  req_pointcloud.angle_begin = -atan2(handle_top - laser_height, dist);
  req_pointcloud.angle_end = atan2(laser_height - handle_bottom, dist);
  req_pointcloud.duration = scan_height/scan_speed;
  if (!ros::service::call("point_cloud_srv/single_sweep_cloud", req_pointcloud, res_pointcloud)){
    ROS_ERROR("failed to get point cloud for door detection");
    return false;
  }

  // detect handle
  if (isPreemptRequested()) return false;
  ROS_INFO("start detecting the handle using the laser, in a pointcloud of size %i", res_pointcloud.cloud.pts.size());
  door_handle_detector::DoorsDetectorCloud::Request  req_handledetect;
  door_handle_detector::DoorsDetectorCloud::Response res_handledetect;
  req_handledetect.door = door_in;
  req_handledetect.cloud = res_pointcloud.cloud;
  if (!ros::service::call("handle_detector_cloud", req_handledetect, res_handledetect)){
    ROS_ERROR("failed to detect a handle using the laser");
    return false;
  }

  door_out = res_handledetect.doors[0];
  return true;
}


void DetectHandleNoCameraAction::cameraDetectionFunction(const door_msgs::Door& door_in,
						 door_msgs::Door* door_out, bool* success)
{
  *success = cameraDetection(door_in, *door_out);
}


bool DetectHandleNoCameraAction::cameraDetection(const door_msgs::Door& door_in,
                                         door_msgs::Door& door_out)
{
  // make the head point towards the door
  if (isPreemptRequested()) return false;
  ROS_INFO("point head towards door");
  robot_msgs::PointStamped door_pnt;
  door_pnt.header.frame_id = door_in.header.frame_id;
  door_pnt.point.x = (door_in.door_p1.x+door_in.door_p2.x)/2.0;
  door_pnt.point.y = (door_in.door_p1.y+door_in.door_p2.y)/2.0;
  door_pnt.point.z = 0.9;
  cout << "door_pnt.point " << door_in.header.frame_id << " " 
       << door_pnt.point.x << " " 
       << door_pnt.point.y << " " 
       <<  door_pnt.point.z << endl;
  node_.publish("head_controller/head_track_point", door_pnt);
  ros::Duration().fromSec(2).sleep();

  // detect handle
  if (isPreemptRequested()) return false;
  ROS_INFO("start detecting the handle using the camera");
  door_handle_detector::DoorsDetector::Request  req_handledetect;
  door_handle_detector::DoorsDetector::Response res_handledetect;
  req_handledetect.door = door_in;
  if (!ros::service::call("door_handle_vision_detector", req_handledetect, res_handledetect)){
    ROS_ERROR("failed to detect a handle using the camera");
    return false;
  }

  door_out = res_handledetect.doors[0];
  return true;
}
