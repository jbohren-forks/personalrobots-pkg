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

#include <door_handle_detector/action_detect_door.h>
#include <door_handle_detector/DoorsDetectorCloud.h>
#include <point_cloud_assembler/BuildCloudAngle.h>


using namespace ros;
using namespace std;
using namespace door_handle_detector;


static const string fixed_frame = "odom_combined";




DetectDoorAction::DetectDoorAction(Node& node):
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("detect_door"),
  tf_(node)
{};


DetectDoorAction::~DetectDoorAction(){};



void DetectDoorAction::handleActivate(const robot_msgs::Door& door)
{
  ROS_INFO("DetectDoorAction: handle activate");
  request_preempt_ = false;
  notifyActivated();

  robot_msgs::Door result_laser;
  if (!laserDetection(door, result_laser)){
    if (request_preempt_){
      ROS_INFO("DetectDoorAction: Preempted");
      notifyPreempted(door);
    }
    else{
      ROS_INFO("DetectDoorAction: Aborted");
      notifyAborted(door);
    }
  }
  else{
    ROS_INFO("DetectDoorAction: Succeeded");
    notifySucceeded(result_laser);
  }
}




bool DetectDoorAction::laserDetection(const robot_msgs::Door& door_in, robot_msgs::Door& door_out)
{
  // check where robot is relative to door
  if (request_preempt_) return false;
  if (!tf_.canTransform("base_footprint", "laser_tilt_link", ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("DetectDoorAction: error getting transform from 'base_footprint' to 'laser_tilt_link'");
    return false;
  }
  tf::Stamped<tf::Transform> tilt_stage;
  tf_.lookupTransform("base_footprint", "laser_tilt_link", ros::Time(), tilt_stage);
  double laser_height = tilt_stage.getOrigin()[2];
  tf::Stamped<tf::Vector3> doorpoint(tf::Vector3((door_in.frame_p1.x+door_in.frame_p2.x)/2.0,
						 (door_in.frame_p1.y+door_in.frame_p2.y)/2.0,
						 (door_in.frame_p1.z+door_in.frame_p2.z)/2.0),
				     ros::Time(), door_in.header.frame_id);
  if (request_preempt_) return false;
  if (!tf_.canTransform("base_footprint", doorpoint.frame_id_, ros::Time(), ros::Duration().fromSec(5.0))){
    ROS_ERROR("DetectDoorAction: error getting transform from 'base_footprint' to '%s'", doorpoint.frame_id_.c_str());
    return false;
  }
  tf_.transformPoint("base_footprint", doorpoint, doorpoint);
  double dist = doorpoint[0];
  double door_bottom = -0.5;
  double door_top    =  2.5;
  ROS_INFO("DetectDoorAction: tilt laser is at height %f, and door at distance %f", laser_height, dist);

  // gets a point cloud from the point_cloud_srv
  if (request_preempt_) return false;
  ROS_INFO("DetectDoorAction: get a point cloud from the door");
  point_cloud_assembler::BuildCloudAngle::Request req_pointcloud;
  point_cloud_assembler::BuildCloudAngle::Response res_pointcloud;
  req_pointcloud.angle_begin = -atan2(door_top - laser_height, dist);
  req_pointcloud.angle_end = atan2(laser_height - door_bottom, dist);
  req_pointcloud.duration = 10.0;
  if (!ros::service::call("point_cloud_srv/single_sweep_cloud", req_pointcloud, res_pointcloud)){
    ROS_ERROR("DetectDoorAction: failed to get point cloud for door detection");
    return false;
  }

  // detect door
  if (request_preempt_) return false;
  ROS_INFO("DetectDoorAction: detect the door");
  door_handle_detector::DoorsDetectorCloud::Request  req_doordetect;
  door_handle_detector::DoorsDetectorCloud::Response res_doordetect;
  req_doordetect.door = door_in;
  req_doordetect.cloud = res_pointcloud.cloud;
  if (!ros::service::call("doors_detector_cloud", req_doordetect, res_doordetect)){
    ROS_ERROR("DetectDoorAction: failed to detect a door");
    return false;
  }

  door_out = res_doordetect.doors[0];
  return true;
}



void DetectDoorAction::handlePreempt()
{
  request_preempt_ = true;
};

