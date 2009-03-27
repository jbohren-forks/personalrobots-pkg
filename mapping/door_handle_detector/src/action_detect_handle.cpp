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

#include <door_handle_detector/action_detect_handle.h>
#include <door_handle_detector/DoorsDetectorCloud.h>
#include <point_cloud_assembler/BuildCloudAngle.h>


using namespace ros;
using namespace std;
using namespace door_handle_detector;


static const string fixed_frame = "odom_combined";




DetectHandleAction::DetectHandleAction(Node& node): 
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("detect_door_action"),
  tf_(node)
{};


DetectHandleAction::~DetectHandleAction(){};



void DetectHandleAction::handleActivate(const robot_msgs::Door& door)
{
  ROS_INFO("DetectHandleAction: handle activate");
  request_preempt_ = false;
  notifyActivated();

  // check where robot is relative to handle
  if (!tf_.canTransform("base_footprint", "laser_tilt_link", ros::Time(), ros::Duration().fromSec(5.0)))
    ROS_ERROR("DetectHandleAction: error getting transform");
  tf::Stamped<tf::Transform> tilt_stage;
  tf_.lookupTransform("base_footprint", "laser_tilt_link", ros::Time(), tilt_stage);
  ROS_INFO("DetectHandleAction: handle activate");
  double laser_height = tilt_stage.getOrigin()[2];
  tf::Stamped<tf::Vector3> handlepoint(tf::Vector3((door.door_p1.x+door.door_p2.x)/2.0,
						   (door.door_p1.y+door.door_p2.y)/2.0,
						   (door.door_p1.z+door.door_p2.z)/2.0),
				       ros::Time(), door.header.frame_id);
  if (!tf_.canTransform("base_footprint", handlepoint.frame_id_, ros::Time(), ros::Duration().fromSec(5.0)))
    ROS_ERROR("DetectHandleAction: error getting transform");
  tf_.transformPoint("base_footprint", handlepoint, handlepoint);
  double dist = handlepoint[0];
  double handle_bottom = handlepoint[2]-0.4;
  double handle_top = handlepoint[2]+0.4;
  ROS_INFO("DetectHandleAction: tilt laser is at height %f, and door at distance %f", laser_height, dist);
  
  // gets a point cloud from the point_cloud_srv
  ROS_INFO("DetectHandleAction: get a point cloud from the door");
  point_cloud_assembler::BuildCloudAngle::Request req_pointcloud;
  point_cloud_assembler::BuildCloudAngle::Response res_pointcloud;
  req_pointcloud.angle_begin = -atan2(handle_top - laser_height, dist);
  req_pointcloud.angle_end = atan2(laser_height - handle_bottom, dist);
  req_pointcloud.duration = 4.0;
  if (!ros::service::call("point_cloud_srv/single_sweep_cloud", req_pointcloud, res_pointcloud)){
    ROS_ERROR("DetectHandleAction: failed to get point cloud for door detection");
    notifyAborted(door);
    return;
  }

  // detect handle
  ROS_INFO("DetectHandleAction: detect the hangle");
  door_handle_detector::DoorsDetectorCloud::Request  req_handledetect;
  door_handle_detector::DoorsDetectorCloud::Response res_handledetect;
  req_handledetect.door = door;
  req_handledetect.cloud = res_pointcloud.cloud;
  if (!ros::service::call("handle_detector_cloud", req_handledetect, res_pointcloud)){
    ROS_ERROR("DetectHandleAction: failed to detect a handle");
    notifyAborted(door);
    return;
  }

  notifySucceeded(res_handledetect.doors[0]);
}



void DetectHandleAction::handlePreempt()
{
  request_preempt_ = true;
};

