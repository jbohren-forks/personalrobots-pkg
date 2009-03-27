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
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("detect_door_action"),
  tf_(node)
{
  cout << "DectectDoorAction: Constructor" << endl;
};


DetectDoorAction::~DetectDoorAction(){};



void DetectDoorAction::handleActivate(const robot_msgs::Door& door)
{
  request_preempt_ = false;
  notifyActivated();

  // check where robot is relative to door
    if (!tf_.canTransform("base_footprint", "laser_tilt_link", ros::Time(), ros::Duration().fromSec(5.0)))
      ROS_ERROR("TriggerDoorsScan: constructor failed");
    tf::Stamped<tf::Transform> tilt_stage;
    tf_.lookupTransform("base_footprint", "laser_tilt_link", ros::Time(), tilt_stage);
    double laser_height = tilt_stage.getOrigin()[2];
    tf::Stamped<tf::Vector3> doorpoint(tf::Vector3((door.door_p1.x+door.door_p2.x)/2.0,
                                                   (door.door_p1.y+door.door_p2.y)/2.0,
                                                   (door.door_p1.z+door.door_p2.z)/2.0),
                                       ros::Time(), door.header.frame_id);
    if (!tf_.canTransform("base_footprint", doorpoint.frame_id_, ros::Time(), ros::Duration().fromSec(5.0)))
      ROS_ERROR("TriggerDoorsScan: constructor failed");
    tf_.transformPoint("base_footprint", doorpoint, doorpoint);
    double dist = doorpoint[0];
    double door_height = 2.3;
  ROS_INFO("DectectDoorAction: tilt laser is at height %f, and door at distance %f", laser_height, dist);
  
  // gets a point cloud from the point_cloud_srv
  ROS_INFO("DectectDoorAction: get a point cloud from the door");
  point_cloud_assembler::BuildCloudAngle::Request req_pointcloud;
  point_cloud_assembler::BuildCloudAngle::Response res_pointcloud;
  req_pointcloud.angle_begin = -atan2(door_height - laser_height, dist);
  req_pointcloud.angle_end = atan2(laser_height, dist);
  req_pointcloud.duration = 10.0;
  if (!ros::service::call("point_cloud_srv/single_sweep_cloud", req_pointcloud, res_pointcloud))
    notifyAborted(door);

  // detect door
  ROS_INFO("DectectDoorAction: detect the door");
  door_handle_detector::DoorsDetectorCloud::Request  req_doordetect;
  door_handle_detector::DoorsDetectorCloud::Response res_doordetect;
  req_doordetect.door = door;
  req_doordetect.cloud = res_pointcloud.cloud;
  if (!ros::service::call("doors_detector_cloud", req_doordetect, res_doordetect))
    notifyAborted(door);
  notifySucceeded(res_doordetect.doors[0]);
}



void DetectDoorAction::handlePreempt()
{
  request_preempt_ = true;
};

