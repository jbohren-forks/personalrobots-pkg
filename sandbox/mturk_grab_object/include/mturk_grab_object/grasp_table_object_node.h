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

//! \author Alex Sorokin 
#ifndef PCD_NOVELTY_NOVELTY_ESTIMATOR_NODE_H
#define PCD_NOVELTY_NOVELTY_ESTIMATOR_NODE_H

// ROS core
#include <ros/ros.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


#include <map>

#include <tabletop_srvs/FindTable.h>
#include <tabletop_msgs/Table.h>
#include <tabletop_msgs/ObjectOnTable.h>

#include <robot_actions/action_client.h>
#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>

namespace mturk_grab_object
{

class GraspTask
{
 public:
  robot_msgs::PointCloudConstPtr cloud_;
  sensor_msgs::CameraInfoConstPtr cam_info_;
  sensor_msgs::ImageConstPtr image_;

  tabletop_msgs::TableConstPtr table_;
  tabletop_msgs::ObjectOnTable active_object_;

 public:
  int num_attempted;
  int num_success;

 public:
  enum GraspTaskState{ CREATED, ACTIVE, SUCCESS, ABORTED };
  int state;
  bool isActive(){
    return state==ACTIVE;
  }
};



class GraspObjectNode
{
protected:

  ros::NodeHandle n_;
  

  std::map<int,boost::shared_ptr<GraspTask> > grab_tasks;

  boost::shared_ptr<GraspTask> active_task;
  robot_msgs::Pose default_arm_pose;
  
  ros::ServiceClient detect_object_client_;

  int num_skip_images_;
  int num_already_skipped_images_;


  ros::Subscriber image_sub_;

  ros::Publisher  marker_pub_;
  
public:

  robot_actions::ActionClient<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t> move_arm;

public:
  GraspObjectNode();

  void setup();

  void imageCallback(const sensor_msgs::ImageConstPtr& the_image);

  void computeGraspPose(const tabletop_msgs::ObjectOnTable& object_on_table, robot_msgs::Pose& output_pose);

  int num_attempts;
  int num_success;

  bool is_test_;
};



}


#endif
