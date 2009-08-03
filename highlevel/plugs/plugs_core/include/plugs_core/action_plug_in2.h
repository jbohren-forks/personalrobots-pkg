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


#ifndef ACTION_SERVO_TO_OUTLET_H
#define ACTION_SERVO_TO_OUTLET_H

// ROS Stuff
#include <ros/node.h>

// Msgs
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include "geometry_msgs/PoseStamped.h"
#include "manipulation_msgs/TaskFrameFormalism.h"
#include "plugs_core/PlugInState.h"
#include "robot_mechanism_controllers/CartesianHybridState.h"

//TF
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include "boost/scoped_ptr.hpp"

// Robot Action Stuff
#include <robot_actions/action.h>


namespace plugs_core{

class PlugInAction: public robot_actions::Action<std_msgs::Int32, std_msgs::Empty>
{
public:
  PlugInAction(ros::Node& node);
  ~PlugInAction();

  robot_actions::ResultStatus execute(const std_msgs::Int32& outlet_id, std_msgs::Empty& feedback);

private:

  void reset();

  boost::scoped_ptr<tf::MessageNotifier<geometry_msgs::PoseStamped> > notifier_;
  void plugMeasurementCB(const tf::MessageNotifier<geometry_msgs::PoseStamped>::MessagePtr &msg);

  robot_mechanism_controllers::CartesianHybridState c_state_msg_;
  void controllerStateCB();
  /*
  void measure();
  void move();
  void hold();
  void force();
  void insert();
  */

  std::string action_name_;

  ros::Node& node_;

  std::string arm_controller_;

  std_msgs::Empty empty_;

  boost::scoped_ptr<tf::TransformListener> TF_;

  // Data from the vision estimate of plug pose
  boost::mutex from_viz_lock_;
  tf::Stamped<tf::Transform> viz_offset_from_viz_;
  tf::Stamped<tf::Transform> mech_offset_from_viz_;
  bool updated_from_viz_;

  // Data from the controller state
  boost::mutex from_c_lock_;
  tf::Stamped<tf::Transform> pose_from_mech_;


  // TODO: mutex
  ros::Time vision_estimate_time_;
  tf::Pose outlet_pose_mech_;  // Outlet pose in the mechanism "frame"





  double last_standoff_;
  ros::Time g_started_inserting_, g_started_forcing_, g_stopped_forcing_;
  int g_state_;
  int prev_state_;
  tf::Pose prev_viz_offset_;

  tf::Stamped<tf::Transform> mech_offset_;
  tf::Pose mech_offset_desi_;
  manipulation_msgs::TaskFrameFormalism tff_msg_;
};

}
#endif
