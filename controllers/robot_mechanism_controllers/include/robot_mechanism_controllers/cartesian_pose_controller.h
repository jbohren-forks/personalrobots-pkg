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

/*
 * Author: Wim Meeussen
 */

#ifndef CARTESIAN_POSE_CONTROLLER_H
#define CARTESIAN_POSE_CONTROLLER_H

#include <vector>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <ros/node.h>
#include <robot_msgs/PoseStamped.h>
#include <robot_msgs/Twist.h>
#include <mechanism_model/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include "robot_mechanism_controllers/cartesian_twist_controller.h"


namespace controller {

class CartesianPoseController
{
public:
  CartesianPoseController();
  ~CartesianPoseController();

  bool init(mechanism::RobotState *robot, const std::string& root_name, 
            const std::string& tip_name, const std::string& controller_name);
  bool starting();
  void update();

  // input of the controller
  KDL::Frame pose_desi_, pose_meas_;
  KDL::Twist twist_ff_;


private:
  KDL::Frame getPose();

  ros::Node* node_;
  std::string controller_name_;
  unsigned int  num_joints_, num_segments_;
  double last_time_;

  // robot structure
  mechanism::RobotState *robot_state_;       
  mechanism::Chain robot_;

  // pid controllers
  std::vector<control_toolbox::Pid> pid_controller_;     

  // kdl stuff for kinematics
  KDL::Chain             chain_;
  KDL::ChainFkSolverPos* jnt_to_pose_solver_;
  KDL::JntArray          jnt_pos_;

  // to get joint positions, velocities, and to set joint torques
  std::vector<mechanism::JointState*> joints_; 

  // internal twist controller
  CartesianTwistController twist_controller_;

  // reatltime publisher
  realtime_tools::RealtimePublisher<robot_msgs::Twist>* error_publisher_;
  unsigned int loop_count_;
};




class CartesianPoseControllerNode : public Controller
{
 public:
  CartesianPoseControllerNode();
  ~CartesianPoseControllerNode();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  bool starting();
  void update();
  void command(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr& pose_msg);


 private:
  void TransformToFrame(const tf::Transform& trans, KDL::Frame& frame);
  ros::Node* node_;
  std::string controller_name_;
  tf::TransformListener robot_state_;
  tf::MessageNotifier<robot_msgs::PoseStamped>* command_notifier_;
  std::string root_name_;

  CartesianPoseController controller_;

};

} // namespace


#endif
