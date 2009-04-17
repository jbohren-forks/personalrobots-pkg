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
#include <boost/scoped_ptr.hpp>
#include "robot_mechanism_controllers/cartesian_twist_controller.h"


namespace controller {

class CartesianPoseController : public Controller
{
public:
  CartesianPoseController();
  ~CartesianPoseController();

  bool initXml(mechanism::RobotState *robot_state, TiXmlElement *config);

  bool starting();
  void update();

  void command(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr& pose_msg);

  // input of the controller
  KDL::Frame pose_desi_, pose_meas_;
  KDL::Twist twist_ff_;

private:
  KDL::Frame getPose();
  void TransformToFrame(const tf::Transform& trans, KDL::Frame& frame);

  ros::Node* node_;
  std::string controller_name_, root_name_;
  double last_time_;

  // robot structure
  mechanism::RobotState *robot_state_;       
  mechanism::Chain chain_;

  // pid controllers
  std::vector<control_toolbox::Pid> pid_controller_;     

  // kdl stuff for kinematics
  KDL::Chain             kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  KDL::JntArray          jnt_pos_;

  // reatltime publisher
  boost::scoped_ptr<realtime_tools::RealtimePublisher<robot_msgs::Twist> > state_publisher_;
  unsigned int loop_count_;

  tf::TransformListener tf_;
  boost::scoped_ptr<tf::MessageNotifier<robot_msgs::PoseStamped> > command_notifier_;

  // twist controller
  CartesianTwistController* twist_controller_;
};

} // namespace


#endif
