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
 * Author: Wim Meeussen, Mrinal Kalakrishnan
 */

#ifndef CARTESIAN_POSE_TWIST_CONTROLLER_H
#define CARTESIAN_POSE_TWIST_CONTROLLER_H

#include <vector>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <ros/node.h>
#include <geometry_msgs/PoseStamped.h>
#include <experimental_controllers/PoseTwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <pr2_controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/scoped_ptr.hpp>
#include <robot_mechanism_controllers/cartesian_wrench_controller.h>
#include <control_toolbox/pid.h>
#include <kdl/chainfksolver.hpp>

namespace controller {

class CartesianPoseTwistController : public Controller
{
public:
  CartesianPoseTwistController();
  ~CartesianPoseTwistController();

  bool init(mechanism::RobotState *robot, const ros::NodeHandle &n);
  bool starting();
  void update();

  void command(const tf::MessageNotifier<experimental_controllers::PoseTwistStamped>::MessagePtr& pose_msg);

  // input of the controller
  KDL::Frame pose_desi_, pose_meas_;
  KDL::Twist twist_desi_, twist_meas_;
  KDL::Twist twist_ff_;

  // state output
  KDL::Twist twist_error_, pose_error_;

private:
  KDL::Frame getPose();

  void poseToFrame(const tf::Pose& pose, KDL::Frame& frame);
  void frameToPose(const KDL::Frame& frame, tf::Pose& pose);

  ros::NodeHandle node_;
  std::string controller_name_, root_name_;
  ros::Time last_time_;

  // output of the controller
  KDL::Wrench wrench_out_;

  // robot structure
  mechanism::RobotState *robot_state_;
  mechanism::Chain chain_;

  // pid controllers
  std::vector<control_toolbox::Pid> pid_controller_;

  // kdl stuff for kinematics
  KDL::Chain             kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  KDL::JntArray          jnt_pos_;
  boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
  KDL::JntArrayVel       jnt_posvel_;

  // reatltime publisher
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > state_error_publisher_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > state_pose_actual_publisher_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > state_pose_desired_publisher_;
  unsigned int loop_count_;

  tf::TransformListener tf_;
  boost::scoped_ptr<tf::MessageNotifier<experimental_controllers::PoseTwistStamped> > command_notifier_;
  ros::Subscriber sub_command_;

  // wrench controller
  CartesianWrenchController* wrench_controller_;

  double max_position_error_;
  double max_orientation_error_;
};

} // namespace


#endif
