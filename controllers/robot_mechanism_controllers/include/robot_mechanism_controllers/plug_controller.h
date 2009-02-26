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
 * Author: Melonee Wise
 */

#ifndef PLUG_CONTROLLER_H
#define PLUG_CONTROLLER_H

#include <vector>
#include "boost/scoped_ptr.hpp"
#include "mechanism_model/chain.h"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "ros/node.h"
#include "robot_msgs/Wrench.h"
#include "robot_msgs/PoseStamped.h"
#include "robot_msgs/Transform.h"
#include "robot_srvs/SetPoseStamped.h"

#include "misc_utils/subscription_guard.h"
#include "mechanism_model/controller.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "misc_utils/advertised_service_guard.h"

#include <realtime_tools/realtime_publisher.h>


#include "Eigen/Geometry"
#include "Eigen/LU"
#include "Eigen/Core"
#include "robot_kinematics/robot_kinematics.h"

#include <robot_msgs/VisualizationMarker.h>


namespace controller {

class PlugController : public Controller
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PlugController();
  ~PlugController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();
  void computeConstraintJacobian();
  void computeConstraintNullSpace();

  void setToolOffset(const tf::Transform &);


  const char *root_name_;
  // input of the controller
  KDL::Wrench wrench_desi_;
  Eigen::Matrix<float,6,1> task_wrench_;
  Eigen::Vector3f outlet_pt_;
  Eigen::Vector3f outlet_norm_; // this must be normalized

  KDL::Frame endeffector_frame_;
  KDL::Frame desired_frame_;

  mechanism::Chain chain_;

private:

  mechanism::RobotState *robot_;

  KDL::Chain kdl_chain_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;

  // to get joint positions, velocities, and to set joint torques
  Eigen::Matrix<float,6,5> constraint_jac_;
  Eigen::Matrix<float,6,1> constraint_wrench_;
  Eigen::Matrix<float,5,1> constraint_force_;
  // joint constraint
  Eigen::MatrixXf joint_constraint_force_;
  Eigen::MatrixXf joint_constraint_jac_;
  Eigen::MatrixXf joint_constraint_null_space_;

  Eigen::MatrixXf task_jac_;
  Eigen::MatrixXf identity_;
  Eigen::MatrixXf identity_joint_;
  Eigen::MatrixXf constraint_null_space_;
  Eigen::MatrixXf constraint_torq_;
  Eigen::MatrixXf joint_constraint_torq_;
  Eigen::MatrixXf task_torq_;

  // some parameters to define the constraint

  double upper_arm_limit;
  double upper_arm_dead_zone;
  double f_r_max;
  double f_pose_max;
  double f_limit_max;

  bool initialized_;
};


class PlugControllerNode : public Controller
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PlugControllerNode();
  ~PlugControllerNode();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();
  void command();
  void outletPose();

  bool setToolFrame(robot_srvs::SetPoseStamped::Request &req,
                    robot_srvs::SetPoseStamped::Response &resp);

 private:
  std::string topic_;
  ros::Node *node_;
  PlugController controller_;
  SubscriptionGuard guard_command_;
  SubscriptionGuard guard_outlet_pose_;
  AdvertisedServiceGuard guard_set_tool_frame_;

  robot_msgs::Wrench wrench_msg_;
  robot_msgs::PoseStamped outlet_pose_msg_;
  unsigned int loop_count_;

  tf::TransformListener TF;                    /**< The transform for converting from point to head and tilt frames. */
  realtime_tools::RealtimePublisher <robot_msgs::Transform>* current_frame_publisher_;


};

} // namespace


#endif
