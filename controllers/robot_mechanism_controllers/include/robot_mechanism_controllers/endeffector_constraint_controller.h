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
 * Author: John Hsu and Melonee Wise
 */

#ifndef ENDEFFECTOR_CONSTRAINT_CONTEROLLER_H
#define ENDEFFECTOR_CONSTRAINT_CONTEROLLER_H

#include <vector>
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "ros/node.h"
#include "robot_msgs/Wrench.h"
#include "misc_utils/subscription_guard.h"
#include "mechanism_model/controller.h"
#include "tf/transform_datatypes.h"
#include "misc_utils/advertised_service_guard.h"
#include "joy/Joy.h"
#include "Eigen/LU"
#include "Eigen/Core"
#include "robot_kinematics/robot_kinematics.h"
#include <std_msgs/VisualizationMarker.h>


namespace controller {

class EndeffectorConstraintController : public Controller
{
public:
  EndeffectorConstraintController();
  ~EndeffectorConstraintController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();
  void computeConstraintJacobian();
  // input of the controller
  KDL::Wrench wrench_desi_;

private:
  unsigned int  num_joints_, num_segments_;

  // kdl stuff for kinematics
  KDL::Chain                 chain_;
  KDL::ChainJntToJacSolver*  jnt_to_jac_solver_;
  KDL::ChainFkSolverPos* jnt_to_pose_solver_;

  // to get joint positions, velocities, and to set joint torques
  std::vector<mechanism::JointState*> joints_; 
  Eigen::Matrix<float,6,2> constraint_jac_;
  Eigen::Matrix<float,6,1> constraint_wrench_;
  Eigen::Matrix<float,2,1> constraint_force_;
  KDL::Frame endeffector_frame_;

  // some parameters to define the constraint
  double wall_x;
  double threshold_x;
  double wall_r;
  double threshold_r;
  double f_x_max;
  double f_r_max;


};






class EndeffectorConstraintControllerNode : public Controller
{
 public:
  EndeffectorConstraintControllerNode();
  ~EndeffectorConstraintControllerNode();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();
  void command();

 private:
  std::string topic_;
  ros::node *node_;     
  EndeffectorConstraintController controller_;
  SubscriptionGuard guard_command_;

  robot_msgs::Wrench wrench_msg_;
};

} // namespace


#endif
