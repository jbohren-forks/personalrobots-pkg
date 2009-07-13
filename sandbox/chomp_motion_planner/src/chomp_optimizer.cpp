/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <chomp_motion_planner/chomp_optimizer.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <chomp_motion_planner/chomp_utils.h>

using namespace std;

namespace chomp
{

ChompOptimizer::ChompOptimizer(ChompTrajectory *trajectory, const ChompRobotModel *robot_model,
    const ChompRobotModel::ChompPlanningGroup *planning_group, const ChompParameters *parameters,
    const ros::Publisher& vis_marker_array_publisher):
      full_trajectory_(trajectory),
      robot_model_(robot_model),
      planning_group_(planning_group),
      parameters_(parameters),
      group_trajectory_(*full_trajectory_, planning_group_, ChompCost::DIFF_RULE_LENGTH),
      kdl_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
      vis_pub_(vis_marker_array_publisher)
{
  initialize();
}

void ChompOptimizer::initialize()
{

  // init some variables:
  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();

  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

  num_collision_points_ = planning_group_->collision_points_.size();

  // set up the joint costs:
  joint_costs_.reserve(num_joints_);

  // @TODO hardcoded derivative costs:
  std::vector<double> derivative_costs(3);
  derivative_costs[0] = 1.0;
  derivative_costs[1] = 1.0;
  derivative_costs[2] = 1.0;

  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_.push_back(ChompCost(group_trajectory_, i, derivative_costs));
  }

  // allocate memory for matrices:
  smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  collision_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);

  joint_axis_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  joint_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  segment_frames_.resize(num_vars_all_, std::vector<KDL::Frame>(robot_model_->getKDLTree()->getNrOfSegments()));
  collision_point_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
  collision_point_vel_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
  collision_point_acc_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));

  // create the eigen maps:
  kdlVecVecToEigenVecVec(joint_axis_, joint_axis_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(joint_pos_, joint_pos_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_pos_, collision_point_pos_eigen_, 3, 1);

}

ChompOptimizer::~ChompOptimizer()
{
}

void ChompOptimizer::optimize()
{
  // iterate
  for (iteration_=0; iteration_<parameters_->getMaxIterations(); iteration_++)
  {
    //cout << "Iteration " << iteration_ << endl;
    calculateSmoothnessIncrements();
    performForwardKinematics();
    calculateCollisionIncrements();
    addIncrementsToTrajectory();
    //debugCost();
    updateFullTrajectory();
  }

}

void ChompOptimizer::calculateSmoothnessIncrements()
{
  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i), smoothness_derivative_);
    smoothness_increments_.col(i) = -smoothness_derivative_.segment(
        group_trajectory_.getStartIndex(), num_vars_free_);
  }
}

void ChompOptimizer::calculateCollisionIncrements()
{
  collision_increments_.setZero(num_vars_free_, num_joints_);
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {

    }
  }
}

void ChompOptimizer::addIncrementsToTrajectory()
{
  for (int i=0; i<num_joints_; i++)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(i) += parameters_->getSmoothnessCostWeight() *
        (joint_costs_[i].getQuadraticCostInverse() * smoothness_increments_.col(i));
  }
}

void ChompOptimizer::updateFullTrajectory()
{
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void ChompOptimizer::debugCost()
{
  double cost = 0.0;
  for (int i=0; i<num_joints_; i++)
    cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  cout << "Cost = " << cost << endl;
}

void ChompOptimizer::performForwardKinematics()
{
  double invTime = 1.0 / group_trajectory_.getDiscretization();
  double invTimeSq = invTime*invTime;

  // calculate the forward kinematics for the fixed states only in the first iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_==0)
  {
    start = 0;
    end = num_vars_all_-1;
  }

  // for each point in the trajectory
  for (int i=start; i<=end; ++i)
  {
    int full_traj_index = group_trajectory_.getFullTrajectoryIndex(i);
    full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);
    robot_model_->getForwardKinematicsSolver()->JntToCart(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);

    // calculate the position of every collision point:
    for (int j=0; j<num_collision_points_; j++)
    {
      int segment_number = planning_group_->collision_points_[j].getSegmentNumber();
      collision_point_pos_[i][j] = segment_frames_[i][segment_number] * planning_group_->collision_points_[j].getPosition();
    }
  }

  // now, get the vel and acc for each collision point (using finite differencing)
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      SetToZero(collision_point_vel_[i][j]);
      SetToZero(collision_point_acc_[i][j]);
      for (int k=-ChompCost::DIFF_RULE_LENGTH/2; k<=ChompCost::DIFF_RULE_LENGTH/2; k++)
      {
        collision_point_vel_[i][j] += (invTime * ChompCost::DIFF_RULES[0][k+ChompCost::DIFF_RULE_LENGTH/2]) *
            collision_point_pos_[i+k][j];
        collision_point_acc_[i][j] += (invTimeSq * ChompCost::DIFF_RULES[1][k+ChompCost::DIFF_RULE_LENGTH/2]) *
            collision_point_pos_[i+k][j];
      }
    }
  }

  if (iteration_==0 && false)
  {
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(num_collision_points_);
    for (int i=0; i<num_collision_points_; i++)
    {
      msg.markers[i].header.frame_id = "base_link";
      msg.markers[i].header.stamp = ros::Time();
      msg.markers[i].ns = "chomp_collisions";
      msg.markers[i].id = i;
      msg.markers[i].type = visualization_msgs::Marker::SPHERE;
      msg.markers[i].action = visualization_msgs::Marker::ADD;
      msg.markers[i].pose.position.x = collision_point_pos_[0][i].x();
      msg.markers[i].pose.position.y = collision_point_pos_[0][i].y();
      msg.markers[i].pose.position.z = collision_point_pos_[0][i].z();
      msg.markers[i].pose.orientation.x = 0.0;
      msg.markers[i].pose.orientation.y = 0.0;
      msg.markers[i].pose.orientation.z = 0.0;
      msg.markers[i].pose.orientation.w = 1.0;
      double scale = planning_group_->collision_points_[i].getRadius()*2;
      msg.markers[i].scale.x = scale;
      msg.markers[i].scale.y = scale;
      msg.markers[i].scale.z = scale;
      msg.markers[i].color.a = 1.0;
      msg.markers[i].color.r = 0.5;
      msg.markers[i].color.g = 1.0;
      msg.markers[i].color.b = 0.3;
    }
    vis_pub_.publish(msg);
  }

}

void ChompOptimizer::eigenMapTest()
{
  double foo_eigen;
  double foo_kdl;

  cout << "Eigen location: " << &(joint_axis_eigen_[free_vars_start_][0](0)) <<
          "  KDL location: " << &(joint_axis_[free_vars_start_][0](0)) << endl;

  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);

  joint_axis_eigen_[free_vars_start_][0](0) = 1.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_kdl == foo_eigen);

  joint_axis_[free_vars_start_][0](0) = 2.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen == foo_kdl);

  foo_eigen = joint_pos_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_pos_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);

  foo_eigen = collision_point_pos_eigen_[free_vars_start_][5](0);
  foo_kdl = collision_point_pos_[free_vars_start_][5](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);
}

} // namespace chomp
