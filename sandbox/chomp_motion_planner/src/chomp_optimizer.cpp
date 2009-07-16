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
USING_PART_OF_NAMESPACE_EIGEN

namespace chomp
{

ChompOptimizer::ChompOptimizer(ChompTrajectory *trajectory, const ChompRobotModel *robot_model,
    const ChompRobotModel::ChompPlanningGroup *planning_group, const ChompParameters *parameters,
    const ros::Publisher& vis_marker_array_publisher, ChompCollisionSpace *collision_space):
      full_trajectory_(trajectory),
      robot_model_(robot_model),
      planning_group_(planning_group),
      parameters_(parameters),
      collision_space_(collision_space),
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
  derivative_costs[0] = 0.0;
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
  jacobian_ = Eigen::MatrixXd::Zero(3, num_joints_);

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
  kdlVecVecToEigenVecVec(collision_point_vel_, collision_point_vel_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_acc_, collision_point_acc_eigen_, 3, 1);

}

ChompOptimizer::~ChompOptimizer()
{
}

void ChompOptimizer::optimize()
{
  collision_space_->lock();

  ros::WallTime start_time = ros::WallTime::now();
  // iterate
  for (iteration_=0; iteration_<parameters_->getMaxIterations(); iteration_++)
  {
    //cout << "Iteration " << iteration_ << endl;
    calculateSmoothnessIncrements();
    performForwardKinematics();
    calculateCollisionIncrements();
    addIncrementsToTrajectory();
    handleJointLimits();
    updateFullTrajectory();
  }
  ROS_INFO("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());

  collision_space_->unlock();
  if (parameters_->getAnimatePath())
    animatePath();
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
  double potential;
  double vel_mag_sq;
  double vel_mag;
  Vector3d potential_gradient;
  Vector3d normalized_velocity;
  Matrix3d orthogonal_projector;
  Vector3d curvature_vector;
  Vector3d cartesian_gradient;

  collision_increments_.setZero(num_vars_free_, num_joints_);
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      bool colliding = getCollisionPointPotentialGradient(planning_group_->collision_points_[j],
          collision_point_pos_eigen_[i][j], potential, potential_gradient);
      if (potential <= 1e-10)
        continue;

      // all math from the CHOMP paper:

      vel_mag_sq = collision_point_vel_eigen_[i][j].squaredNorm();
      vel_mag = sqrt(vel_mag_sq);
      normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
      orthogonal_projector = Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
      curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
      cartesian_gradient = planning_group_->collision_points_[j].getVolume() *
          vel_mag*(orthogonal_projector*potential_gradient - potential*curvature_vector);

      // pass it through the jacobian transpose to get the increments
      planning_group_->collision_points_[j].getJacobian(joint_pos_eigen_[i], joint_axis_eigen_[i],
          collision_point_pos_eigen_[i][j], jacobian_);
      collision_increments_.row(i-free_vars_start_).transpose() -=
          jacobian_.transpose() * cartesian_gradient;

      if (colliding)
        break;
    }
  }
  //cout << collision_increments_ << endl;
}

void ChompOptimizer::addIncrementsToTrajectory()
{
  for (int i=0; i<num_joints_; i++)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(i) += parameters_->getLearningRate() *
        (
            joint_costs_[i].getQuadraticCostInverse() *
            (
                parameters_->getSmoothnessCostWeight() * smoothness_increments_.col(i) +
                parameters_->getObstacleCostWeight() * collision_increments_.col(i)
            )
        );
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

void ChompOptimizer::handleJointLimits()
{
  for (int joint=0; joint<num_joints_; joint++)
  {
    if (!planning_group_->chomp_joints_[joint].has_joint_limits_)
      continue;

    double joint_max = planning_group_->chomp_joints_[joint].joint_limit_max_;
    double joint_min = planning_group_->chomp_joints_[joint].joint_limit_min_;

    int count = 0;

    bool violation = false;
    do
    {
      double max_abs_violation =  1e-6;
      double max_violation = 0.0;
      int max_violation_index = 0;
      violation = false;
      for (int i=free_vars_start_; i<=free_vars_end_; i++)
      {
        double amount = 0.0;
        double absolute_amount = 0.0;
        if (group_trajectory_(i, joint) > joint_max)
        {
          amount = joint_max - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        else if (group_trajectory_(i, joint) < joint_min)
        {
          amount = joint_min - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        if (absolute_amount > max_abs_violation)
        {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;
          violation = true;
        }
      }

      if (violation)
      {
        int free_var_index = max_violation_index - free_vars_start_;
        double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,free_var_index);
        group_trajectory_.getFreeJointTrajectoryBlock(joint) +=
            multiplier * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
      }
      if (++count > 10)
        break;
    }
    while(violation);
  }
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

template<typename Derived, typename DerivedOther>
bool ChompOptimizer::getCollisionPointPotentialGradient(const ChompCollisionPoint& collision_point, const Eigen::MatrixBase<Derived>& collision_point_pos,
    double& potential, Eigen::MatrixBase<DerivedOther>& gradient) const
{
  Vector3d field_gradient;
  double field_distance = collision_space_->getDistanceGradient(
      collision_point_pos(0), collision_point_pos(1), collision_point_pos(2),
      field_gradient(0), field_gradient(1), field_gradient(2));

  double d = field_distance - collision_point.getRadius();

  // three cases below:
  if (d >= collision_point.getClearance())
  {
    potential = 0.0;
    gradient.setZero();
  }
  else if (d >= 0.0)
  {
    double diff = (d - collision_point.getClearance());
    double gradient_magnitude = diff * collision_point.getInvClearance(); // (diff / clearance)
    potential = 0.5*gradient_magnitude*diff;
    gradient = gradient_magnitude * field_gradient;
  }
  else // if d < 0.0
  {
    gradient = field_gradient;
    potential = -d + 0.5 * collision_point.getClearance();
  }

  return (field_distance <= collision_point.getRadius()); // true if point is in collision
}

void ChompOptimizer::animatePath()
{
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    visualizeState(i);
    ros::WallDuration(group_trajectory_.getDiscretization()).sleep();
  }
}

void ChompOptimizer::visualizeState(int index)
{
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(num_collision_points_);
  for (int i=0; i<num_collision_points_; i++)
  {
    msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
    msg.markers[i].header.stamp = ros::Time();
    msg.markers[i].ns = "chomp_collisions";
    msg.markers[i].id = i;
    msg.markers[i].type = visualization_msgs::Marker::SPHERE;
    msg.markers[i].action = visualization_msgs::Marker::ADD;
    msg.markers[i].pose.position.x = collision_point_pos_[index][i].x();
    msg.markers[i].pose.position.y = collision_point_pos_[index][i].y();
    msg.markers[i].pose.position.z = collision_point_pos_[index][i].z();
    msg.markers[i].pose.orientation.x = 0.0;
    msg.markers[i].pose.orientation.y = 0.0;
    msg.markers[i].pose.orientation.z = 0.0;
    msg.markers[i].pose.orientation.w = 1.0;
    double scale = planning_group_->collision_points_[i].getRadius()*2;
    msg.markers[i].scale.x = scale;
    msg.markers[i].scale.y = scale;
    msg.markers[i].scale.z = scale;
    msg.markers[i].color.a = 0.9;
    msg.markers[i].color.r = 0.5;
    msg.markers[i].color.g = 1.0;
    msg.markers[i].color.b = 0.3;
  }
  vis_pub_.publish(msg);
}

} // namespace chomp
