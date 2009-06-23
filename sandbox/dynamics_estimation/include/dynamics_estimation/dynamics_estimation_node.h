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

#ifndef DYNAMICS_ESTIMATION_NODE_H_
#define DYNAMICS_ESTIMATION_NODE_H_

#include <ros/ros.h>
#include <mechanism_msgs/MechanismState.h>
#include <vector>
#include <dynamics_estimation/trajectory_point.h>
#include <mechanism_model/robot.h>
#include <mechanism_model/chain.h>
#include <hardware_interface/hardware_interface.h>
#include <kdl/chain.hpp>
#include <Eigen/Core>

namespace dynamics_estimation
{

class DynamicsEstimationNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DynamicsEstimationNode();
  virtual ~DynamicsEstimationNode();

  /**
   * Initializes the dynamics estimation node
   *
   * \return true if successful, false if not
   */
  bool init();

  /**
   * Runs the dynamics estimation node
   *
   * \return Exit code, 0 if successful
   */
  int run();

  /**
   * Callback for mechanism state messages
   */
  void mechanismStateCallback(const boost::shared_ptr<mechanism_msgs::MechanismState const>& mechanism_state);

  /**
   * Callback for mechanism state messages loaded from a bag file
   */
  void mechanismStatePlayerCallback(std::string name, mechanism_msgs::MechanismState* m, ros::Time play_time, ros::Time record_time, void* n);

private:
  ros::NodeHandle node_handle_;                         /**< ROS Node Handle */
  ros::Subscriber mechanism_state_sub_;                 /**< Subscriber to /mechanism_state */

  int max_samples_;                                     /**< Maximum number of trajectory samples that we will process */

  std::vector<TrajectoryPoint> trajectory_points_;      /**< Storage for the trajectory data */
  int num_trajectory_points_;                           /**< Number of trajectory points stored */
  std::string chain_root_;                              /**< Root name of the kinematic chain */
  std::string chain_tip_;                               /**< Tip name of the kinematic chain */
  std::string robot_param_;                             /**< Name on the parameter server where the robot description is stored */
  std::string bag_file_;                                /**< Bag file to load trajectory data from */
  bool use_bag_file_;                                   /**< Should I load data from a bag file or listen to mechanism_state? */

  HardwareInterface hardware_interface_;                /**< Hardware interface, for creating the mechanism_chain_ */
  mechanism::Robot robot_;                              /**< Robot description */
  mechanism::Chain mechanism_chain_;                    /**< Mechanism chain */
  KDL::Chain kdl_chain_;                                /**< KDL chain */
  int num_joints_;                                      /**< Number of actuated joints */
  std::map<std::string, int> joint_name_to_index_;      /**< Map of joint names to indices */

  bool finished_data_collection_;                       /**< Flag indicating if we're done with data collection */

  Eigen::MatrixXd Ktrans_K_;
  Eigen::VectorXd Ktrans_tau_;
  Eigen::MatrixXd K_;
  Eigen::VectorXd tau_;
  Eigen::VectorXd psi_;

  static const int diff_rule_points_ = 5;
  /**
   * Runs all the data processing steps
   */
  void runProcessing();

  /**
   * Performs filtering and differentiation to get joint velocities and accelerations from positions
   */
  void filterTrajectories();

  /**
   * Performs forward kinematics on each trajectory point to get positions, velocities, accelerations
   * of each link
   */
  void performForwardKinematics();

  /**
   * Marks trajectory points that are close to joint limits as invalid
   */
  void removeJointLimitPoints();

  /**
   * Set up the regression matrices and adds all the data to them
   */
  void processDataForRegression();

  /**
   * Performs the regression to get the parameters
   */
  void runRegression();

  /**
   * Loads data from the bag file
   *
   * \return true if successful, false if not
   */
  bool loadDataFromBagFile();

};

}

#endif /* DYNAMICS_ESTIMATION_NODE_H_ */
