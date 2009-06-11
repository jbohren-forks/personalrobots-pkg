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

/**
 * \class controller::JointChainSineController
 * \brief Runs sinusoids through all joints in a kinematic chain using a PID controller per joint
 *
 * This is useful as a "random" trajectory generator to collect data for
 * dynamics parameter estimation. It also allows the definition of cartesian
 * cuboids relative to the root that are a "no-go" zone for the tip, as a
 * rudimentary way to avoid self-collisions.
 *
 * Example configuration:<br>
 *
 * <controller type="JointChainSineController" name="controller_name"><br>
 *   <chain root="torso_lift_link" tip="l_gripper_tool_frame" min_freq="0.4" max_freq="1.0" freq_randomness="0.01"/><br>
 *   <cartesian_constraint min="-10 -10 -10 max="10 10 -0.0" /><br>
 *   <cartesian_constraint min="-5 -2 -3" max="3 4 5" /><br>
 *   <joint name="l_forearm_roll_link"><br>
 *     <pid p="1.0" i="0.0" d="3.0" iClamp="0.0" /><br>
 *   </joint><br>
 * </controller><br>
 */

#ifndef JOINT_CHAIN_SINE_CONTROLLER_H_
#define JOINT_CHAIN_SINE_CONTROLLER_H_

#include <mechanism_model/controller.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <mechanism_model/chain.h>
#include <string>
#include <vector>
#include <control_toolbox/sinusoid.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>

namespace controller
{

class JointChainSineController: public controller::Controller
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  JointChainSineController();
  virtual ~JointChainSineController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  /**
   * \brief Issues commands to the joints. Should be called at regular intervals
   */
  virtual void update();

private:
  mechanism::Chain mechanism_chain_;            /**< Kinematic chain */
  mechanism::RobotState *robot_state_;          /**< Pointer to robot structure. */
  double last_time_;                            /**< Last time stamp of update. */
  bool initialized_;                            /**< Flag which indicates whether this class has been initialized */
  KDL::Chain kdl_chain_;                        /**< KDL chain */
  KDL::JntArrayVel jnt_pos_vel_;                /**< Joint positions and velocities */
  KDL::JntArrayVel jnt_des_pos_vel_;            /**< Joint desired positions and velocities */
  KDL::JntArray jnt_eff_;                       /**< Joint efforts */
  KDL::JntArray jnt_prev_des_pos_;              /**< Previous desired joint positions */
  std::string controller_name_;                 /**< Controller name */
  int num_joints_;                              /**< Number of joints in the chain */
  std::vector<std::vector<control_toolbox::Sinusoid> > sinusoids_;      /**< Vector of sinusoids for each joint */
  std::vector<control_toolbox::Pid> pid_controllers_;                   /**< A PID controller for each joint */
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;         /**< Position forward kinematics solver */
  std::vector<Eigen::Matrix<double, 2, 3> > cart_constraints_;          /**< Cartesian box constraints */
  int count_;                                   /**< Tick counter */

  /**
   * \brief Initializes the sinusoids
   */
  void initSinusoids(double min_freq, double max_freq, double freq_randomness);

  /**
   * \brief Performs kinematic constraint checks
   * \return true if a constraint is violated, false otherwise
   */
  bool violatesConstraints();

  /**
   * \brief Add a cartesian constraint
   */
  void addCartesianConstraint(TiXmlElement *constraint);

};

}

#endif /* JOINT_CHAIN_SINE_CONTROLLER_H_ */
