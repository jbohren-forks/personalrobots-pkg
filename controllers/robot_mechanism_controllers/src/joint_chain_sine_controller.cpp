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

#include <iostream>
#include <cmath>
#include <cstdio>
#include <sstream>

#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <robot_mechanism_controllers/joint_chain_sine_controller.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

using namespace std;

namespace controller
{

ROS_REGISTER_CONTROLLER(JointChainSineController)

JointChainSineController::JointChainSineController():
  robot_state_(NULL),
  initialized_(false),
  count_(0)
{
}

JointChainSineController::~JointChainSineController()
{
}

bool JointChainSineController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{

  // ensure that we got the robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // initialize the time
  last_time_ = robot_state_->hw_->current_time_;

  // get the controller name
  controller_name_ = config->Attribute("name");

  // ensure that the chain element exists in the config
  TiXmlElement *chain_xml = config->FirstChildElement("chain");
  if (!chain_xml)
  {
    ROS_ERROR("JointChainSineController: chain element not found, failed to initialize");
    return false;
  }

  // get the name of root and tip
  std::string root_name, tip_name;
  const char *root_name_c = chain_xml->Attribute("root");
  const char *tip_name_c = chain_xml->Attribute("tip");
  if (!root_name_c || !tip_name_c)
  {
    ROS_ERROR("JointChainSineController: root or tip attribute not found, failed to initialize");
    return false;
  }
  root_name = std::string(root_name_c);
  tip_name = std::string(tip_name_c);

  // construct the chain
  if (!mechanism_chain_.init(robot_state_->model_, root_name, tip_name))
  {
    ROS_ERROR("JointChainSineController: Could not construct chain\n");
    return false;
  }
  mechanism_chain_.toKDL(kdl_chain_);

  // get number of joints in chain
  num_joints_ = kdl_chain_.getNrOfJoints();

  // allocate memory:
  jnt_pos_vel_.resize(num_joints_);
  jnt_des_pos_vel_.resize(num_joints_);
  jnt_eff_.resize(num_joints_);
  jnt_prev_des_pos_.resize(num_joints_);
  pid_controllers_.resize(num_joints_, control_toolbox::Pid());
  sinusoids_.resize(num_joints_, std::vector<control_toolbox::Sinusoid>());

  // create the FK solver:
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  // get the default parameters for the sinusoids:
  const char *min_freq_c = chain_xml->Attribute("min_freq");
  const char *max_freq_c = chain_xml->Attribute("max_freq");
  const char *freq_randomness_c = chain_xml->Attribute("freq_randomness");
  double min_freq = min_freq_c ? atof(min_freq_c) : 0.4;
  double max_freq = max_freq_c ? atof(max_freq_c) : 1.0;
  double freq_randomness = freq_randomness_c ? atof(freq_randomness_c) : 0.01;

  // initialize the sinusoids:
  initSinusoids(min_freq, max_freq, freq_randomness);

  // initialize the pid controllers for each joint
  TiXmlElement *joint_element = config->FirstChildElement("joint");
  for (; joint_element; joint_element = joint_element->NextSiblingElement("joint"))
  {
    const char *joint_name_c = joint_element->Attribute("name");
    if (!joint_name_c)
      continue;
    // iterate through our list of joints to check for a name match
    // this is slow but doesn't matter since this is just the initialization
    int matchIndex=-1;
    for (int i=0; i<num_joints_; i++)
    {
      int real_joint_index = mechanism_chain_.joint_indices_[i];
      mechanism::Joint *joint = robot_state_->joint_states_[real_joint_index].joint_;
      if (joint->name_.compare(joint_name_c)==0)
      {
        matchIndex = i;
        break;
      }
    }
    if (matchIndex==-1)
    {
      ROS_WARN("JointChainSineController: joint not found in chain: %s", joint_name_c);
      continue;
    }
    TiXmlElement *pid_element = joint_element->FirstChildElement("pid");
    if (pid_element)
    {
      pid_controllers_[matchIndex].initXml(pid_element);
    }
    else
    {
      ROS_WARN("JointChainSineController: pid settings not found for joint %s", joint_name_c);
    }
  }

  // parse the constraints:
  cart_constraints_.clear();
  TiXmlElement *constraint_element = config->FirstChildElement("cartesian_constraint");
  for (; constraint_element; constraint_element = constraint_element->NextSiblingElement("cartesian_constraint"))
  {
    addCartesianConstraint(constraint_element);
  }

  return true;
}

void JointChainSineController::initSinusoids(double min_freq, double max_freq, double freq_randomness)
{
  // create the random number generator:
  boost::mt19937 rng;
  boost::normal_distribution<> normal_dist(0,freq_randomness);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
    gaussian(rng, normal_dist);

  // divide the frequencies evenly in logarithm space:
  double log_min_freq = log(min_freq);
  double log_max_freq = log(max_freq);
  double log_freq_add = (log_max_freq - log_min_freq) / (num_joints_-1);

  for (int i=0; i<num_joints_; i++)
  {
    double freq = exp(log_min_freq + log_freq_add*i) + gaussian();
    int real_joint_index = mechanism_chain_.joint_indices_[i];
    mechanism::Joint *joint = robot_state_->joint_states_[real_joint_index].joint_;
    //cout << joint->name_ << endl;
    if (joint->type_ == mechanism::JOINT_CONTINUOUS)
    {
      // add a slow high amplitude sine:
      sinusoids_[i].push_back(control_toolbox::Sinusoid(0.0, 2.0*M_PI, min_freq/5.0 + fabs(gaussian()), 0.0));
      // and a regular lower amplitude sine:
      sinusoids_[i].push_back(control_toolbox::Sinusoid(0.0, 0.5*M_PI, freq, 0.0));
    }
    else
    {
      // get the joint range and center
      double joint_range = joint->joint_limit_max_ - joint->joint_limit_min_;
      double joint_center = joint->joint_limit_min_ + joint_range/2.0;

      // let peak to peak amplitude be 80% of the total joint span:
      double amplitude = (joint_range*0.80) / 2.0;
      sinusoids_[i].push_back(control_toolbox::Sinusoid(joint_center, amplitude, freq, 0.0));
    }
    //sinusoids_[i][0].debug();
  }
}

void JointChainSineController::update()
{
  // check if joints are calibrated
  if (!mechanism_chain_.allCalibrated(robot_state_->joint_states_))
    return;

  // get joint positions and velocities
  mechanism_chain_.getVelocities(robot_state_->joint_states_, jnt_pos_vel_);

  if (!initialized_)
  {
    initialized_ = true;
    jnt_prev_des_pos_ = jnt_pos_vel_.q;
  }

  // update the time
  double time = robot_state_->hw_->current_time_;
  double dt = time - last_time_;

  // calculate the desired positions and velocities for all joints:
  for (int i=0; i<num_joints_; i++)
  {
    jnt_des_pos_vel_.q(i) = 0.0;
    jnt_des_pos_vel_.qdot(i) = 0.0;
    // iterate through the sinusoids:
    for (unsigned int j=0; j<sinusoids_[i].size(); j++)
    {
      double tq, tqd, tqdd;
      tq = sinusoids_[i][j].update(time, tqd, tqdd);
      jnt_des_pos_vel_.q(i) += tq;
      jnt_des_pos_vel_.qdot(i) += tqd;
    }
  }

  // perform kinematic constraint checks:
  if (violatesConstraints())
  {
    jnt_des_pos_vel_.q = jnt_prev_des_pos_;
    for (int i=0; i<num_joints_; i++)
      jnt_des_pos_vel_.qdot(i) = 0.0;
    if (count_%200 == 0)
      printf("Violating constraints!\n");
  }
  else
  {
    jnt_prev_des_pos_ = jnt_pos_vel_.q;
    if (count_%200 == 0)
      printf("Not violating constraints!\n");
  }

  // set the efforts for the joints using pid:
  for (int i=0; i<num_joints_; i++)
  {
    int real_joint_index = mechanism_chain_.joint_indices_[i];
    mechanism::JointState *joint_state = &robot_state_->joint_states_[real_joint_index];

    // get the errors:
    double error = jnt_pos_vel_.q(i) - jnt_des_pos_vel_.q(i);
    double errord = (jnt_pos_vel_.qdot(i) - jnt_des_pos_vel_.qdot(i));

    // correct the position error if it's a continuous joint with wrap-arounds...
    if(joint_state->joint_->type_ == mechanism::JOINT_CONTINUOUS)
    {
      while (error < -M_PI)
      {
        error = error + 2.0*M_PI;
      }
      while (error > M_PI)
      {
        error = error - 2.0*M_PI;
      }
    }

    jnt_eff_(i) = pid_controllers_[i].updatePid(error, errord, dt);

  //  cout << error << " " << jnt_eff_(i) << " ";
  }
  //cout << endl;

  mechanism_chain_.setEfforts(jnt_eff_, robot_state_->joint_states_);

  last_time_ = time;
  ++count_;
}

bool JointChainSineController::violatesConstraints()
{
  KDL::Frame pose;
  KDL::Frame cur_pose;
  KDL::Frame test_pose;
  jnt_to_pose_solver_->JntToCart(jnt_des_pos_vel_.q, pose);
  jnt_to_pose_solver_->JntToCart(jnt_pos_vel_.q, cur_pose);

  KDL::JntArray test_q;
  test_q.resize(num_joints_);
  for (int i=0; i<num_joints_; i++)
    test_q(i) = 0.0;
  jnt_to_pose_solver_->JntToCart(test_q, test_pose);


  if (count_%200 == 0)
  {
    printf("Desired = %f %f %f\n", pose.p[0], pose.p[1], pose.p[2]);
    printf("Current = %f %f %f\n", cur_pose.p[0], cur_pose.p[1], cur_pose.p[2]);
    printf("Test    = %f %f %f\n", test_pose.p[0], test_pose.p[1], test_pose.p[2]);
    double roll, pitch, yaw;
    //cur_pose.M.GetRPY(roll, pitch, yaw);
    //printf("Current rpy = %f %f %f\n", roll, pitch, yaw);
  }

  // iterate through each constraint, check if anything is violated:
  for (unsigned int i=0; i<cart_constraints_.size(); i++)
  {
    bool violated = true;
    for (int j=0; j<3; j++)
    {
      if (cart_constraints_[i](0,j) > pose.p[j] ||
          cart_constraints_[i](1,j) < pose.p[j])
      {
        violated = false;
        break;
      }
    }
    if (violated)
      return true;
  }
  return false;
}

void JointChainSineController::addCartesianConstraint(TiXmlElement *constraint)
{
  Eigen::Matrix<double, 2, 3> matrix;
  const char* strings[2];

  strings[0] = constraint->Attribute("min");
  strings[1] = constraint->Attribute("max");

  for (int i=0; i<2; i++)
  {
    if (!strings[i])
    {
      ROS_WARN("JointChainSineController: error in constraints");
      return;
    }
    double val;
    stringstream ss(strings[i]);
    for (int j=0; j<3; j++)
    {
      ss >> val;
      matrix(i,j) = val;
    }
  }

  cout << matrix << endl;

  cart_constraints_.push_back(matrix);
}

}
