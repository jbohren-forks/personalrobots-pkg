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

// Author: Stuart Glaser, Wim Meeussen

#include "pr2_mechanism_model/chain.h"

namespace pr2_mechanism {

using namespace std;

bool Chain::init(Robot *robot, const std::string &root, const std::string &tip)
{

  robot_ = robot;

  // Constructs the kdl chain
  if (!robot_->robot_model_.getChain(root, tip, kdl_chain_)) return false;

  // Pulls out all the joint indices
  joint_indices_.clear();
  all_joint_indices_.clear();
  for (size_t i=0; i<kdl_chain_.getNrOfSegments(); i++){
    int jnt_index = robot_->getJointIndex(kdl_chain_.getSegment(i).getJoint().getName());
    if (jnt_index == -1){
      ROS_ERROR("Could not find joint n%s", kdl_chain_.getSegment(i).getJoint().getName().c_str());
      return false;
    }
    ROS_DEBUG("Index of joint %s at link %s is %i", 
              kdl_chain_.getSegment(i).getJoint().getName().c_str(), kdl_chain_.getSegment(i).getName().c_str(), jnt_index);
    all_joint_indices_.push_back(jnt_index);
    if (robot_->joints_[jnt_index]->type_ == urdf::Joint::REVOLUTE || 
        robot_->joints_[jnt_index]->type_ == urdf::Joint::CONTINUOUS || 
        robot_->joints_[jnt_index]->type_ == urdf::Joint::PRISMATIC)
      joint_indices_.push_back(jnt_index);
  }
  ROS_DEBUG("Added %i joints", joint_indices_.size());

  return true;
}

void Chain::getPositions(std::vector<JointState> &states, std::vector<double> &positions)
{
  positions.resize(joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
  {
    positions[i] = states[joint_indices_[i]].position_;
  }
}

void Chain::getVelocities(std::vector<JointState> &states, std::vector<double> &velocities)
{
  velocities.resize(joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
  {
    velocities[i] = states[joint_indices_[i]].velocity_;
  }
}

void Chain::getEfforts(std::vector<JointState> &states, std::vector<double> &efforts)
{
  efforts.resize(joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
  {
    efforts[i] = states[joint_indices_[i]].applied_effort_;
  }
}

bool Chain::allCalibrated(std::vector<JointState> &js)
{
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
  {
    if (!js[joint_indices_[i]].calibrated_)
      return false;
  }
  return true;
}

void Chain::toKDL(KDL::Chain &chain)
{
  chain = kdl_chain_;
}


void Chain::getPositions(std::vector<JointState>& s, KDL::JntArray& a)
{
  assert(a.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
    a(i) = s[joint_indices_[i]].position_;
}

void Chain::getVelocities(std::vector<JointState>& s, KDL::JntArrayVel& a)
{
  assert(a.q.rows() == joint_indices_.size());
  assert(a.qdot.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i){
    a.q(i) = s[joint_indices_[i]].position_;
    a.qdot(i) = s[joint_indices_[i]].velocity_;
  }
}

void Chain::getEfforts(std::vector<JointState>& s, KDL::JntArray& a)
{
  assert(a.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
    a(i) = s[joint_indices_[i]].applied_effort_;
}

void Chain::setEfforts(KDL::JntArray& a, std::vector<JointState>& s)
{
  assert(a.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
    s[joint_indices_[i]].commanded_effort_ = a(i);
}

void Chain::addEfforts(KDL::JntArray& a, std::vector<JointState>& s)
{
  assert(a.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
    s[joint_indices_[i]].commanded_effort_ += a(i);
}


Joint *Chain::getJoint(unsigned int actuated_joint_i)
{
  return robot_->joints_[joint_indices_[actuated_joint_i]];
}



}
