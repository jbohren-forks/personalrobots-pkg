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

// Author: Stuart Glaser

#include "mechanism_model/chain.h"


namespace mechanism {

void TransformTFToKDL(const tf::Transform &t, KDL::Frame &k)
{
  for (unsigned int i = 0; i < 3; ++i)
    k.p.data[i] = t.getOrigin()[i];
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = t.getBasis()[i/3][i%3];
}

void TransformKDLToTF(const KDL::Frame &k, tf::Transform &t)
{
  t.setOrigin(tf::Vector3(k.p.data[0], k.p.data[1], k.p.data[2]));
  t.setBasis(btMatrix3x3(k.M.data[0], k.M.data[1], k.M.data[2],
                         k.M.data[3], k.M.data[4], k.M.data[5],
                         k.M.data[6], k.M.data[7], k.M.data[8]));
}

void RotationTFToKDL(const tf::Quaternion& t, KDL::Rotation& k) 
{
  k.Quaternion(t[0], t[1], t[2], t[3]);
}

void VectorTFToKDL(const tf::Vector3& t, KDL::Vector& k) 
{
  k = KDL::Vector(t[0], t[1], t[2]);
}


bool Chain::init(Robot *robot, const std::string &root, const std::string &tip)
{
  robot_ = robot;

  // Here we find the chain of links that connects the root to the
  // tip.  We find the common ancestor of the two, and then we link
  // together the chain that connects the root to the ancestor and the
  // chain that connects the tip to the ancestor.

  std::vector<int> root_joints, root_links, tip_joints, tip_links;
  if (!getAncestors(robot, root, root_links, root_joints))
    return false;
  if (!getAncestors(robot, tip, tip_links, tip_joints))
    return false;

  // Finds the first common ancestor
  int ancestor_index = -1;  // Indexes into the root_links and tip_links arrays
  while (ancestor_index + 1 < (int)root_links.size() &&
         ancestor_index + 1 < (int)tip_links.size() &&
         root_links[ancestor_index+1] == tip_links[ancestor_index+1])
  {
    ++ancestor_index;
  }
  if (ancestor_index < 0)
  {
    fprintf(stderr, "Chain: The links \"%s\" and \"%s\" are not connected\n",
            root.c_str(), tip.c_str());
    return false;
  }

  // Constructs the full chain, from root to ancestor to tip.
  joint_indices_.clear();
  all_joint_indices_.clear();
  link_indices_.clear();

  for (size_t i = root_links.size() - 1; (int)i > ancestor_index; --i)
  {
    link_indices_.push_back(root_links[i]);
    all_joint_indices_.push_back(root_joints[i-1]);
  }
  reversed_index_ = link_indices_.size();
  link_indices_.push_back(root_links[ancestor_index]); // this link will not be used
  for (size_t i = ancestor_index + 1; i < tip_links.size(); ++i)
  {
    all_joint_indices_.push_back(tip_joints[i-1]);
    link_indices_.push_back(tip_links[i]);
  }
  assert(all_joint_indices_.size() == link_indices_.size() - 1);

  // Pulls out all the joints that are actuable (the non-fixed joints).
  for (unsigned int i = 0; i < all_joint_indices_.size(); ++i)
  {
    if (robot_->joints_[all_joint_indices_[i]]->type_ != JOINT_FIXED)
      joint_indices_.push_back(all_joint_indices_[i]);
  }

  return true;
}

void Chain::getPositions(std::vector<JointState> &states, std::vector<double> &positions)
{
  positions.resize(joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
  {
    positions[i] = states[i].position_;
  }
}

void Chain::getVelocities(std::vector<JointState> &states, std::vector<double> &velocities)
{
  velocities.resize(joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
  {
    velocities[i] = states[i].velocity_;
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
  assert(robot_);

  if (reversed_index_ != 0)
    ROS_ERROR("Creating KDL tree: walking down the mechanism tree is not supported yet");

  tf::Transform continuation;
  continuation.setIdentity();

  unsigned int i;
  // construct a kdl chain from root to ancestor to tip
  for (i = 0; i < link_indices_.size(); ++i)
  {
    // Here we create a KDL Segment for each part of the chain, between root and tip.
    // A KDL Segment consists of a link and a joint. The joint is positioned at the 
    // beginning of the link, and connects this link to the previous link.
    KDL::Frame kdl_link;
    KDL::Vector pos;
    KDL::Rotation rot;
    KDL::Vector axis;

    // Creates the link: a link is defined by a position and rotation, 
    // relative to the reference frame of the previous link.
    if (i == 0){
      kdl_link = KDL::Frame::Identity();
    }
    // moving from root to ancestor
    else if (i<= reversed_index_){
      VectorTFToKDL(robot_->links_[link_indices_[i-1]]->getOffset().getOrigin(), pos);
      RotationTFToKDL(robot_->links_[link_indices_[i-1]]->getRotation().getRotation(), rot);
      kdl_link = KDL::Frame(rot, pos).Inverse();
    }
    // moving from ancestor to tip
    else{
      VectorTFToKDL(robot_->links_[link_indices_[i]]->getOffset().getOrigin(), pos);
      RotationTFToKDL(robot_->links_[link_indices_[i]]->getRotation().getRotation(), rot);
      kdl_link = KDL::Frame(rot, pos);
    }

    // Creates the joint: a joint is defined by a position and an axis, 
    // relative to the reference frame of the previous link.
    KDL::Joint kdl_joint;
    if (i == 0 || robot_->joints_[all_joint_indices_[i-1]]->type_ == JOINT_FIXED){
      kdl_joint = KDL::Joint(KDL::Joint::None);
    }
    // moving from ancestor to tip
    else if (i<= reversed_index_){
      VectorTFToKDL(robot_->joints_[all_joint_indices_[i-1]]->axis_, axis);
      kdl_joint = KDL::Joint(KDL::Vector::Zero(), kdl_link.M * axis * -1, KDL::Joint::RotAxis);
    }
    // moving from ancestor to tip
    else{
      VectorTFToKDL(robot_->joints_[all_joint_indices_[i-1]]->axis_, axis);
      kdl_joint = KDL::Joint(kdl_link.p, axis, KDL::Joint::RotAxis);
  }

    // Combines the link and the joint into a segment, and adds the segment to the chain
    chain.addSegment(KDL::Segment(kdl_joint, kdl_link /*, inertia, com*/));
  }

  kdl_cached_ = true;
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


bool Chain::getAncestors(mechanism::Robot* robot, const std::string &link_name,
                         std::vector<int> &links, std::vector<int> &joints)
{
  int current_link = -1;
  int current_joint = -1;

  // Works up the kinematic tree from the chain's root.
  current_link = robot->getLinkIndex(link_name);
  if (current_link < 0)
  {
    fprintf(stderr, "Chain could not find link: %s\n", link_name.c_str());
    return false;
  }
  links.push_back(current_link);
  while (true)
  {
    current_joint = robot->getJointIndex(robot->links_[current_link]->joint_name_);
    if (current_joint < 0)
      break;  // Top of the kinematic tree
    current_link = robot->getLinkIndex(robot->links_[current_link]->parent_name_);
    if (current_link < 0)
      break;
    joints.push_back(current_joint);
    links.push_back(current_link);
  }

  std::reverse(links.begin(), links.end());
  std::reverse(joints.begin(), joints.end());

  assert(joints.size() == links.size() - 1);

  return true;
}


Joint *Chain::getJoint(unsigned int actuated_joint_i)
{
  return robot_->joints_[joint_indices_[actuated_joint_i]];
}

std::string Chain::getJointName(unsigned int actuated_joint_i)
{
  return getJoint(actuated_joint_i)->name_;
}


std::string Chain::getLinkName(int index)
{
  if (index == -1)
    index = link_indices_.size() - 1;

  return robot_->links_[link_indices_[index]]->name_;
}



}
