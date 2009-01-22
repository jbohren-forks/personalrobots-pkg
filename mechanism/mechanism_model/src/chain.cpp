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
  link_indices_.clear();
  for (size_t i = root_links.size() - 1; (int)i > ancestor_index; --i)
  {
    link_indices_.push_back(root_links[i]);
    joint_indices_.push_back(root_joints[i-1]);
  }
  reversed_index_ = link_indices_.size();
  link_indices_.push_back(root_links[ancestor_index]);
  for (size_t i = ancestor_index + 1; i < tip_links.size(); ++i)
  {
    link_indices_.push_back(tip_links[i]);
    joint_indices_.push_back(tip_joints[i-1]);
  }
  assert(joint_indices_.size() == link_indices_.size() - 1);

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

void Chain::toKDL(KDL::Chain &chain)
{
  assert(robot_);
  KDL::Frame frame;
  kdl_transforms_.resize(link_indices_.size());

  tf::Transform continuation;
  continuation.setIdentity();

  size_t i;
  for (i = 0; i < link_indices_.size() - 1; ++i)
  {
    KDL::Joint kdl_joint;
    if (i == 0 || robot_->joints_[joint_indices_[i-1]]->type_ == JOINT_FIXED)
      kdl_joint = KDL::Joint::None;
    else
      kdl_joint = KDL::Joint::RotZ;

    tf::Transform align_next_joint = getKdlJointTransform(robot_->joints_[joint_indices_[i]]);

    tf::Transform kdl_segment =
      continuation *
      robot_->links_[i+1]->getOffset() *
      align_next_joint;
    kdl_transforms_[i] =
      (robot_->links_[i+1]->getOffset() *
       align_next_joint).inverse();

    TransformTFToKDL(kdl_segment, frame);
    chain.addSegment(KDL::Segment(kdl_joint, frame /*, inertia, com*/));


    continuation = align_next_joint.inverse() * robot_->links_[i+1]->getRotation();
  }

  // Adds the end-effector segment
  KDL::Joint kdl_joint = robot_->joints_[joint_indices_[i]]->type_ == JOINT_FIXED ?
    KDL::Joint::None : KDL::Joint::RotZ;
  kdl_transforms_[i].setIdentity();
  TransformTFToKDL(continuation, frame);
  chain.addSegment(KDL::Segment(kdl_joint, frame));

  kdl_cached_ = true;
}

tf::Transform Chain::getKdlJointTransform(mechanism::Joint *j)
{
  if (j->type_ == JOINT_FIXED)
    return tf::Transform::getIdentity();
  if (j->axis_[2] > 0.99999)
    return tf::Transform::getIdentity();

  tf::Vector3 z(0, 0, 1);
  return tf::Transform(
    tf::Quaternion(cross(j->axis_, z), -angle(j->axis_, z)));
}

void Chain::positionsToKDL(std::vector<JointState>& s, KDL::JntArray& a)
{
  assert(a.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
    a(i) = s[joint_indices_[i]].position_;
}

void Chain::setEffortsFromKDL(KDL::JntArray& a, std::vector<JointState>& s)
{
  assert(a.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
    s[joint_indices_[i]].commanded_effort_ = a(i);
}

void Chain::addEffortsFromKDL(KDL::JntArray& a, std::vector<JointState>& s)
{
  assert(a.rows() == joint_indices_.size());
  for (unsigned int i = 0; i < joint_indices_.size(); ++i)
    s[joint_indices_[i]].commanded_effort_ += a(i);
}

tf::Transform Chain::getTransformToKDL(int frame_index)
{
  assert(frame_index < (int)kdl_transforms_.size());
  if (!kdl_cached_)
    updateCachedKDL();

  return kdl_transforms_[frame_index].inverse();
}


tf::Transform Chain::getTransformFromKDL(int frame_index)
{
  assert(frame_index < (int)kdl_transforms_.size());
  if (!kdl_cached_)
    updateCachedKDL();

  return kdl_transforms_[frame_index];
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
    joints.push_back(current_joint);
    current_link = robot->getLinkIndex(robot->links_[current_link]->parent_name_);
    links.push_back(current_link);
  }

  std::reverse(links.begin(), links.end());
  std::reverse(joints.begin(), joints.end());

  assert(joints.size() == links.size() - 1);

  return true;
}


}
