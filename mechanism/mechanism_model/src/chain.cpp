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

#include "mechanism_model/chain.h"

namespace mechanism {

bool Chain::init(Robot *robot, const std::string &root, const std::string &tip)
{
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
    joint_indices_.push_back(tip_joints[i]);
  }
  assert(joint_indices_.size() == link_indices_.size() - 1);

  return true;
}

void Chain::toKdl(KDL::Chain &chain)
{
}

void Chain::positionsToKDL(std::vector<JointState>&, KDL::JntArray&)
{
}

void Chain::setEffortsFromKDL(KDL::JntArray&, std::vector<JointState>&)
{
}

void Chain::addEffortsFromKDL(KDL::JntArray&, std::vector<JointState>&)
{
}

bool Chain::getAncestors(mechanism::Robot* robot, const std::string &link_name,
                         std::vector<int> links, std::vector<int> joints)
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
