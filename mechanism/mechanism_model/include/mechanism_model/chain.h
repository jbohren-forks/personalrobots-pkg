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

#ifndef MECHANISM_MODEL_CHAIN_H
#define MECHANISM_MODEL_CHAIN_H

#include "mechanism_model/robot.h"
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

namespace mechanism {

void TransformTFToKDL(const tf::Transform &t, KDL::Frame &k);
void TransformKDLToTF(const KDL::Frame &k, tf::Transform &t);

class Chain
{
public:
  Chain() : kdl_cached_(false) {}
  ~Chain() {}

  bool init(Robot *robot, const std::string &root, const std::string &tip);

  void getPositions(std::vector<JointState>&, std::vector<double>&);
  void getVelocities(std::vector<JointState>&, std::vector<double>&);

  // Constructs a KDL chain that corresponds to the mechanism chain.
  //
  // Be aware that frame i in the mechanism corresponds to frame i+1 in KDL.
  void toKDL(KDL::Chain &chain);

  void positionsToKDL(std::vector<JointState>&, KDL::JntArray&);

  void setEffortsFromKDL(KDL::JntArray&, std::vector<JointState>&);
  void addEffortsFromKDL(KDL::JntArray&, std::vector<JointState>&);

  // The robot model and KDL have different frames for each link.
  // These functions give the transforms between the robot model
  // frames and the KDL frames.
  //
  // Note: frame_index is the index of the MECHANISM frame.  The
  // corresponding KDL frame is frame_index+1
  tf::Transform getTransformToKDL(int frame_index);
  tf::Transform getTransformFromKDL(int frame_index);

  std::vector<int> joint_indices_;
  std::vector<int> link_indices_;

  // Unless the root is a direct ancestor of the tip, the chain likely
  // goes up the kinematic tree and then back down again.
  // reversed_index_ indicates where this switch occurs, pointing to
  // the link which is the ancestor of all the other links in the
  // chain.
  //
  // The switch point is important to keep track of because the Link
  // and Joint classes specify transforms of the child link in terms
  // of the parent link.  When we go from reversed_index_ to the tip,
  // the transforms are correct, but when we go from the root to
  // reversed_index_, the transforms are reversed.
  int reversed_index_;  // TODO: Not implemented properly

private:
  mechanism::Robot *robot_;

  // Helper function to get the sequence of links and joints from one
  // place on the kinematic tree until the top of the kinematic tree.
  // The specified link is returned as the last link in the results.
  static bool getAncestors(mechanism::Robot *robot, const std::string &link_name,
                           std::vector<int> &links, std::vector<int> &joints);

// Our model allows the joints to operate along any arbitrary axis,
// but the KDL joints operate along the Z axis.  This function returns
// the transform that aligns the joint's axis with the Z axis.
  static tf::Transform getKdlJointTransform(mechanism::Joint *j);

  // The KDL frames do not match the robot model frames because KDL
  // limits joints to the major axes.  kdl_transforms_ holds the
  // transform from the robot model model frame to the KDL frame for
  // each link.
  std::vector<tf::Transform> kdl_transforms_;
  bool kdl_cached_;
  void updateCachedKDL()
  {
    KDL::Chain c;
    toKDL(c);
  }
};

}

#endif
