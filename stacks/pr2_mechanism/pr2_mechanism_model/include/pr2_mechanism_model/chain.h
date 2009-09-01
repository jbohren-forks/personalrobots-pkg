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

#include "pr2_mechanism_model/robot.h"
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntarrayacc.hpp>

namespace pr2_mechanism {

class Chain
{
public:
  Chain() {}
  ~Chain() {}

  bool init(Robot *robot, const std::string &root, const std::string &tip);

  void getPositions(std::vector<JointState>&, std::vector<double>&);
  void getPositions(std::vector<JointState>&, KDL::JntArray&);

  void getVelocities(std::vector<JointState>&, std::vector<double>&);
  void getVelocities(std::vector<JointState>&, KDL::JntArrayVel&);

  void getEfforts(std::vector<JointState>&, std::vector<double>&);
  void getEfforts(std::vector<JointState>&, KDL::JntArray&);

  void setEfforts(KDL::JntArray&, std::vector<JointState>&);

  void addEfforts(KDL::JntArray&, std::vector<JointState>&);

  bool allCalibrated(std::vector<JointState>&);

  // get KDL chain
  void toKDL(KDL::Chain &chain);

  Joint* getJoint(unsigned int actuated_joint_i);

private:
  pr2_mechanism::Robot *robot_;
  KDL::Chain kdl_chain_;

  std::vector<int> joint_indices_;  // ONLY joints that can be actuated (not fixed joints)
  std::vector<int> all_joint_indices_;  // Includes fixed joints
};

}

#endif
