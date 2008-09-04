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

/*
 * Author: Stuart Glaser
 */

#include "mechanism_model/chain.h"
#include "mechanism_model/robot.h"

namespace mechanism {

Chain::Chain() : solverFK_(NULL)
{
}

Chain::~Chain()
{
  if (solverFK_)
    delete solverFK_;
}

bool Chain::initXml(TiXmlElement *config, Robot *robot)
{
  const char *root_name = config->Attribute("root");
  const char *tip_name = config->Attribute("tip");
  if (!root_name || !tip_name)
  {
    fprintf(stderr, "Error: Chain must have both a root link and a tip link specified.\n");
    return false;
  }

  Link *root = robot->getLink(root_name);
  Link *tip = robot->getLink(tip_name);

  if (!root)
  {
    fprintf(stderr, "Error: Chain could not find root link \"%s\"\n", root_name);
    return false;
  }

  if (!tip)
  {
    fprintf(stderr, "Error: Chain could not find tip link \"%s\"\n", tip_name);
    return false;
  }

  // Pulls out the links that make up the chain.
  for (Link *it = tip; it != root; it = it->parent_)
  {
    if (!it)
    {
      fprintf(stderr, "Error: Chain's tip link \"%s\" is not a descendant of the root link \"%s\"\n",
              tip_name, root_name);
      return false;
    }
    links_.push_back(it);
  }
  links_.push_back(root);
  std::reverse(links_.begin(), links_.end());

#if 0

  // Adds segments to the KDL chain.
  for (unsigned int i = 1; i < links_.size(); ++i)
    kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), links_[i]->frame_));

  // Initializes the solver.
  solverFK_ = new KDL::ChainFkSolverPos_recursive(kdl_chain_);
#endif

  return true;
}

bool Chain::propagateFK()
{
  assert(solverFK_);
#if 0

  // Copies the joint angles into the KDL structure.
  KDL::JntArray ja(links_.size() - 1);
  for (unsigned int i = 0; i < ja.rows(); ++i)
    ja(i) = links_[i+1]->joint_->position_;

  // Forward kinematics (placing results into links_)
  for (unsigned int i = 0; i < links_.size(); ++i)
    assert(solverFK_->JntToCart(ja, links_[i]->frame_, i) >= 0);
#endif
  return true;
}

} // namespace mechanism
