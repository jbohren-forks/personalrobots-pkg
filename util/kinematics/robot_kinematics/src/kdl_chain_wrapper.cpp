/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/*
 * KDLChainWrapper.cpp
 *
 *  Created on: Jan 29, 2009
 *      Author: Timothy Hunter <tjhunter@willowgarage.com>
 */

#include <robot_kinematics/kdl_chain_wrapper.h>

using namespace robot_kinematics;
using namespace Eigen;
//
KDLChainWrapper::KDLChainWrapper(const KDL::Chain &chain) :
  chain_(chain)
{}

int KDLChainWrapper::segments() const
{
  return chain_.getNrOfSegments();
}

KDLChainWrapper::Rotation KDLChainWrapper::relRotation(int i, const Scalar &q_i) const
{
  const KDL::Rotation & rot=chain_.getSegment(i).pose(q_i).M;
  return Rotation(Map<Rotation>(rot.data));
}

KDLChainWrapper::Vector KDLChainWrapper::center(int i) const
{
  return Vector(Map<Vector>(chain_.getSegment(i).getCM().data));
}

KDLChainWrapper::Inertia KDLChainWrapper::inertia(int i) const
{
  return Inertia(Map<Inertia>(chain_.getSegment(i).getInertia().I.data));
}

KDLChainWrapper::Rotation KDLChainWrapper::tipRot(int i) const
{
  return Rotation(Map<Rotation>(chain_.getSegment(i).getFrameToTip().M.data));
}

KDLChainWrapper::Vector KDLChainWrapper::tipVec(int i) const
{
  return Vector(Map<Vector>(chain_.getSegment(i).getFrameToTip().p.data));
}

KDLChainWrapper::Vector KDLChainWrapper::axis(int i) const
{
  return Vector(Map<Vector>(chain_.getSegment(i).getJoint().JointAxis().data));
}

KDLChainWrapper::Scalar KDLChainWrapper::mass(int i) const
{
  return chain_.getSegment(i).getInertia().m;
}

KDLChainWrapper::Vector KDLChainWrapper::gravity() const
{
  return Vector(0,0,-9.8);
}
