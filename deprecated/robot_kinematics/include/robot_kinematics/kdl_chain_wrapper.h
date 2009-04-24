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

#ifndef KDLCHAINWRAPPER_H_
#define KDLCHAINWRAPPER_H_
#include <Eigen/Core>
#include <kdl/chain.hpp>

namespace robot_kinematics
{

/**
 * A wrapper for the RevoluteSerialChainRNESolver around a KDL chain.
 *
 * KDLChainWrapper.h
 *
 *  Created on: Jan 27, 2009
 *      Author: Timothy Hunter <tjhunter@willowgarage.com>
 *
 */
class KDLChainWrapper
{
public:
  KDLChainWrapper(const KDL::Chain &chain=KDL::Chain());
  enum
  {
    Length=Eigen::Dynamic
  };
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar,3,3, Eigen::RowMajorBit|Eigen::AutoAlign> Rotation;
  typedef Eigen::Matrix<Scalar,3,3> Inertia;
  typedef Eigen::Matrix<Scalar,3,1> Vector;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Parameters;

  int segments() const;

  const KDL::Chain & chain() const {return chain_;}

  Rotation relRotation(int, const Scalar &) const;

  Vector center(int i) const;

  Inertia inertia(int i) const;

  Vector tipVec(int i) const;

  Rotation tipRot(int i) const;

  Vector axis(int i) const;

  Scalar mass(int i) const;

  Vector gravity() const;
private:
  KDL::Chain chain_;
};

//class KDLChainWrapper
//{
//public:
//  inline KDLChainWrapper(const KDL::Chain &chain=KDL::Chain())  :
//	  chain_(chain)
//	{}
//  enum
//  {
//    Length=Eigen::Dynamic
//  };
//  typedef double Scalar;
//  typedef Eigen::Matrix<Scalar,3,3, Eigen::RowMajorBit|Eigen::AutoAlign> Rotation;
//  typedef Eigen::Matrix<Scalar,3,3> Inertia;
//  typedef Eigen::Matrix<Scalar,3,1> Vector;
//  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Parameters;
//
//  inline int segments() const
//  {
//    return chain_.getNrOfSegments();
//  }
//
//  const KDL::Chain & chain() const {return chain_;}
//
//  inline Rotation relRotation(int i, const Scalar &q_i) const
//  {
//    const KDL::Rotation & rot=chain_.getSegment(i).pose(q_i).M;
//    return Rotation(Eigen::Map<Rotation>(rot.data));
//  }
//
//  inline Vector center(int i) const
//  {
//    return Vector(Eigen::Map<Vector>(chain_.getSegment(i).getCM().data));
//  }
//
//  inline Inertia inertia(int i) const
//  {
//    return Inertia(Eigen::Map<Inertia>(chain_.getSegment(i).getInertia().I.data));
//  }
//
//  inline Vector tipVec(int i) const
//  {
//    return Vector(Eigen::Map<Vector>(chain_.getSegment(i).getFrameToTip().p.data));
//  }
//
//  inline Rotation tipRot(int i) const
//  {
//    return Rotation(Eigen::Map<Rotation>(chain_.getSegment(i).getFrameToTip().M.data));
//  }
//
//  inline Vector axis(int i) const
//  {
//    return Vector(Eigen::Map<Vector>(chain_.getSegment(i).getJoint().JointAxis().data));
//  }
//
//  inline Scalar mass(int i) const
//  {
//    return chain_.getSegment(i).getInertia().m;
//  }
//
//  Vector gravity() const
//  {
//    return Vector(0,0,-9.8);
//  }
//private:
//  KDL::Chain chain_;
//};

}

#endif /* KDLCHAINWRAPPER_H_ */
