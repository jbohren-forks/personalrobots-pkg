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
 * ChainSolver.h
 *
 *  Created on: Jan 3, 2009
 *      Author: Timothy Hunter <tjhunter@willowgarage.com>
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//#include <robot_kinematics/Array.hpp>


#include <ros/console.h>

#ifndef REVOLUTECHAINRNESOLVER_H_
#define REVOLUTECHAINRNESOLVER_H_

namespace robot_kinematics
{
/**
 * Implementation of the Newton Euler method in a vectorized way.
 * (i.e, with respect to q_dot, q_dotdot and 1)
 * The reference implementation is assumed to be the current Newton Euler implementation developped by Willow Garage for libKDL.
 *
 * This implementation is template based and heaviliy influenced by the Eigen Matrix library. In particulair, it makes use of the Eigen::Dynamic constant.
 *
 * Given a kinematic chain with revolute joints and a pose, computes in one go the inertia matrix, the christoffel symbols and the gravity terms. It also provides the linearization of the forces (at the center of mass of the joints) and torques (at the joint axis) with respect to qdotdot, and the cross-products qdot_i qdot_j.
 *
 * @note on the performance. Since the computations are done on vectors of size
 *  n^2, the overall cost is O(n^3). In practice, for a chain of 7 joints (like the arm) it performs about 10x slower than a single pass in KDL (computing the torques) and 4x faster than a computation of the dynamics (inertia, c. symbols and gravity terms).
 *
 * This class is templatized to let users switch between different implementations of chains.
 * See the example used to compute symbolic solutions of the RNE algorithm for example.
 * ChainType must be a class with the following parameters:
 *
 * static const int Length : the length of the chain (set it to Eigen::Dynamic if it is not known at compile-time)
 *
 * typedef Scalar : the types of the scalar used in the rotations, translations, etc. Usually float or double.
 * typedef Parameters : the type of the parameter q. Should respect the interface of a std::vector<>
 *
 * Some mathematical types are required:
 * typedef Vector : a 3d vector type
 * typedef Rotation : a 3d rotation type
 * typedef Inertia : an inertia type
 * Note that this class relies internally on the Eigen matrix library, so these types must be compatible.
 * In particular, you cannot directly use types from KDL unless you define some operators.
 * TODO: define some compatibility operators with KDL
 *
 * The wrapper class must define the following methods (which can be const, virtual or return references). All these methods should be familiar to KDL users.
 *
 *  int segments()  <-- the length of the chain
 *
 *  Rotation relRotation(int i, const Scalar & q_i) const <-- the rotation corresponding to the pose defined by q_i for the joint i
 *
 *  Vector center(int i) <-- center of mass in segment coordinates
 *
 *  Inertia inertia(int i) <-- inertia of segment i
 *
 *  Vector tipVec(int i) <-- vector from the origin to the tip of the segment
 *
 *  Rotation tipRot(int i) <-- the rotation from the origin of the segment to the tip
 *
 *  Vector axis(int i) <-- axis of the joint of segment i
 *
 *  Scalar mass(int i) <-- mass of segment i
 *
 *  Vector gravity() <-- the gravity in the reference frame of the chain
 *
 *  //TODO add external forces/torques
 *  //TODO consider reference frame
 *  //TODO remove all the .corner()
 *  //TODO pack the qdot products - might lead to a x2 speedup gain?
 *  //FIXME projection of the torques
 *
 *  //TODO it would not cost much to add the linearization of the speed around the COMs.
 */
template<typename ChainType>
class RevoluteChainRNESolver
{
public:
  enum
  {
    Length=ChainType::Length,
    /**< The length of the chain. */
    CSymbolLength=(Length==Eigen::Dynamic?Eigen::Dynamic:Length*Length),
    /**< Size of the Christoffel Symbols: all the qdot_i*qdot_j products. */
    AccelLength=(Length==Eigen::Dynamic?Eigen::Dynamic:Length+CSymbolLength+1)
    /**< Size of the acceleration vector: q_dotdot, all the cross products of q_dot and +1 for the gravitation terms. */
  };
  /** Scalar type. */
  typedef typename ChainType::Scalar Scalar;
  /** Type of the parameters for the chain (the vector q). */
  typedef typename ChainType::Parameters Parameters;
  /** Type ofthe inertia matrix. */
  typedef Eigen::Matrix<Scalar,Length,Length> InertiaType;
  /** Chirstoffel symbols matrix type. One line per symbol.
   * The storage order is m(i,j)=q_i*q_j : the commutativity is not used to compact the vector. */
  typedef Eigen::Matrix<Scalar,Length,CSymbolLength> CSymbolsType;
  /** Type of a vector. */
  typedef Eigen::Matrix<Scalar,Length,1> VectorType;
  /** Type of the matrix representing a force: 3D by accel size. */
  typedef Eigen::Matrix<Scalar,3,AccelLength> ForceType;
  typedef Eigen::Matrix<Scalar,3,Length> AngVelType;
  typedef Eigen::Matrix<Scalar,3,AccelLength> AccelType;

  /** Default constructor. */
  RevoluteChainRNESolver(const ChainType & chain=ChainType());

  /** Computes the elements given a new pose.
   * Does all the computation. */
  void compute(const Parameters &q);

  /**
   * Returns the inertia matrix.
   */
  inline const InertiaType & inertia() const {return D_;}

  /**
   * Cristoffel symbols.
   * Each line is for a different joint; q_j*q_i is constained in m(.,j*n+i)
   */
  inline const CSymbolsType & cSymbols() const { return cs_; }

  /**
   * Gravity terms.
   */
  inline const VectorType & gravity() const { return g_; }

  /**
   * Linearization of the forces.
   * The vector returned is [v(0) v(1) .... v(n+n*n+1)] with v(i) a 3d vector.
   * The force f on the joint u is then:
   * f = qdotdot_0 v(0) + ... qdotdot_i v(i)
   *     + qdot_i qdot_j v(n+n*j+i) + ...
   *     + v(n+n^2)
   * @param u the index of the joint (from 0 to n-1)
   */
  inline const ForceType & forces(int u) const { return forces_[u]; }

  /**
   * Linearization of the torques.
   * The vector returned is [v(0) v(1) .... v(n+n*n+1)] with v(i) a 3d vector.
   * The force t on the joint u at its joint axis is then:
   * t = qdotdot_0 v(0) + ... qdotdot_i v(i)
   *     + qdot_i qdot_j v(n+n*j+i) + ...
   *     + v(n+n^2)
   * @param u the index of the joint (from 0 to n-1)
   */
  inline const ForceType & torques(int i) const { return torques_[i]; }

  /**
   * The angular rates of joint i expressed as a linear combination of the qdot_j
   */
  inline const AngVelType & omega(int i) const { return omega_[i]; }


  inline const AccelType & alpha(int i) const { return alpha_[i]; }

  /**
   * Acceleration of the center of mass of each joint.
   * The vector returned is [v(0) v(1) .... v(n+n*n+1)] with v(i) a 3d vector.
   * The acceleration a at the center of mass of segment u is then:
   * a = qdotdot_0 v(0) + ... qdotdot_i v(i)
   *     + qdot_i qdot_j v(n+n*j+i) + ...
   *     + v(n+n^2)
   * @param u the index of the joint (from 0 to n-1)
   */
  inline const AccelType & a_c(int i) const { return a_c_[i]; }

  /**
   * @brief Helper function that computes the AccelType vector from qdot and qdotdot;
   */
  template<typename T>
  void toAccelVector(const Parameters &qdot, const Parameters &qdotdot, Eigen::MatrixBase<T> &) const;


private:
  // Internal storage types.
  typedef std::vector<AngVelType> AngVelStorage;
  typedef std::vector<AccelType> AccelStorage;
  typedef std::vector<ForceType> ForceStorage;
//  typedef Eigen::StorageArray<AngVelType,Length> AngVelStorage;
//  typedef Eigen::StorageArray<AccelType,Length> AccelStorage;
//
//  typedef Eigen::StorageArray<ForceType,Length> ForceStorage;
  typedef ForceStorage TorqueStorage;
  typedef ForceType TorqueType;

  // Implementation of the cross prodcut column-wise on a matrix.
  // TODO: use Eigen facilities.
  template<typename T1, typename T2>
  T1 cross(const T1 & v1, const T2 & v2);
  template<typename T1, typename T2, typename T3>
  void addCross2(const T1 & v1, const T2 & v2, T3 & v3);

  int length_;
  int cSymbolLength_;
  int accelLength_;

  /** Acceleration of center of mass. */
  /** The terms are packed together 3 lines by 3 lines. */
  AccelStorage a_c_;
  AccelStorage a_e_;
  AngVelStorage omega_;
  AccelStorage alpha_;

  ForceStorage torques_;
  TorqueStorage forces_;

  InertiaType D_;
  CSymbolsType cs_;
  VectorType g_;

  ChainType chain_;
};

template<typename ChainType>
RevoluteChainRNESolver<ChainType>::RevoluteChainRNESolver(const ChainType & chain) :
  length_(std::max(chain.segments(),1)),
  cSymbolLength_(length_*length_),
  accelLength_(cSymbolLength_+length_+1),
  a_c_(length_,AccelType::Zero(3,accelLength_).eval()),
  a_e_(length_,AccelType::Zero(3,accelLength_).eval()),
  omega_(length_,AngVelType::Zero(3,length_).eval()),
  alpha_(length_,AccelType::Zero(3,accelLength_).eval()),
  torques_(length_,ForceType::Zero(3,accelLength_).eval()),
  forces_(length_,ForceType::Zero(3,accelLength_).eval()),
  D_(length_,length_),
  cs_(length_,cSymbolLength_),
  g_(length_,1),
  chain_(chain)
{
}


//TODO detail the algorithm. It is not very clear like that...
template<typename ChainType>
void RevoluteChainRNESolver<ChainType>::compute(const typename RevoluteChainRNESolver<ChainType>::Parameters &q)
{
  typedef typename ChainType::Rotation Rotation;
  typedef typename ChainType::Vector Vector;


  ei_assert(length_==g_.rows());

//  ROS_DEBUG("1");

//  ROS_DEBUG("2");
  Rotation R_i_im1_t=chain_.relRotation(0,q[0]).transpose();;

  //FIXME: set to initial frame of the chain
  Rotation rot_end=R_i_im1_t;
//  ROS_DEBUG("2");

  //1st joint:
  const Vector & b_0 = chain_.axis(0);
//  ROS_DEBUG_STREAM(""<<b_0);
  omega_[0].col(0) = b_0;
//  ROS_DEBUG("2");
//  ROS_DEBUG_STREAM(""<<omega_[0]);
  alpha_[0].col(0) = b_0;
//  ROS_DEBUG("2");
  alpha_[0].block(0,length_,3,length_)+=cross(omega_[0],b_0);
  a_e_[0].corner(Eigen::TopLeft,3,accelLength_)+=cross(alpha_[0],chain_.tipVec(0));
//  ROS_DEBUG("2");
  addCross2(omega_[0],cross(omega_[0],chain_.tipVec(0)),a_e_[0]);
  a_c_[0].corner(Eigen::TopLeft,3,accelLength_)+=cross(alpha_[0],chain_.center(0));
//  ROS_DEBUG("2");
  addCross2(omega_[0],cross(omega_[0],chain_.center(0)),a_c_[0]);

//  ROS_DEBUG("2");
  //Iterations - forward pass
  for(int i=1;i<length_;++i)
  {
    const Rotation & relRotTi = chain_.relRotation(i,q[i]).transpose();
    R_i_im1_t = chain_.tipRot(i)*relRotTi*chain_.tipRot(i-1).transpose();

    rot_end=relRotTi*rot_end;

    const Vector & b_i = chain_.axis(i);

    omega_[i]=(R_i_im1_t*omega_[i-1]).eval();
    omega_[i].col(i)+=b_i;

    alpha_[i] = (R_i_im1_t*alpha_[i-1]).eval();
    alpha_[i].col(i) += b_i;
    //Computing the cross product of omega_i and b_i*qdot_i.
    // +length due to the qdotdots
    //FIXME: optimize here as well
    alpha_[i].block(0,i*(length_)+length_,3,length_)+=cross(omega_[i],b_i);

    a_e_[i] = (R_i_im1_t*a_e_[i-1]).eval();
    //FIXME: no need?
    a_e_[i].corner(Eigen::TopLeft,3,accelLength_)+=cross(alpha_[i],chain_.tipVec(i));
    addCross2(omega_[i],cross(omega_[i],chain_.tipVec(i)),a_e_[i]);


    a_c_[i] = (R_i_im1_t*a_e_[i-1]).eval();
    a_c_[i].corner(Eigen::TopLeft,3,accelLength_)+=cross(alpha_[i],chain_.center(i));
    addCross2(omega_[i],cross(omega_[i],chain_.center(i)),a_c_[i]);
//    ROS_DEBUG("FW %i",i);
  }

//  ROS_DEBUG("10");

  //Last joint:
  Vector g=chain_.tipRot(length_-1)*rot_end*chain_.gravity();
  forces_[length_-1]+=chain_.mass(length_-1)*(a_c_[length_-1]);
  forces_[length_-1].col(accelLength_-1)-=chain_.mass(length_-1)*g;
  torques_[length_-1] -= cross(forces_[length_-1],chain_.center(length_-1));
  addCross2(omega_[length_-1],chain_.inertia(length_-1)*omega_[length_-1],torques_[length_-1]);
  //FIXME: no longer necessary?
  torques_[length_-1].corner(Eigen::TopLeft,3,accelLength_)+=chain_.inertia(length_-1)*alpha_[length_-1];

  for(int i=length_-2;i>=0;--i)
  {
    Rotation R_ip1_i = chain_.tipRot(i);

    R_ip1_i=chain_.tipRot(i)*chain_.relRotation(i+1,q(i+1))*chain_.tipRot(i+1).transpose().eval();

    g=R_ip1_i*g;

    forces_[i] = (R_ip1_i*forces_[i+1]).eval();

    forces_[i]+=chain_.mass(i)*(a_c_[i]);
    forces_[i].col(accelLength_-1)-=chain_.mass(i)*g;

    torques_[i] = (R_ip1_i*torques_[i+1]
      +cross((R_ip1_i*forces_[i+1]).eval(),chain_.center(i)-chain_.tipVec(i))).eval();

    torques_[i] -= cross(forces_[i],chain_.center(i));
    addCross2(omega_[i],chain_.inertia(i)*omega_[i],torques_[i]);
    //FIXME no longer necessary?
    torques_[i].corner(Eigen::TopLeft,3,accelLength_)+=chain_.inertia(i)*alpha_[i];
  }
//  ROS_DEBUG("20");

  //Stores the results.
  for(int i=0;i<length_;++i)
  {
    D_.row(i)=torques_[i].row(2).segment(0,length_);
    cs_.row(i)=torques_[i].row(2).segment(length_,length_*length_);
    g_(i)=torques_[i](2,accelLength_-1);
  }
//  ROS_DEBUG("30");

}

template<typename ChainType>
template<typename T>
void RevoluteChainRNESolver<ChainType>::toAccelVector(const typename RevoluteChainRNESolver<ChainType>::Parameters &qdot, const typename RevoluteChainRNESolver<ChainType>::Parameters &qdotdot, Eigen::MatrixBase<T> &res) const
{
	assert((res.cols()==1)&&"The acceleration vector must be a vector");
	assert((res.rows()==accelLength_)&&"THE ACCEL VECTOR MUST BE OF LENGTH n^2+n+1");
	res.segment(0,length_)=qdotdot;
	res(accelLength_-1)=Scalar(1);
	for(int i=0;i<length_;++i)
		res.segment(length_+i*length_,length_)=qdot(i)*qdot;
}

//TODO optimize these cross products

template<typename ChainType>
template<typename T1, typename T2>
T1 RevoluteChainRNESolver<ChainType>::cross(const T1 & v1, const T2 & v2)
{
  T1 res=T1::Zero(v1.rows(),v1.cols()).eval();
  for(int i=0;i<v1.cols();++i)
    res.col(i)=v1.col(i).cross(v2);
  return res;
}

template<typename ChainType>
template<typename T1, typename T2, typename T3>
void RevoluteChainRNESolver<ChainType>::addCross2(const T1 & v1, const T2 & v2, T3 & v3)
{
  const int n=v1.cols();
  assert(n==v2.cols());
  for(int i=0;i<v1.cols();++i)
    for(int j=0;j<v2.cols();++j)
      v3.col(i*n+j+n)+=v1.col(i).cross(v2.col(j));
}

}

#endif /* CHAINSOLVER_H_ */
