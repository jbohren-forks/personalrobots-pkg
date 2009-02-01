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

 #include <control_toolbox/serialchain_model.h>
 #include <ros/console.h>
 #include <Eigen/LU>

using namespace control_toolbox;

SerialChainModel::SerialChainModel():
	states_(-1),
	inputs_(-1)
{
	  ROS_DEBUG("SERIALCHAINMODEL");
}

bool SerialChainModel::init(const std::string & robot_description, const std::string & chain_name)
{
  ROS_DEBUG_STREAM("LOAD KIN STRING --"<<chain_name<<"--");
  robot_kinematics::RobotKinematics robot_kin;
  robot_kin.loadString(robot_description.c_str());
  robot_kinematics::SerialChain * c=robot_kin.getSerialChain(chain_name);
  if(!c)
  {
	  ROS_DEBUG_STREAM("CHAIN "<<chain_name<<" NOT FOUND.");
	  return false;
  }
  return init(c->chain);
}

bool SerialChainModel::init(const KDL::Chain & chain)
{
  robot_kinematics::KDLChainWrapper wrapper(chain);
  solver_=robot_kinematics::RevoluteChainRNESolver<robot_kinematics::KDLChainWrapper>(wrapper);
  inputs_=wrapper.segments();
  states_=2*inputs_; //(angle and angular velocity)
  return true;
}

// General outline: the model can be written:
//    M(o)o_dotdot + Q(o,o_dot)o_dot + G(o) = tau
// When we linearize to the first order around position O, we get the approximation: o = O + e and
//    M(O)e_dotdot + Q(O,O_dot)e_dot + G(O) + grad(G)(O)e = tau
// Or:
//    (e)_dot = I e_dot                                + 0
//    (e_dot)_dot = -M^-1 Q e_dot -M^-1 G e + M^-1 tau - G(O)
// We first compute the matrices M,Q,G and then linearize the gravity term.
// No linearizatiob of the gravity for now => we assume g==0
bool SerialChainModel::getLinearization(const StateVector & x, StateMatrix & A, InputMatrix & B, InputVector & c)
{
  ROS_DEBUG("1");
  assert(inputs_>0);
  const int n=inputs_;
  ROS_DEBUG("#of inputs is %i",n);
  const Eigen::VectorXd q=x.segment(0,n); /// n first elements -> q
  const Eigen::VectorXd q_dot=x.segment(n,n);
  ROS_DEBUG("computing elements");
  solver_.compute(q);
  ROS_DEBUG("computing elements done");
  //Reduction of the cristoffel symbols to a nxn matrix.
  ROS_DEBUG("filling C");
  StateMatrix C(n,n);
  for(int i=0;i<n;++i)
	  C.col(i)=solver_.cSymbols().block(0,n*i,n,n)*q_dot;
  ROS_DEBUG("filling C done");

  //The old fashion way
  KDL::JntArray kdl_q(n);
  KDL::JntArray kdl_q_dot(n);

//   ROS_DEBUG("2");
  //Fills the data
//  for(int i=0; i<n; ++i)
//  {
//    kdl_q(i) = x(i);
//    kdl_q_dot(i) = x(i+n);
//  }

//
//
////   ROS_DEBUG("3");
//  //Compute M
//  Eigen::MatrixXd M(n,n);
//  NEWMAT::Matrix mass(n,n);
//  chain_->computeMassMatrix(kdl_q,kdl_torque_,mass);
//  for(int i=0;i<n;i++)
//    for(int j=0;j<n;j++)
//      M(i,j)=mass(i+1,j+1);
////   ROS_DEBUG("4");
//  //Compute Q
//  Eigen::MatrixXd Q(n,n);
//  Q.setZero();
//  NEWMAT::Matrix christoffel(n*n,n);
//  chain_->computeChristoffelSymbols(kdl_q,kdl_torque_,christoffel);
////   ROS_DEBUG("5-");
//  for(int i=0;i<n;i++)
//    for(int j=0;j<n;j++)
//      for(int k=0;k<n;k++)
//        Q(i,j)+=christoffel(i*n+j+1,k+1)*kdl_q_dot(j);
////   ROS_DEBUG("5");
//  //Compute G
//  Eigen::VectorXd G(n,1);
//  chain_->computeGravityTerms(kdl_q,kdl_torque_);
//  for(int i=0;i<n;i++)
//    G(i)=kdl_torque_[i][2];
////   ROS_DEBUG("6");
//  //Computing the gradient of G
//  Eigen::MatrixXd gG(n,n);
//  const double epsi=0.01;
//  for(int i=0;i<n;i++)
//  {
//    KDL::JntArray ja = kdl_q;
//    ja(i)=ja(i)+epsi;
//    chain_->computeGravityTerms(ja,kdl_torque_);
//    for(int j=0;j<n;j++)
//      gG(i,j)=(kdl_torque_[j][2]-G(j))/epsi;
//  }
//   ROS_DEBUG("7");
  // Final assembling
  Eigen::MatrixXd Minvminus=-solver_.inertia().inverse(); //Computing M's inverse once
//  ROS_DEBUG_STREAM(A.rows());
//  assert(A.rows()==n*2);
//  assert(A.cols()==n*2);
  A<<Eigen::MatrixXd::Zero(n,n),Eigen::MatrixXd::Identity(n,n),
  Eigen::MatrixXd::Zero(n,n),Minvminus*C;

  B<<Eigen::MatrixXd::Zero(n,n),Minvminus;
  c.setZero();
//
//  A.block(0,0,n,n).setZero();
//  A.block(0,n,n,n)=
//  A.block(n,0,n,n)=Minvminus*solver_.gravity();
//  A.block(n,n,n,n)=Minvminus*Q;
//
//  assert(B.rows()==n*2);
//  assert(B.cols()==n);
//  B.block(0,0,n,n).setZero();
//  B.block(n,0,n,n)=Minvminus;
////   ROS_DEBUG("8");
//  c=-G;
  return true;
}

bool SerialChainModel::forward(const StateVector & x, const InputVector & u, double dt, StateVector & next)
{
  return false; // Not implemented
}


