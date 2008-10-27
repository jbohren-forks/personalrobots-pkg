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
 #include <rosconsole/rosconsole.h>
 #include <Eigen/LU>
 
bool SerialChainModel::init(const std::string & robot_description, const std::string & chain_name)
{
  robot_kin_.loadString(robot_description.c_str());
  chain_=robot_kin_.getSerialChain(chain_name);
  return init(robot_kin_,chain_);
}
 
bool SerialChainModel::init(const robot_kinematics::RobotKinematics & r_kin, robot_kinematics::SerialChain *chain)
{
  robot_kin_=r_kin;
  chain_=chain;
  if(!chain)
    return false;
  inputs_=chain_->num_joints_;
  states_=2*inputs_; //(angle and angular velocity)
  kdl_torque_ = new KDL::Vector[inputs_+1];
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
bool SerialChainModel::getLinearization(const StateVector & x, StateMatrix & A, InputMatrix & B, InputVector & c)
{
  ROS_DEBUG("1");
  assert(inputs_>0);
  const int n=inputs_;
  KDL::JntArray kdl_q(n);
  KDL::JntArray kdl_q_dot(n);

  ROS_DEBUG("2");
  //Fills the data
  for(int i=0; i<n; ++i)
  {
    kdl_q(i) = x(i);
    kdl_q_dot(i) = x(i+n);
  }
  
  ROS_DEBUG("3");
  //Compute M
  Eigen::MatrixXd M(n,n);
  NEWMAT::Matrix mass;
  chain_->computeMassMatrix(kdl_q,kdl_torque_,mass);
  for(int i=0;i<n;i++)
    for(int j=0;j<n;j++)
      M(i,j)=mass(i+1,j+1);
  ROS_DEBUG("4");
  //Compute Q
  Eigen::MatrixXd Q(n,n);
  Q.setZero();
  NEWMAT::Matrix christoffel;
  chain_->computeChristoffelSymbols(kdl_q,kdl_torque_,christoffel);
  for(int i=0;i<n;i++)
    for(int j=0;j<n;j++)
      for(int k=0;k<n;k++)
        Q(i,j)+=christoffel(i*n+j,k)*kdl_q_dot(j);
  ROS_DEBUG("5");
  //Compute G
  Eigen::VectorXd G(n,1);
  chain_->computeGravityTerms(kdl_q,kdl_torque_);
  for(int i=0;i<n;i++)
    G(i)=kdl_torque_[i][2];
  ROS_DEBUG("6");
  //Computing the gradient of G
  Eigen::MatrixXd gG(n,n);
  const double epsi=0.01;
  for(int i=0;i<n;i++)
  {
    KDL::JntArray ja = kdl_q;
    ja(i)=ja(i)+epsi;
    chain_->computeGravityTerms(ja,kdl_torque_);
    for(int j=0;j<n;j++)
      gG(i,j)=(kdl_torque_[j][2]-G(j))/epsi;
  }
  ROS_DEBUG("7");
  // Final assembling
  Eigen::MatrixXd Minvminus=-M.inverse(); //Computing M's inverse once
  
  A.block(0,0,n,n).setZero();
  A.block(0,n,n,n)=Eigen::MatrixXd::Identity(n,n);
  A.block(n,0,n,n)=Minvminus*gG;
  A.block(n,n,n,n)=Minvminus*Q;
  
  B.block(0,0,n,n).setZero();
  B.block(n,0,n,n)=Minvminus;
  ROS_DEBUG("8");
  c=-G;
  return true;
}

bool SerialChainModel::forward(const StateVector & x, const InputVector & u, double dt, StateVector & next)
{
  return false; // Not implemented
}