//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <robot_kinematics/serial_chain.h>

#define eps 0.000001
//#define DEBUG 1

#define KDL_DYNAMICS

using namespace KDL;
using namespace std;
using namespace robot_kinematics;

SerialChain::SerialChain()
{
}

void SerialChain::finalize()
{
  this->num_joints_               = this->chain.getNrOfJoints();
  this->q_IK_guess                = new JntArray(this->num_joints_);
  this->q_IK_result               = new JntArray(this->num_joints_);
  this->forwardKinematics         = new ChainFkSolverPos_recursive(this->chain);
  this->differentialKinematicsInv = new ChainIkSolverVel_pinv(this->chain);
  this->differentialKinematics    = new ChainFkSolverVel_recursive(this->chain);
  this->inverseKinematics         = new ChainIkSolverPos_NR(this->chain, *this->forwardKinematics, *this->differentialKinematicsInv);
  
#ifdef KDL_DYNAMICS
  this->inverseDynamics           = new ChainIdSolver_NE(this->chain);
#endif  
  
  if(!this->link_kdl_frame_)
  {
    this->link_kdl_frame_         = new KDL::Frame[this->num_joints_];
  }
/*
  if(!this->joints_)
  {
    this->joints_ = new mechanism::Joint[this->num_joints_];
  }  
*/
}

bool SerialChain::computeFK(const KDL::JntArray &q, KDL::Frame &f)
{
	if (this->forwardKinematics->JntToCart(q,f) >= 0)
		return true;
	else
		return false;
}

bool SerialChain::computeFK(const KDL::JntArray &q, KDL::Frame &f, int i)
{
   if (this->forwardKinematics->JntToCart(q,f,i) >= 0)
		return true;
	else
		return false;
}

bool SerialChain::computeIK(const KDL::JntArray &q_init, const KDL::Frame &f, KDL::JntArray &q_out)
{
  if (this->inverseKinematics->CartToJnt(q_init,f,q_out) >= 0)
  {
    angle_within_mod180(q_out, this->num_joints_);
    return true;
  }
  else
    return false;
}

bool SerialChain::computeIK(const KDL::Frame &f, KDL::JntArray &q_out)
{
  if (this->inverseKinematics->CartToJnt(*this->q_IK_guess,f,*this->q_IK_result) >= 0)
  {
    angle_within_mod180(*this->q_IK_result, this->num_joints_);
    q_out = *this->q_IK_result;
    return true;
  }
  else
    return false;
}

bool SerialChain::computeIK(const KDL::Frame &f)
{
  if (this->inverseKinematics->CartToJnt(*this->q_IK_guess,f,*this->q_IK_result) >= 0)
  {
    angle_within_mod180(*this->q_IK_result, this->num_joints_);
    return true;
  }
  else
    return false;
}

bool SerialChain::computeDKInv(const JntArray & q_in, const Twist &v_in, JntArray & qdot_out)
{
  if (this->differentialKinematicsInv->CartToJnt(q_in,v_in, qdot_out) >= 0)
    return true;
  else
    return false;  
}

bool SerialChain::computeDK(const JntArrayVel & q_in, FrameVel& out)
{
  if (this->differentialKinematics->JntToCart(q_in,out) >= 0)
    return true;
  else
    return false;  
}

int SerialChain::getID(std::string name)
{
  std::map<std::string, int>::const_iterator it = joint_id_map_.find(name);
  return (it == joint_id_map_.end()) ? (-1) : (it->second);
}

#ifdef KDL_DYNAMICS
bool SerialChain::computeInverseDynamics(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, Vector* torque)
{

  if (this->inverseDynamics->InverseDynamics(q,q_dot,q_dotdot,torque) >= 0)
  {
     for(int i=0; i< (int) q.rows(); i++)
   {
//     fprintf(stderr,"%d:: %f, %f, %f, t:: %f, %f, %f\n",i,q(i),q_dot(i),q_dotdot(i),torque[i][0],torque[i][1],torque[i][2]);
   }
    return true;
  }
  else
    return false;  
}

bool SerialChain::computeGravityTerms(const JntArray &q, Vector* torque)
{

  JntArray qdot(q.rows());
  JntArray qdotdot(q.rows());

  SetToZero(qdot);
  SetToZero(qdotdot);

  if (this->inverseDynamics->InverseDynamics(q,qdot,qdotdot,torque) >= 0)
  {
    return true;
  }
  else
    return false;  
}

void SerialChain::computeMassMatrix(const JntArray &q, Vector* torque, NEWMAT::Matrix &mass)
{

  JntArray qdot(q.rows());
  JntArray qdotdot(q.rows());
  JntArray temp_1(q.rows()), temp_0(q.rows());

  SetToZero(qdot);
  SetToZero(qdotdot);

  for(unsigned int i=0; i < q.rows(); i++)
  {
    qdotdot(i) = 1;

    this->inverseDynamics->InverseDynamics(q,qdot,qdotdot,torque);
    for(unsigned int j=0; j < q.rows(); j++)
      temp_1(j) = torque[j][2];

    qdotdot(i) = 0;

    this->inverseDynamics->InverseDynamics(q,qdot,qdotdot,torque);
    for(unsigned int j=0; j< q.rows(); j++)
    {
      temp_0(j) = torque[j][2];
      mass(i+1,j+1) = temp_1(j)-temp_0(j);
    }
  }
}

void SerialChain::computeChristoffelSymbols(const JntArray &q, Vector* torque, NEWMAT::Matrix &christoffel)
{
  double scale = 1.0;
  int row_start = 0;
  JntArray qdot(q.rows());
  JntArray qdotdot(q.rows());
  JntArray temp_1(q.rows()), temp_0(q.rows());

  SetToZero(qdot);
  SetToZero(qdotdot);

  this->inverseDynamics->InverseDynamics(q,qdot,qdotdot,torque);

  for(unsigned int i=0; i < q.rows(); i++)
    temp_1(i) = torque[i][2];

  for(unsigned int j=0; j < q.rows(); j++)
  {
    for(unsigned int k=0; k < q.rows(); k++)
    {
      SetToZero(qdot);
      qdot(j) = 1;
      qdot(k) = 1;
      this->inverseDynamics->InverseDynamics(q,qdot,qdotdot,torque);
      for(unsigned int i=0; i < q.rows(); i++)
      {
        row_start = i*q.rows();
        if(j == k)
          scale = 1.0;
        else
          scale = 0.5;
        christoffel(row_start+j+1,k+1) =  scale*(temp_1(i) - torque[i][2]);
      }
    }
  }
}

#endif

// returns a%b
double modulus_double(double a, double b)
{
  int quo = (int) (a/b);
  return a-b*quo;
}

double angle_within_mod180(double ang)
{
  double rem = modulus_double(ang, 2*M_PI);

  if (rem > M_PI)
    rem -= 2*M_PI;
  else if (rem < (-1*M_PI))
    rem += 2*M_PI;

  return rem;
}

void angle_within_mod180(JntArray &q, int nJnts)
{
  for(int i=0; i < nJnts; i++)
  {
    q(i) = angle_within_mod180(q(i));
  }	
}

