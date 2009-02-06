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
 * rev_rne.cpp
 *
 *  Created on: Jan 31, 2009
 *      Author: tjhunter
 *
 *  Test units for the recursive newton-euler solver. The reference is the KDL implementation.
 */



#include <kdl/chain.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_newtoneuler.hpp>
#include <iostream>

#include <robot_kinematics/RevoluteChainRNESolver.h>
#include <robot_kinematics/kdl_chain_wrapper.h>
#include <robot_kinematics/serial_chain.h>

#include <gtest/gtest.h>
#include <newmat10/newmat.h>

using namespace KDL;
using namespace std;
using namespace robot_kinematics;


static const int REPEATS=5;
static const double precision=0.00001;


// Helper function
template<typename T>
inline KDL::JntArray toJntArray(const Eigen::MatrixBase<T> &v)
{
//  assert(v.cols()==1);
  const int n=v.rows();
  KDL::JntArray res(n);
  for(int i=0;i<n;++i)
    res(i)=v[i];
  return res;
}

Eigen::MatrixXd fromNM(const NEWMAT::Matrix &m)
{
	Eigen::MatrixXd res(m.Nrows(),m.Ncols());
	for(int i=0;i<m.Nrows();++i)
		for(int j=0;j<m.Ncols();++j)
			res(i,j)=m(i+1,j+1);
	return res;
}

// Helper function
Eigen::Map<Eigen::Vector3d> wrap(const KDL::Vector &v)
{
  return Eigen::Map<Eigen::Vector3d>(v.data);
}

SerialChain * createChain(const Chain &c)
{
	SerialChain *cs=new SerialChain();
	cs->chain=c;
	cs->finalize();
	return cs;
}


//TODO use KDL facilities
Frame fromDH(double a, double alpha, double d,double theta)
{
  const double c_a=std::cos(alpha);
  const double s_a=std::sin(alpha);
  const double c_t=std::cos(theta);
  const double s_t=std::sin(theta);

  Rotation r=Rotation(c_t,      s_t,      0,
                      -s_t*c_a, c_t*c_a,  s_a,
                      s_t*s_a,  -c_t*s_a, c_a);
  Vector v=Vector(a*c_t,a*s_t,d);
  return Frame(r,v);
}

Segment segFromDH(double a, double alpha, double d, double theta, double m, const Eigen::Matrix3d & I, const Eigen::Vector3d & cm)
{
  Segment s=Segment(Joint(Joint::RotZ),
                    fromDH(a,alpha,d,theta),
                    Inertia(m,Vector(cm(0), cm(1), cm(2)),I(0,0),I(1,1),I(2,2),I(0,1),I(0,2),I(1,2)));
  return s;
}

Chain randomChain(int length)
{
	srandom(100);
	Chain chain=Chain();
	for(int i=0;i<length;++i)
	{
		const Eigen::Matrix3d I_=Eigen::Matrix3d::Random()+Eigen::Matrix3d::Identity();
		Segment s=segFromDH(drand48(),drand48(),drand48(),drand48(),1,I_.transpose()*I_,Eigen::Vector3d::Random());
		chain.addSegment(s);
	}
	return chain;
}

//Only the torque can be compareed with the current API of the KDL solver
void compare(const KDL::Chain &chain, double precision)
{
  KDLChainWrapper wrapper(chain);
  const int n=wrapper.segments();
  RevoluteChainRNESolver<KDLChainWrapper> solver(wrapper);
  KDLChainWrapper::Parameters q(n,1), qdot(n,1), qdotdot(n,1);

  q.setRandom();
  qdot.setRandom();
  qdotdot.setRandom();

  Eigen::VectorXd accel(n+n*n+1,1);

  solver.compute(q);
  solver.toAccelVector(qdot,qdotdot,accel);

  KDL::Vector * vec = new KDL::Vector[n+1];
  ChainIdSolver_NE * idsolver=new ChainIdSolver_NE(chain);
  idsolver->InverseDynamics(toJntArray(q),toJntArray(qdot),toJntArray(qdotdot),vec);

//  cout<<"Compare\n";
//  cout<<"omega \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.omega(i)*qdot).transpose()<<endl;
//  cout<<"alpha \n";
  for(int i=0;i<n;++i)
    EXPECT_TRUE((solver.torques(i)*accel-wrap(vec[i])).cwise().abs().maxCoeff()<precision);
//  cout<<"a_c \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.a_c(i)*accel).transpose()<<endl;
//  cout<<"forces \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.forces(i)*accel).transpose()<<endl;
//  cout<<"torques \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.torques(i)*accel).transpose()<<endl;
//
//  cout<<"omega KDL\n";
//  for(int i=1;i<n+1;++i)
//    cout<<wrap(idsolver->omega[i]).transpose()<<endl;
//  cout<<"alpha KDL\n";
//  for(int i=1;i<n+1;++i)
//    cout<<wrap(idsolver->alpha[i]).transpose()<<endl;
//  cout<<"a_c KDL\n";
//  for(int i=1;i<n+1;++i)
//    cout<<wrap(idsolver->a_c[i]).transpose()<<endl;
//  cout<<"forces KDL\n";
//  for(int i=0;i<n;++i)
//    cout<<wrap(idsolver->forces[i]).transpose()<<endl;
//  cout<<"torque KDL\n";
//  for(int i=0;i<n;++i)
//    cout<<wrap(vec[i]).transpose()<<endl;
//
//  cout<<"Diff\n";
//  cout<<"omega \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.omega(i)*qdot).transpose()-wrap(idsolver->omega[i+1]).transpose()<<endl;
//  cout<<"alpha \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.alpha(i)*accel).transpose()-wrap(idsolver->alpha[i+1]).transpose()<<endl;
//  cout<<"a_c \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.a_c(i)*accel).transpose()-wrap(idsolver->a_c[i+1]).transpose()<<endl;
//  cout<<"forces \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.forces(i)*accel).transpose()-wrap(idsolver->forces[i]).transpose()<<endl;
//  cout<<"torques \n";
//  for(int i=0;i<n;++i)
//    cout<<(solver.torques(i)*accel).transpose()-wrap(vec[i]).transpose()<<endl;
}

TEST(FAST_RNE,Empty)
{
	//Test for a nasty crash
	KDLChainWrapper wrapper;
	RevoluteChainRNESolver<KDLChainWrapper> solver;
}

TEST(FAST_RNE,CompareKDLRandom1)
{
	const Chain c=randomChain(1);
	for(int i=0;i<REPEATS;++i)
		compare(c,precision);
}

TEST(FAST_RNE,CompareKDLRandom2)
{
	const Chain c=randomChain(2);
	for(int i=0;i<REPEATS;++i)
		compare(c,precision);
}

TEST(FAST_RNE,CompareKDLRandom4)
{
	const Chain c=randomChain(4);
	for(int i=0;i<REPEATS;++i)
		compare(c,precision);
}

TEST(FAST_RNE,CompareKDLRandom6)
{
	const Chain c=randomChain(6);
	for(int i=0;i<REPEATS;++i)
		compare(c,precision);
}

TEST(FAST_RNE,CompareKDLRandom7)
{
	const Chain c=randomChain(7);
	for(int i=0;i<REPEATS;++i)
		compare(c,precision);
}

TEST(FAST_RNE,CompareKDLRandom10)
{
	const Chain c=randomChain(10);
	for(int i=0;i<REPEATS;++i)
		compare(c,precision);
}

TEST(FAST_RNE,CompareKDLRandom20)
{
	const Chain c=randomChain(20);
	for(int i=0;i<REPEATS+1;++i)
		compare(c,precision);
}


//--------- Comparison of the dynamics elements  ---------


void checkInertia(SerialChain *chain, double precision)
{
	  KDLChainWrapper wrapper(chain->chain);
	  const int n=wrapper.segments();
	  RevoluteChainRNESolver<KDLChainWrapper> solver(wrapper);
	  KDLChainWrapper::Parameters q=KDLChainWrapper::Parameters::Random(n,1);

	  solver.compute(q);

	  NEWMAT::Matrix mass(n,n);
	  Vector *torque=new Vector[n+1];
	  chain->computeMassMatrix(toJntArray(q),torque,mass);
	  delete[] torque;
	  const bool res=(fromNM(mass)-solver.inertia()).cwise().abs().maxCoeff()<precision;
	  EXPECT_TRUE(res);
}

//DISABLED FOR NOW
//TODO: check computations in serial_chain
void checkGravity(SerialChain *chain, double precision)
{
	  KDLChainWrapper wrapper(chain->chain);
	  const int n=wrapper.segments();
	  RevoluteChainRNESolver<KDLChainWrapper> solver(wrapper);
	  KDLChainWrapper::Parameters q=KDLChainWrapper::Parameters::Random(n,1);

	  solver.compute(q);

	  Eigen::VectorXd gravity(n,1);
	  Vector *torque=new Vector[n+1];
	  chain->computeGravityTerms(toJntArray(q),torque);

	  for(int i=0;i<n;++i)
		  gravity(i)=torque[i][2]; //Using the assumption that the joint axis is the Z axis

	  delete[] torque;
	  const bool res=(gravity-solver.gravity()).cwise().abs().maxCoeff()<precision;
	  EXPECT_TRUE(res);
}

//FIXME
//CHECK the order
void checkCristoffel(SerialChain *chain, double precision)
{
	  KDLChainWrapper wrapper(chain->chain);
	  const int n=wrapper.segments();
	  RevoluteChainRNESolver<KDLChainWrapper> solver(wrapper);
	  KDLChainWrapper::Parameters q=KDLChainWrapper::Parameters::Random(n,1);

	  solver.compute(q);
//	  cout<<solver.cSymbols()<<endl;

	  NEWMAT::Matrix cSymbols(n*n,n);
	  Vector *torque=new Vector[n+1];
	  chain->computeChristoffelSymbols(toJntArray(q),torque,cSymbols);
	  delete[] torque;
//	  cout<<fromNM(cSymbols)<<endl<<endl;
	  //WRONG PROBABLY
	  const bool res=(fromNM(cSymbols).transpose()-solver.cSymbols()).cwise().abs().maxCoeff()<precision;
	  EXPECT_TRUE(res);
}


TEST(FAST_RNE,CompareGravity2)
{
	SerialChain * c=createChain(randomChain(2));
	for(int i=0;i<REPEATS+1;++i)
		checkGravity(c,precision);
	delete c;
}

TEST(FAST_RNE,CompareGravity7)
{
	SerialChain * c=createChain(randomChain(7));
	for(int i=0;i<REPEATS+1;++i)
		checkGravity(c,precision);
	delete c;
}

TEST(FAST_RNE,CompareGravity20)
{
	SerialChain * c=createChain(randomChain(20));
	for(int i=0;i<REPEATS+1;++i)
		checkGravity(c,precision);
	delete c;
}

TEST(FAST_RNE,CompareInertia2)
{
	SerialChain * c=createChain(randomChain(2));
	for(int i=0;i<REPEATS+1;++i)
		checkInertia(c,precision);
	delete c;
}

TEST(FAST_RNE,CompareInertia7)
{
	SerialChain * c=createChain(randomChain(7));
	for(int i=0;i<REPEATS+1;++i)
		checkInertia(c,precision);
	delete c;
}

TEST(FAST_RNE,CompareInertia20)
{
	SerialChain * c=createChain(randomChain(20));
	for(int i=0;i<REPEATS+1;++i)
		checkInertia(c,precision);
	delete c;
}

//TEST(FAST_RNE,CompareCristoffel2)
//{
//	SerialChain * c=createChain(randomChain(2));
//	for(int i=0;i<REPEATS+1;++i)
//		checkCristoffel(c,precision);
//	delete c;
//}
//
//
//TEST(FAST_RNE,CompareCristoffel7)
//{
//	SerialChain * c=createChain(randomChain(7));
//	for(int i=0;i<REPEATS+1;++i)
//		checkCristoffel(c,precision);
//	delete c;
//}
//
//
//TEST(FAST_RNE,CompareCristoffel20)
//{
//	SerialChain * c=createChain(randomChain(20));
//	for(int i=0;i<REPEATS+1;++i)
//		checkCristoffel(c,precision);
//	delete c;
//}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
