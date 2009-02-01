/*
 * rne_speed.cpp
 *
 *  Created on: Jan 31, 2009
 *      Author: Tim Hunter <tjhunter@willowgarage.com>
 */

/**
 * Some performance tests
 */


#include <kdl/chain.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_newtoneuler.hpp>
#include <iostream>
#include <robot_kinematics/RevoluteChainRNESolver.h>
#include <robot_kinematics/kdl_chain_wrapper.h>
#include <robot_kinematics/serial_chain.h>

using namespace KDL;
using namespace std;
using namespace Eigen;
using namespace robot_kinematics;


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
      Inertia(m,I(0,0),I(1,1),I(2,2),I(0,1),I(0,2),I(1,2)),
      Vector(cm(0),cm(1),cm(2)));
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








void speedNG_0(const KDL::Chain &chain, int iter)
{
  KDLChainWrapper wrapper(chain);
  const int n=wrapper.segments();
  RevoluteChainRNESolver<KDLChainWrapper> solver(wrapper);

  KDLChainWrapper::Parameters q(n,1);

  for(int i=0;i<iter;++i)
  {
    q.setRandom();
    solver.compute(q);
  }
}

void speedNG(const KDL::Chain &chain, int iter)
{
  KDLChainWrapper wrapper(chain);
  const int n=wrapper.segments();
  RevoluteChainRNESolver<KDLChainWrapper> solver(wrapper);

  KDLChainWrapper::Parameters q(n,1), qdot(n,1), qdotdot(n,1);
  q.setRandom();
  solver.compute(q);

  VectorXd accel(n+n*n+1,1);
  Eigen::MatrixXd torques(n,n);

  for(int i=0;i<iter;++i)
  {
    qdot.setRandom();
    qdotdot.setRandom();
    solver.toAccelVector(qdot,qdotdot,accel);
    for(int i=0;i<n;++i)
      torques.row(i)=(solver.torques(i)*accel).transpose();
  }
}

void speedRNE_KDL(const KDL::Chain &chain, int iter)
{
  const int n=chain.getNrOfSegments();
  KDLChainWrapper::Parameters q(n,1), qdot(n,1), qdotdot(n,1);
  KDL::Vector * vec = new KDL::Vector[n+1];
  ChainIdSolver_NE idsolver=ChainIdSolver_NE(chain);
  for(int i=0;i<iter;++i)
  {
    q.setRandom();
    qdot.setRandom();
    qdotdot.setRandom();
    idsolver.InverseDynamics(toJntArray(q),toJntArray(qdot),toJntArray(qdotdot),vec);
  }
  delete [] vec;
}

void speedDynamicsKDL(SerialChain *chain, int iter)
{
	  KDLChainWrapper wrapper(chain->chain);
	  const int n=wrapper.segments();
	  RevoluteChainRNESolver<KDLChainWrapper> solver(wrapper);
	  KDLChainWrapper::Parameters q=KDLChainWrapper::Parameters::Random(n,1);

	  NEWMAT::Matrix mass(n,n);
	  NEWMAT::Matrix cSymbols(n*n,n);
	  Eigen::VectorXd gravity(n,1);
	  Vector *torque=new Vector[n+1];
	  for(int i=0;i<iter;++i)
	  {
		  chain->computeMassMatrix(toJntArray(q),torque,mass);
		  chain->computeChristoffelSymbols(toJntArray(q),torque,cSymbols);
		  chain->computeGravityTerms(toJntArray(q),torque);
		  q.setRandom();
	  }
	  delete[] torque;
}

int main(int argc, char **argv)
{
	int n=20000;
	SerialChain * c=createChain(randomChain(7));
//	speedDynamicsKDL(c,n);
	speedNG_0(c->chain,n);
//	speedRNE_KDL(c->chain,n);
}




