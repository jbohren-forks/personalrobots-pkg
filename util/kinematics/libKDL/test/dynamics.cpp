

#include "libKDL/kdl_kinematics.h"
#include "libKDL/dynamics.h"

#include <sys/time.h>
#include <unistd.h>


using namespace KDL;
using namespace PR2;
using namespace std;

//----------- inertia.cpp -------------------

Inertia_advait::Inertia_advait(double m,double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
{
	ublas::matrix<double> Mat(3,3);
	I = &Mat;
	(*I)(0,0) = Ixx;
//	, *I(1,1) = Iyy, *I(2,2) = Izz;
//	*I(1,0) = *I(0,1) = Ixy;
//	*I(2,0) = *I(0,2) = Ixz;
//	*I(1,2) = *I(2,1) = Iyz;
}

//double Inertia_advait::getMass() const
//{
//	return m;
//}
//
//ublas::matrix<double> Inertia_advait::getInertiaMatrix() const
//{
//	return I;
//}

Inertia_advait::~Inertia_advait()
{
}


Vector operator * (ublas::matrix<double> M_boost, Vector v)
{
	ublas::vector<double> v_boost(3);
	for(unsigned int i=0;i<3;i++)
		v_boost(i) = v.data[i];

	v_boost = prod(M_boost, v_boost);
	for(unsigned int i=0;i<3;i++)
		v.data[i] = v_boost(i);

	return v;
}


//--------------------------------------------

/*
 * Conventions for joint and link numbering - Spong page 74.
 *
 */

Vector jointAxis(const Joint &jnt)
{
	Joint::JointType type = jnt.getType();
	switch(type)
	{
		case Joint::RotX:
			return Vector(1.,0.,0.);
		case Joint::RotY:
			return Vector(0.,1.,0.);
		case Joint::RotZ:
			return Vector(0.,0.,1.);
		default:
			printf("This joint type does not have an axis. Type:%d\n",type);
			printf("Exiting...\n");
			exit(0);
	}
}

//-- will later add parameters for forces/torques on the individual joints.
int newtonEulerSolver(const Chain &chain, const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Inertia_advait *gen_mass)
{
	//-- source: Robot Modeling and Control (Spong,Hutchinson,Vidyasagar). Page 279.
	int nJoints = q.rows();

	Vector *omega = new Vector[nJoints+1];
	Vector *alpha = new Vector[nJoints+1];
	Vector *a_c = new Vector[nJoints+1];

	omega[0] = Vector::Zero();
	alpha[0] = Vector::Zero();
	a_c[0] = Vector::Zero();

	Vector a_e = Vector::Zero();

	Frame _iT0 = Frame::Identity();
	Frame _iTi_minus_one;
	Rotation _iRi_minus_one;

	Vector z_i_minus_one, b_i, r,r_c;
	Segment link_i;

	// logical joint numbering begins from 1 and index from 0.

	// computing angular velocities and accelerations for each link.
	for(int i=1;i<=nJoints;i++)
	{
		link_i = chain.getSegment(i-1);
		z_i_minus_one = jointAxis(link_i.getJoint());

		_iTi_minus_one = link_i.pose(q(i-1)).Inverse();
		_iT0 = _iTi_minus_one*_iT0;
		b_i = z_i_minus_one;
		_iRi_minus_one = _iTi_minus_one.M;
		r = _iTi_minus_one.p;
		r_c = r/2; // Center of mass is currently fixed, need a clean solution for this.

		omega[i] = _iRi_minus_one*omega[i-1] + b_i*q_dot(i-1);
		alpha[i] = _iRi_minus_one*alpha[i-1] + b_i*q_dotdot(i-1) + omega[i]*b_i*q_dot(i-1);
		a_c[i] = _iRi_minus_one*a_e + alpha[i]*r_c + omega[i]*(omega[i]*r_c);
		a_e = _iRi_minus_one*a_e + alpha[i]*r + omega[i]*(omega[i]*r);
	}

	// now for joint forces and torques.
	Vector g(0.,0.,-9.8);
	Vector *f = new Vector[nJoints+1];
	Vector *t = new Vector[nJoints+1];
	Rotation rot, rot_g;
	Frame T;

	ublas::matrix <double> I(3,3);
	double m;

	f[nJoints] = Vector::Zero();
	t[nJoints] = Vector::Zero();
	// _iT0 is now _nT0
	rot_g = _iT0.M;

	Vector r_iplusone_c; // Vector from o_{i+1} to c_i

	for(int i=nJoints-1;i>=0;i--)
	{
		link_i = chain.getSegment(i);
		T = link_i.pose(q(i-1)).Inverse();
		rot = T.M; // rot: i+1 -> i
		r = T.p;
		r_c = r/2; // Center of mass is currently fixed, need a clean solution for this.
		r_iplusone_c = -r/2; // temporary

//		I = gen_mass[i].getInertiaMatrix();
//		m = gen_mass[i].getMass();
		I = *(gen_mass[i].I);
		m = gen_mass[i].m;

		f[i] = rot*f[i+1] + m*a_c[i+1] - m*(rot_g*g);
		t[i] = rot*t[i+1] - f[i]*r_c + (rot*f[i+1])*r_iplusone_c+I*alpha[i]
					 + omega[i]*(I*omega[i]);
		rot_g = rot*rot_g;
	}

	return 0;
}


Chain oneLink()
{
	double l1 = 1.;
	Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,l1,0.0))));
	return chain;
}


int main( int argc, char** argv )
{
	Chain chain = oneLink();

	int n = chain.getNrOfJoints();
	JntArray q = JntArray(n);
	JntArray q_dot = JntArray(n);
	JntArray q_dotdot = JntArray(n);

	//-------- one link ---------
	q(0) = deg2rad*0.;
	q_dot(0) = deg2rad*0.;
	q_dotdot(0) = deg2rad*0.;

	Inertia_advait gen_mass[1];
	gen_mass[0].m = 1.;

//	newtonEulerSolver(chain, q, q_dot, q_dotdot, gen_mass);
}



