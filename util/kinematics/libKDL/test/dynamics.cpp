

#include "libKDL/kdl_kinematics.h"
#include "libKDL/dynamics.h"

#include <sys/time.h>
#include <unistd.h>


using namespace KDL;
using namespace PR2;
using namespace std;

//----------- inertiamatrix.cpp -------------------
std::ostream& operator << (std::ostream& os,const InertiaMatrix& I)
{
	os << "[";
	for (int i=0;i<=2;i++)
	{
		os << std::setw(KDL_FRAME_WIDTH) << I.data[2*i+0] << "," <<
			std::setw(KDL_FRAME_WIDTH) << I.data[2*i+1] << "," <<
			std::setw(KDL_FRAME_WIDTH) << I.data[2*i+2];
		if (i<2)
			os << ";"<< std::endl << " ";
		else
			os << "]";
	}
	return os;
}


InertiaMatrix::InertiaMatrix(double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
{
	data[0] = Ixx, data[4] = Iyy, data[8] = Izz;
	data[1] = data[3] = Ixy;
	data[2] = data[6] = Ixz;
	data[5] = data[7] = Iyz;
}

InertiaMatrix::~InertiaMatrix()
{
}

Vector InertiaMatrix::operator*(const Vector& v) const {
	// Complexity : 9M+6A
	return Vector(
			data[0]*v.data[0] + data[1]*v.data[1] + data[2]*v.data[2],
			data[3]*v.data[0] + data[4]*v.data[1] + data[5]*v.data[2],
			data[6]*v.data[0] + data[7]*v.data[1] + data[8]*v.data[2]
			);
}
//--------------------------------------------

//---------- inertia.cpp --------------------
Inertia_advait::Inertia_advait(double m,double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
{
	this->m = m;
	this->I = InertiaMatrix(Ixx,Iyy,Izz,Ixy,Ixz,Iyz);
}

Inertia_advait::~Inertia_advait()
{
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
//f,t arrays of size n+1, last element forced to zero in this function.
int newtonEulerSolver(const Chain &chain, const JntArray &q, const JntArray &q_dot, 
											const JntArray &q_dotdot, const Inertia_advait *gen_mass,
											Vector *f, Vector *t)
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
	Frame _iTi_minus_one, _i_minus_oneTi;
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
		r = link_i.pose(0).p; // I want vector in body coordinates.
		r_c = r/2; // Center of mass is currently fixed, need a clean solution for this.

		omega[i] = _iRi_minus_one*omega[i-1] + b_i*q_dot(i-1);
		alpha[i] = _iRi_minus_one*alpha[i-1] + b_i*q_dotdot(i-1) + omega[i]*b_i*q_dot(i-1);
		a_c[i] = _iRi_minus_one*a_e + alpha[i]*r_c + omega[i]*(omega[i]*r_c);
		a_e = _iRi_minus_one*a_e + alpha[i]*r + omega[i]*(omega[i]*r);

		cout<<"omega["<<i<<"]: "<<omega[i]<<endl<<"alpha["<<i<<"]: "<<alpha[i]<<endl;
		cout<<"a_c["<<i<<"]: "<<a_c[i]<<endl;
	}

	// now for joint forces and torques.
	Vector g(0.,0.,-9.8);
	Rotation rot, rot_g;
	Frame T;

	InertiaMatrix I;
	double m;

	f[nJoints] = Vector::Zero();
	t[nJoints] = Vector::Zero();
	// _iT0 is now _nT0
	rot_g = _iT0.M;

	Vector r_iplusone_c; // Vector from o_{i+1} to c_i

	rot = Rotation::Identity(); // should be rotation matrix for end-effector forces to coord frame of last link.

	for(int i=nJoints-1;i>=0;i--)
	{
		link_i = chain.getSegment(i);
		T = link_i.pose(q(i));
		r = link_i.pose(0).p; // I want vector in body coordinates.
		r_c = r/2; // Center of mass is currently fixed, need a clean solution for this.
		r_iplusone_c = -r/2; // temporary

		I = gen_mass[i].I;
		m = gen_mass[i].m;

//		printf("----------------------\n");
//		printf("i is %d\n", i);
//
//		if(i==1)
//		{
//			cout<<"I: "<<I<<endl;
//			cout<<"alpha: "<<alpha[i+1]<<endl;
//			cout<<"I*alpha: "<<I*alpha[i+1]<<endl;
//		}

		f[i] = rot*f[i+1] + m*a_c[i+1] - m*(rot_g*g);
		t[i] = rot*t[i+1] - f[i]*r_c + (rot*f[i+1])*r_iplusone_c+I*alpha[i+1]
					 + omega[i+1]*(I*omega[i+1]);

//		printf("i: %d, force:",i);
//		cout<<f[i]<<", torque:"<<t[i]<<endl;
//		cout<<"g_i:"<<rot_g*g<<endl;
//		cout<<"r_c:"<<r_c<<endl;

		rot = T.M; // rot: i+1 -> i
		rot_g = rot*rot_g;
	}

	return 0;
}

void checkOneLink_static()
{
//-------- one link ---------
	double theta = deg2rad*30;
	double mass = 3.;
	double l1 = 13.;

	Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,l1,0.0))));

	int n = chain.getNrOfJoints();
	JntArray q = JntArray(n);
	JntArray q_dot = JntArray(n);
	JntArray q_dotdot = JntArray(n);
	Vector *f = new Vector[n+1];
	Vector *t = new Vector[n+1];

//-- STATIC case --
	q(0) = theta;
	q_dot(0) = deg2rad*0.;
	q_dotdot(0) = deg2rad*0.;

	Inertia_advait gen_mass[1];
	gen_mass[0].m = mass;
	gen_mass[0].I.data[0] = 10.;

	newtonEulerSolver(chain, q, q_dot, q_dotdot, gen_mass,f,t);
	printf("====================================\n");
	cout<<"Expected Force (Math):"<<Vector(0,mass*9.8*sin(theta),mass*9.8*cos(theta))<<endl;
	cout<<"Calculated Force (Code):"<<f[0]<<endl;
	cout<<"Expected Torque (Math):"<<Vector(mass*9.8*cos(theta)*l1/2,0,0)<<endl;
	cout<<"Calculated Torque (Code):"<<t[0]<<endl;
	printf("====================================\n");
}


void planarTwoLink_solution(double l1, double l2, JntArray &q, JntArray &q_dot, JntArray &q_dotdot, Inertia_advait *gen_mass)
{
	double I1 = gen_mass[0].I.data[0];
	double I2 = gen_mass[1].I.data[0];
	double m1 = gen_mass[0].m, m2 = gen_mass[1].m;
	double q1 = q(0), q2 = q(1);
	double q1_dot = q_dot(0), q2_dot = q_dot(1);
	double q1_dotdot = q_dotdot(0), q2_dotdot = q_dotdot(1);
	double t1,t2;
	double l_c2 = l2/2, l_c1 = l1/2;
	double g = 9.8;

	t2 = I2*(q1_dotdot+q2_dotdot) + m2*l1*l_c2*sin(q2)*q1_dot*q1_dot +
			 m2*l1*l_c2*cos(q2)*q1_dotdot + 
			 m2*l_c2*l_c2*(q1_dotdot+q2_dotdot)+m2*l_c2*g*cos(q1+q2);
	t1 = t2 + m1*l_c1*l_c1*q1_dotdot + m1*l_c1*g*cos(q1) + m2*l1*g*cos(q1) + 
			 I1*q1_dotdot + m2*l1*l1*q1_dotdot -
			 m2*l1*l_c2*(q1_dot+q2_dot)*(q1_dot+q2_dot)*sin(q2) +
			 m2*l1*l_c2*(q1_dotdot+q2_dotdot)*cos(q2);

	cout<<"Expected joint torques, Spong Pg 282"<<endl;
	cout<<"t1: "<<t1<<endl<<"t2: "<<t2<<endl;
}

void checkTwoLinkPlanar()
{
//-------- one link ---------
	double q1 = deg2rad*23, q2 = deg2rad*19;
	double q1_dot = deg2rad*-71, q2_dot = deg2rad*-25;
	double q1_dotdot = deg2rad*-30, q2_dotdot = deg2rad*60;
	double m1 = 2., m2 = 5.;
	double I1 = 11., I2 = 8.;
	double l1 = 2.3, l2 = 1.6;

	Chain chain;
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,l1,0.0))));
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,l2,0.0))));

	int n = chain.getNrOfJoints();
	JntArray q = JntArray(n);
	JntArray q_dot = JntArray(n);
	JntArray q_dotdot = JntArray(n);
	Vector *f = new Vector[n+1];
	Vector *t = new Vector[n+1];

//-- STATIC case --
	q(0) = q1, q(1) = q2;
	q_dot(0) = q1_dot, q_dot(1) = q2_dot;
	q_dotdot(0) = q1_dotdot, q_dotdot(1) = q2_dotdot;

	Inertia_advait gen_mass[2];
	gen_mass[0].m = m1, gen_mass[1].m = m2;
	gen_mass[0].I.data[0] = I1, gen_mass[1].I.data[0] = I2;

	newtonEulerSolver(chain, q, q_dot, q_dotdot, gen_mass,f,t);
	printf("====================================\n");
	planarTwoLink_solution(l1, l2, q, q_dot, q_dotdot, gen_mass);
	cout<<"Calculated Force 1 (Code):"<<f[0]<<endl;
	cout<<"Calculated Force 2 (Code):"<<f[1]<<endl;
	cout<<"Calculated Torque 1 (Code):"<<t[0]<<endl;
	cout<<"Calculated Torque 2 (Code):"<<t[1]<<endl;
	printf("====================================\n");
}


int main( int argc, char** argv )
{
//	checkOneLink_static();
	checkTwoLinkPlanar();
}





