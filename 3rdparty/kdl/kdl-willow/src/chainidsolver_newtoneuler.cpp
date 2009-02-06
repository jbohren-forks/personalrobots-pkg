
#include "chainidsolver_newtoneuler.hpp"


namespace KDL
{

	ChainIdSolver_NE::ChainIdSolver_NE(const Chain &_chain):
		chain(_chain)
	{
		forces = new Vector[chain.getNrOfJoints()+1];
	}

	int ChainIdSolver_NE::InverseDynamics(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, Vector* torque)
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
		Rotation _iRi_minus_one = Rotation::Identity();
		Frame T_tip = Frame::Identity();

		Vector z_i_minus_one, b_i, r, r_cm;
		Segment link_i;

		// logical joint numbering begins from 1 and index from 0.

		// computing angular velocities and accelerations for each link.
		for(int i=1;i<=nJoints;i++)
		{
			link_i = chain.getSegment(i-1);
			z_i_minus_one = link_i.getJoint().JointAxis();

			r_cm = link_i.getCM();
			_iRi_minus_one = T_tip.M.Inverse();

			b_i = z_i_minus_one;
			T_tip = link_i.getFrameToTip();
			r = T_tip.p; // I want vector in body coordinates.

			_iTi_minus_one = link_i.pose(q(i-1)).Inverse();
			_iT0 = _iTi_minus_one*_iT0;
			_iRi_minus_one = T_tip.M*_iTi_minus_one.M * _iRi_minus_one;

			omega[i] = _iRi_minus_one*omega[i-1] + b_i*q_dot(i-1);
			alpha[i] = _iRi_minus_one*alpha[i-1] + b_i*q_dotdot(i-1) + omega[i]*b_i*q_dot(i-1);
			a_c[i] = _iRi_minus_one*a_e + alpha[i]*r_cm + omega[i]*(omega[i]*r_cm);
			a_e = _iRi_minus_one*a_e + alpha[i]*r + omega[i]*(omega[i]*r);
		}

		// now for joint forces and torques.
		Vector g(0.,0.,-9.8);
		Rotation rot, rot_g;
		Frame T;

		Inertia gen_mass;
		InertiaMatrix I;
		double m;

		Vector *f = forces;
		Vector *t = torque;
		f[nJoints] = Vector::Zero();
		t[nJoints] = Vector::Zero();
		// _iT0 is now _nT0
		rot_g = _iT0.M;
		g = rot_g*g;

		Vector r_iplusone_c; // Vector from o_{i+1} to c_i
		rot = Rotation::Identity(); // should be rotation matrix for end-effector forces to coord frame of last link.

		for(int i=nJoints-1;i>=0;i--)
		{
			link_i = chain.getSegment(i);
			T_tip = link_i.getFrameToTip();
			rot = T_tip.M * rot;
			r_cm = link_i.getCM();

			r = T_tip.p; // I want vector in body coordinates.
			r_iplusone_c = r_cm-r;

			gen_mass = link_i.getInertia();

			I = gen_mass.I;
			m = gen_mass.m;

			T = link_i.pose(q(i));

			g = rot*g;

			f[i] = rot*f[i+1] + m*a_c[i+1] - m*g;
			t[i] = rot*t[i+1] - f[i]*r_cm + (rot*f[i+1])*r_iplusone_c+I*alpha[i+1]
				+ omega[i+1]*(I*omega[i+1]);

			rot = T.M*T_tip.M.Inverse();
		}

		delete [] omega;
		delete [] alpha;
		delete [] a_c;

		return 0;
	}

	ChainIdSolver_NE::~ChainIdSolver_NE()
	{
	}

}

