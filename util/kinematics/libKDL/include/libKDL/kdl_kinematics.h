
#ifndef KDL_KINEMATICS_H
#define KDL_KINEMATICS_H

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/utilities/utility.h>

#include <stdio.h>
#include <iostream>
#include <math.h>

#include <pr2Core/pr2Core.h>

#define RTOD(r) ((r)*180/M_PI)
#define DTOR(d) ((d)*M_PI/180)

double modulus_double(double a, double b);
double angle_within_mod180(double ang);
void angle_within_mod180(KDL::JntArray &q, int nJnts);

class PR2_kinematics
{
	public:
		KDL::Chain *chain;
		KDL::ChainFkSolverPos_recursive *fk_pos_solver;
		KDL::ChainIkSolverVel_pinv *ik_vel_solver;
		KDL::ChainIkSolverPos_NR *ik_pos_solver;
		int nJnts;

	public:
		PR2_kinematics();
		bool FK(const KDL::JntArray &q, KDL::Frame &f);
		bool IK(const KDL::JntArray &q_init, const KDL::Frame &f, KDL::JntArray &q_out);

};

#endif


