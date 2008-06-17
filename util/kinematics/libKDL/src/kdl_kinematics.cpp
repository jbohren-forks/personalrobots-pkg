
#include "libKDL/kdl_kinematics.h"

using namespace KDL;
using namespace PR2;
using namespace std;


PR2_kinematics::PR2_kinematics()
{
	//---------- create the chain for the PR2 ----------
	Chain pr2_chain;
	pr2_chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(ARM_PAN_SHOULDER_PITCH_OFFSET.x,0.0,0.0))));
	pr2_chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(ARM_SHOULDER_PITCH_ROLL_OFFSET.x,0.0,0.0))));
	pr2_chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.x,0.0,0.0))));
	pr2_chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(ELBOW_PITCH_ELBOW_ROLL_OFFSET.x,0.0,0.0))));
	pr2_chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(ELBOW_ROLL_WRIST_PITCH_OFFSET.x,0.0,0.0))));
	pr2_chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(WRIST_PITCH_WRIST_ROLL_OFFSET.x,0.0,0.0))));
	pr2_chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(WRIST_ROLL_GRIPPER_OFFSET.x,0.0,0.0))));

	this->chain = &pr2_chain;
	this->nJnts = pr2_chain.getNrOfJoints();

	//------ Forward Position Kinematics --------------
	this->fk_pos_solver = new ChainFkSolverPos_recursive(pr2_chain);

	//------- IK
	this->ik_vel_solver = new ChainIkSolverVel_pinv(pr2_chain);
	this->ik_pos_solver = new ChainIkSolverPos_NR(*this->chain, *this->fk_pos_solver, *this->ik_vel_solver);
}


/*
 * f - end effector frame (result of the fwd kinematics)
 * returns True if ok, False if error.
 */
bool PR2_kinematics::FK(const JntArray &q, Frame &f)
{
	if (this->fk_pos_solver->JntToCart(q,f) >= 0)
		return true;
	else
		return false;
}

bool PR2_kinematics::IK(const JntArray &q_init, const Frame &f, JntArray &q_out)
{
	if (this->ik_pos_solver->CartToJnt(q_init,f,q_out) >= 0)
	{
		angle_within_mod180(q_out, this->nJnts);
		return true;
	}
	else
		return false;
}


// returns a%b
double modulus_double(double a, double b)
{
	int quo = a/b;
	return a-b*quo;
}

double angle_within_mod180(double ang)
{
	double rem = modulus_double(ang, 2*M_PI);

	if (rem>M_PI)
		rem -= 2*M_PI;
	else if (rem<(-1*M_PI))
		rem += 2*M_PI;

	return rem;
}

void angle_within_mod180(JntArray &q, int nJnts)
{
	for(int i=0;i<nJnts;i++)
	{
		q(i) = angle_within_mod180(q(i));
		printf(".. %f ..", q(i));
	}	
}




