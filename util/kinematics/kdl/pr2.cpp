#include <pr2Core/pr2Core.h>
#include <math.h>

double a1x(0.1),a2x(0.5),a3x(0.59085),a4x(0.81455),a5x(0.81455);
using namespace PR2;
using namespace std;


#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <stdio.h>
#include <iostream>
#include <math.h>

#define RTOD(r) ((r)*180/M_PI)
#define DTOR(d) ((d)*M_PI/180)

using namespace KDL;
using namespace std;


int main( int argc, char** argv )
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

	int nJts = pr2_chain.getNrOfJoints();
	JntArray pr2_config = JntArray(nJts);

	//------ Forward Position Kinematics --------------
	ChainFkSolverPos_recursive fk_pr2 = ChainFkSolverPos_recursive(pr2_chain);
	Frame pr2_gripper;
	if (fk_pr2.JntToCart(pr2_config,pr2_gripper)>=0)
	{
		printf("End effector transformation matrix:\n");
		cout<<pr2_gripper<<endl;
	}
	else
		printf("Error: could not calculate forward kinematics\n");

	//------- IK
	ChainIkSolverVel_pinv ik_pr2_v(pr2_chain);
	ChainIkSolverPos_NR ik_pr2_p(pr2_chain, fk_pr2, ik_pr2_v);

	JntArray q_init(nJts);
	JntArray q(nJts);

	printf("----------------------------\n");
	if (ik_pr2_p.CartToJnt(q_init,pr2_gripper,q)>=0)
		cout<<"IK result:"<<q<<endl;
	else
		printf("Error: could not calculate inverse kinematics\n");
	printf("----------------------------\n");

	//------ checking that IK returned a valid soln -----
	printf("Using config returned by IK\n");
	if (fk_pr2.JntToCart(q,pr2_gripper)>=0)
	{
		printf("End effector transformation matrix:\n");
		cout<<pr2_gripper<<endl;
	}
	else
		printf("Error: could not calculate forward kinematics\n");

}



