
#include "kdl_kinematics.h"

using namespace KDL;
using namespace PR2;
using namespace std;



int main( int argc, char** argv )
{
	PR2_kinematics pr2_kin;

	JntArray pr2_config = JntArray(pr2_kin.nJnts);
	pr2_config(0) = 0.1, pr2_config(1) = -1, pr2_config(2)=0.3, pr2_config(3)=0.3;
	pr2_config(4) = 0.2, pr2_config(5) = 0.5, pr2_config(6) = 0.0;
	cout<<"Config of the arm:"<<pr2_config<<endl;

	Frame f;
	if (pr2_kin.FK(pr2_config,f))
		cout<<"End effector transformation:"<<f<<endl;
	else
		cout<<"Could not compute Fwd Kin."<<endl;

	JntArray q_init = JntArray(pr2_kin.nJnts);
	q_init(0) = 0.1, q_init(1) = 0.0, q_init(2) = 0.0, q_init(3) = 0.0;
	q_init(4) = 0.0, q_init(5) = 0.0, q_init(6) = 0.0;

	JntArray q_out = JntArray(pr2_kin.nJnts);
	if (pr2_kin.IK(q_init, f, q_out) == true)
		cout<<"IK result:"<<q_out<<endl;
	else
		cout<<"Could not compute Inv Kin."<<endl;

	//------ checking that IK returned a valid soln -----
	Frame f_ik;
	if (pr2_kin.FK(q_out,f_ik))
		cout<<"End effector after IK:"<<f_ik<<endl;
	else
		cout<<"Could not compute Fwd Kin. (After IK)"<<endl;

}


/*

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
	
	pr2_config(0) = 0.1, pr2_config(1) = -1, pr2_config(2)=0.3, pr2_config(3)=0.3;
	pr2_config(4) = 0.2, pr2_config(5) = 0.5, pr2_config(6) = 0.0;
	cout<<"Config of the arm:"<<pr2_config<<endl;

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
*/



