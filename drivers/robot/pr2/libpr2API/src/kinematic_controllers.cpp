/*
 Higher level controllers for the arm.
 I don't want to add to libpr2API and proper place for this code is yet to
 be decided.
 

*/

#include "kinematic_controllers.h"
#include <kdl/rotational_interpolation_sa.hpp>

using namespace KDL;

kinematic_controllers::kinematic_controllers()
{
	this->myPR2 = new PR2::PR2Robot();
}

void kinematic_controllers::init()
{
	this->myPR2->InitializeRobot();
}

void kinematic_controllers::go(KDL::Vector p, KDL::Rotation r, double step_size)
{
	JntArray q = JntArray(this->myPR2->pr2_kin.nJnts);
	this->myPR2->GetArmJointPositionCmd(PR2::PR2_RIGHT_ARM, q);
	cout<<"current joint angles:"<<q<<endl;

	Frame f;
	this->myPR2->pr2_kin.FK(q,f);
	Vector start = f.p;
	Vector move = p-start;
	double dist = move.Norm();
	move = move/dist;
//cout << "Start:"<<start<<", End:"<<p<<", dist: "<<dist<<", direction: "<<move<<endl;

	RotationalInterpolation_SingleAxis rotInterpolater;
	rotInterpolater.SetStartEnd(f.M, r);
	double total_angle = rotInterpolater.Angle();
//	printf("Angle: %f\n", rotInterpolater.Angle());

	Vector target;
	int nSteps = (int)(dist/step_size);
	double angle_step = total_angle/nSteps;
	for(int i=0;i<nSteps;i++)
	{
		f.p = start+(i+1)*move*step_size;
		f.M = rotInterpolater.Pos(angle_step*(i+1));
		this->myPR2->SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f,q,q);
	}

	f.p = p;
	f.M = r;
	this->myPR2->SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f,q,q);

}



