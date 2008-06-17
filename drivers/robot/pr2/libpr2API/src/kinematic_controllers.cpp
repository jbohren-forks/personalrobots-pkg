/*
 Higher level controllers for the arm.
 I don't want to add to libpr2API and proper place for this code is yet to
 be decided.
 

*/

#include "kinematic_controllers.h"

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

}




