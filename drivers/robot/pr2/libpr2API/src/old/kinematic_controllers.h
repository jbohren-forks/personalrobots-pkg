#ifndef KINEMATIC_CONTROLLERS_H
#define KINEMATIC_CONTROLLERS_H

#include <libpr2API/pr2API.h>

class kinematic_controllers
{
	public:
		PR2::PR2Robot* myPR2;

	public:
		kinematic_controllers();

		// linear interpolation between current and desired position.
		// p,r -- desired pose, class stores current joint angles.
		void go(KDL::Vector p, KDL::Rotation r, double step_size);
		void init();


};



#endif

