/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Advait Jain

/**

  @mainpage

  @htmlinclude manifest.html

  @b libKDL has been @b deprecated by @b robot_kinematics. It served as a wrapper around KDL to make it easier to use FK and IK.

 **/




#include "libKDL/kdl_kinematics.h"

using namespace KDL;
using namespace PR2;
using namespace std;


PR2_kinematics::PR2_kinematics()
{
	//---------- create the chain for the PR2 ----------
	this->chain = new Chain();
	this->chain->addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(ARM_PAN_SHOULDER_PITCH_OFFSET.x,0.0,0.0))));
	this->chain->addSegment(Segment(Joint(Joint::RotY),Frame(Vector(ARM_SHOULDER_PITCH_ROLL_OFFSET.x,0.0,0.0))));
	this->chain->addSegment(Segment(Joint(Joint::RotX),Frame(Vector(ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.x,0.0,0.0))));
	this->chain->addSegment(Segment(Joint(Joint::RotY),Frame(Vector(ELBOW_PITCH_ELBOW_ROLL_OFFSET.x,0.0,0.0))));
	this->chain->addSegment(Segment(Joint(Joint::RotX),Frame(Vector(ELBOW_ROLL_WRIST_PITCH_OFFSET.x,0.0,0.0))));
	this->chain->addSegment(Segment(Joint(Joint::RotY),Frame(Vector(WRIST_PITCH_WRIST_ROLL_OFFSET.x,0.0,0.0))));
	this->chain->addSegment(Segment(Joint(Joint::RotX),Frame(Vector(WRIST_ROLL_GRIPPER_OFFSET.x,0.0,0.0))));

	this->nJnts = this->chain->getNrOfJoints();
	this->q_IK_guess = new JntArray(this->nJnts);
	this->q_IK_result = new JntArray(this->nJnts);

	//------ Forward Position Kinematics --------------
	this->fk_pos_solver = new ChainFkSolverPos_recursive(*this->chain);
	//------- IK
	this->ik_vel_solver = new ChainIkSolverVel_pinv(*this->chain);
	this->ik_pos_solver = new ChainIkSolverPos_NR(*this->chain, *this->fk_pos_solver, *this->ik_vel_solver);


	//---------- create a chain for the fore-arm camera -------------
	this->chain_forearm_camera = new Chain();
	this->chain_forearm_camera->addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(ARM_PAN_SHOULDER_PITCH_OFFSET.x,0.0,0.0))));
	this->chain_forearm_camera->addSegment(Segment(Joint(Joint::RotY),Frame(Vector(ARM_SHOULDER_PITCH_ROLL_OFFSET.x,0.0,0.0))));
	this->chain_forearm_camera->addSegment(Segment(Joint(Joint::RotX),Frame(Vector(ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.x,0.0,0.0))));
	this->chain_forearm_camera->addSegment(Segment(Joint(Joint::RotY),Frame(Vector(ELBOW_PITCH_ELBOW_ROLL_OFFSET.x,0.0,0.0))));
	this->chain_forearm_camera->addSegment(Segment(Joint(Joint::RotX),Frame(Rotation::RotY(-45*deg2rad),Vector(0.111825,0.0,0.02))));

	this->nJnts_forearm_camera = this->chain_forearm_camera->getNrOfJoints();
	//------ Forward Position Kinematics --------------
	this->fk_pos_solver_forearm_camera = new ChainFkSolverPos_recursive(*this->chain_forearm_camera);
	//------- IK
	this->ik_vel_solver_forearm_camera = new ChainIkSolverVel_pinv(*this->chain_forearm_camera);
	this->ik_pos_solver_forearm_camera = new ChainIkSolverPos_NR(*this->chain_forearm_camera, *this->fk_pos_solver_forearm_camera,
																															 *this->ik_vel_solver_forearm_camera);


}

PR2_kinematics::~PR2_kinematics()
{
	delete this->chain;
	delete this->fk_pos_solver;
	delete this->ik_vel_solver;
	delete this->ik_pos_solver;
	delete this->q_IK_guess;
	delete this->q_IK_result;

	delete this->chain_forearm_camera;
	delete this->fk_pos_solver_forearm_camera;
	delete this->ik_vel_solver_forearm_camera;
	delete this->ik_pos_solver_forearm_camera;
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

/*
 * We might want to get rid of this function.
 * Hasn't been removed for compatibility with ArmController.
 */
bool PR2_kinematics::IK(const JntArray &q_init, const Frame &f, JntArray &q_out)
{
	if (this->ik_pos_solver->CartToJnt(q_init,f,q_out) >= 0)
	{
		angle_within_mod180(q_out, this->nJnts);
		return true;
	}
	else
	{
		printf("[libKDL]<kdl_kinematics.cpp>IK failed.\n");
		return false;
	}
}

/*
 * uses internal state (q_IK_guess) as the seed for KDL's IK.
 * result is stored in q_IK_result
 */
bool PR2_kinematics::IK(const Frame &f)
{
	if (this->ik_pos_solver->CartToJnt(*this->q_IK_guess,f,*this->q_IK_result) >= 0)
	{
		angle_within_mod180(*this->q_IK_result, this->nJnts);
//		cout<<"[libKDL]<kdl_kinematics.cpp> IK result: "<<*this->q_IK_result<<endl;
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
//		printf(".. %f ..", q(i));
	}	
}




