/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#ifndef KINEMATIC_PLANNING_UTILITIES
#define KINEMATIC_PLANNING_UTILITIES

#include <std_msgs/PR2Arm.h>
#include <std_msgs/KinematicState.h>
#include <cstdio>

namespace motion_planning
{
    
    /** \brief Data type identifying the model -- or part(s) of
	the robot -- the motion planner should plan for **/
    enum KINEMATIC_MODEL
    {
	PR2_LEFT_ARM,
	PR2_RIGHT_ARM,
	PR2_BOTH_ARMS,
	PR2_BASE,
	PR2_BASE_LEFT_ARM,
	PR2_BASE_RIGHT_ARM,
	PR2_BASE_BOTH_ARMS,
	PR2_BODY_LEFT_ARM,
	PR2_BODY_RIGHT_ARM,
	PR2_BODY_BOTH_ARMS,
	PR2_BODY_BASE_LEFT_ARM,
	PR2_BODY_BASE_RIGHT_ARM,
	PR2_BODY_BASE_BOTH_ARMS,
	MAX_KINEMATIC_MODELS
    };
    
    /** \brief Convert a PR2Arm message into a KinematicState message **/
    void convertPR2ArmKinematicState(const std_msgs::PR2Arm &arm, std_msgs::KinematicState &state)
    {
	state.set_vals_size(9);
	state.vals[0] = arm.turretAngle;
	state.vals[1] = arm.shoulderLiftAngle;
	state.vals[2] = arm.upperarmRollAngle;
	state.vals[3] = arm.elbowAngle;
	state.vals[4] = arm.forearmRollAngle;
	state.vals[5] = arm.wristPitchAngle;
	state.vals[6] = arm.wristRollAngle;
	state.vals[7] = arm.gripperForceCmd;
	state.vals[8] = arm.gripperGapCmd;
    }
    
    /** \brief Convert a KinematicState message into a PR2Arm message **/
    void convertKinematicStatePR2Arm(const std_msgs::KinematicState &state, std_msgs::PR2Arm& arm)
    {
	arm.turretAngle       = state.vals[0];
	arm.shoulderLiftAngle = state.vals[1];
	arm.upperarmRollAngle = state.vals[2];
	arm.elbowAngle        = state.vals[3];
	arm.forearmRollAngle  = state.vals[4];
	arm.wristPitchAngle   = state.vals[5];
	arm.wristRollAngle    = state.vals[6];
	arm.gripperForceCmd   = state.vals[7]; 
	arm.gripperGapCmd     = state.vals[8];
    }
    
    /** \brief Print the content of a KinematicState message to stdout **/
    void printKinematicState(const std_msgs::KinematicState &state)
    {
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	{
	    printf("%lf ", state.vals[i]);
	}
	printf("\n");	
    }
    
    
}    


#endif

