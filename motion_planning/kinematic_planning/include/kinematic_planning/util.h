#ifndef KINEMATIC_PLANNING_UTILITIES
#define KINEMATIC_PLANNING_UTILITIES

#include <std_msgs/PR2Arm.h>
#include <std_msgs/KinematicState.h>
#include <cstdio>

namespace ros
{
    namespace kinematics
    {
	
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
	
	void printKinematicState(const std_msgs::KinematicState &state)
	{
	    for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	    {
		printf("%lf ", state.vals[i]);
	    }
	    printf("\n");	
	}
	
	
    }    
}

#endif

