
#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Misc.h>
#include <math.h>

using namespace PR2;

PR2Robot::PR2Robot()
{

};

PR2Robot::~PR2Robot(){};

PR2_ERROR_CODE PR2Robot::InitializeRobot()
{
  // initialize robot
  hw.Init();
  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::CalibrateRobot()
{
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::EnableArm(PR2_MODEL_ID id)
{
   switch(id)
   {
      case PR2_LEFT_ARM:
         this->hw.EnableModel(PR2_LEFT_ARM);
         break;
      case PR2_RIGHT_ARM:
         hw.EnableModel(PR2_RIGHT_ARM);
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::EnableGripper(PR2_MODEL_ID id)
{
   switch(id)
   {
      case PR2_LEFT_GRIPPER:
         EnableGripperLeft();
         break;
      case PR2_RIGHT_GRIPPER:
         EnableGripperRight();
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::EnableHead()
{
   hw.EnableModel(HEAD);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableGripperLeft()
{
   hw.EnableModel(PR2_LEFT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableGripperRight()
{
   hw.EnableModel(PR2_RIGHT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableBase()
{
   hw.EnableModel(PR2_BASE);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableSpine()
{
   hw.EnableModel(PR2_SPINE);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableArm(PR2_MODEL_ID id)
{
   switch(id)
   {
      case PR2_LEFT_ARM:
         hw.DisableModel(PR2_LEFT_ARM);
         break;
      case PR2_RIGHT_ARM:
         hw.DisableModel(PR2_RIGHT_ARM);
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::DisableGripper(PR2_MODEL_ID id)
{
   switch(id)
   {
      case PR2_LEFT_GRIPPER:
         DisableGripperLeft();
         break;
      case PR2_RIGHT_GRIPPER:
         DisableGripperRight();
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::DisableHead()
{
   hw.DisableModel(HEAD);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableGripperLeft()
{
   hw.DisableModel(PR2_LEFT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableGripperRight()
{
   hw.DisableModel(PR2_RIGHT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableBase()
{
   hw.DisableModel(PR2_BASE);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableSpine()
{
   hw.DisableModel(PR2_SPINE);
   return PR2_ALL_OK;
}


PR2_ERROR_CODE PR2Robot::SetArmCartesianPosition(PR2_MODEL_ID id, const KDL::Frame &f, const KDL::JntArray &q_init, KDL::JntArray &q_out)
{
	//	KDL::JntArray q_init = KDL::JntArray(this->pr2_kin.nJnts);
	//	q_init(0) = 0.1, q_init(1) = 0.0, q_init(2) = 0.0, q_init(3) = 0.0;
	//	q_init(4) = 0.0, q_init(5) = 0.0, q_init(6) = 0.0;

	//	KDL::JntArray q_out = KDL::JntArray(this->pr2_kin.nJnts);
	if (this->pr2_kin.IK(q_init, f, q_out) == true)
		cout<<"IK result:"<<q_out<<endl;
	else
		cout<<"Could not compute Inv Kin."<<endl;

	//------ checking that IK returned a valid soln -----
	KDL::Frame f_ik;
	if (this->pr2_kin.FK(q_out,f_ik))
	{
		//		cout<<"End effector after IK:"<<f_ik<<endl;
	}
	else
		cout<<"Could not compute Fwd Kin. (After IK)"<<endl;

	for(int ii = 0; ii < 7; ii++)
		hw.SetJointServoCmd((PR2::PR2_JOINT_ID) (JointStart[id]+ii),q_out(ii),0);

	return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::SetArmCartesianPosition(PR2_MODEL_ID id, NEWMAT::Matrix g)
{
   NEWMAT::Matrix theta(8,8);
   double angles[7], speeds[7];
   int validSolution;

   if (id == PR2_RIGHT_ARM)
   {
      g(1,4) = g(1,4) - BASE_RIGHT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - BASE_RIGHT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - BASE_RIGHT_ARM_OFFSET.z;
   }

   if (id == PR2_LEFT_ARM)
   {
      g(1,4) = g(1,4) - BASE_LEFT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - BASE_LEFT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - BASE_LEFT_ARM_OFFSET.z;
   }

   theta = 0;
   theta = myArm.ComputeIK(g,0.1);
   for(int jj = 1; jj <= 8; jj++)
   {
      if (theta(8,jj) > -1)
      {
         validSolution = jj;
         break;
      }
   }
   validSolution = 1;
   if(validSolution <= 8)
   {
      for(int ii = 0; ii < 7; ii++)
      {
         angles[ii] = theta(ii+1,validSolution);
         speeds[ii] = 0;
         hw.SetJointServoCmd((PR2::PR2_JOINT_ID) (JointStart[id]+ii),angles[ii],0);
      }
   }
   return PR2_ALL_OK;
};

NEWMAT::Matrix PR2Robot::ComputeArmInverseKinematics(PR2_MODEL_ID id, NEWMAT::Matrix g)
{
   NEWMAT::Matrix theta(8,8);
   //double angles[7], speeds[7];
   //int validSolution;

   if (id == PR2_RIGHT_ARM)
   {
      g(1,4) = g(1,4) - BASE_RIGHT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - BASE_RIGHT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - BASE_RIGHT_ARM_OFFSET.z;
   }

   if (id == PR2_LEFT_ARM)
   {
      g(1,4) = g(1,4) - BASE_LEFT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - BASE_LEFT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - BASE_LEFT_ARM_OFFSET.z;
   }

   theta = 0;
   theta = myArm.ComputeIK(g,0.1);
   return theta;
};


bool PR2Robot::IsModel(PR2_MODEL_ID modelId, PR2_JOINT_ID jointId)
{
   if (jointId >= JointStart[modelId] && jointId <= JointEnd[modelId])
      return true;

   return false;
};

NEWMAT::Matrix GetPose(PR2::PR2State rS)
{
   NEWMAT::Matrix g(4,4);

   double x[3];
   x[0] = rS.pos.x;
   x[1] = rS.pos.y;
   x[2] = rS.pos.z;

   g = kinematics::Transform(x,rS.roll,rS.pitch,rS.yaw);
   return g;
}

void PR2Robot::ComputeBodyPose(PR2_JOINT_ID id, PR2::PR2State rS, double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
   double lR,lP,lY;
   double lR2,lP2,lY2;
   NEWMAT::Matrix g(4,4);

   g = ComputeBodyPose(id,rS);
   kinematics::MatrixToEuler(g,lR,lP,lY,lR2,lP2,lY2);

   *x = g(1,4);
   *y = g(2,4);
   *z = g(3,4);

   *roll = lR;
   *pitch = lP;
   *yaw = lY;
};


NEWMAT::Matrix PR2Robot::ComputeBodyPose(PR2_JOINT_ID id, PR2::PR2State rS)
{
   NEWMAT::Matrix worldToBase(4,4), baseToBody(4,4);
   NEWMAT::IdentityMatrix I(4);

   worldToBase = GetPose(rS);
   baseToBody = I;

#ifdef DEBUG
   cout << "Id:" << id << endl;
#endif

   if(IsModel(PR2::PR2_BASE,id))
   {
      baseToBody = ComputeBaseBodyPose(id,rS);
      baseToBody(3,4) += rS.spineExtension;
   }

   if(IsModel(PR2::HEAD,id))
   {
      baseToBody = ComputeHeadBodyPose(id,rS);
      baseToBody(3,4) += rS.spineExtension;
   }

   if(IsModel(PR2::PR2_RIGHT_ARM,id) || IsModel(PR2::PR2_LEFT_ARM,id))
   {
      baseToBody = ComputeArmBodyPose(id,rS);
      baseToBody(3,4) += rS.spineExtension;
   }
#ifdef DEBUG
   PrintMatrix(worldToBase,"ComputeBodyPose::worldToBase");
   PrintMatrix(baseToBody,"ComputeBodyPose::baseToBody");
#endif
   return (worldToBase*baseToBody);
};


NEWMAT::Matrix PR2Robot::ComputeBodyPose(PR2_JOINT_ID id, PR2::PR2State rS, NEWMAT::Matrix localPose)
{
   NEWMAT::Matrix worldToBody = ComputeBodyPose(id,rS);
   return (worldToBody*localPose);
};


NEWMAT::Matrix PR2Robot::ComputeHeadBodyPose(PR2_JOINT_ID id, PR2::PR2State rS)
{
   NEWMAT::Matrix g(4,4);
   g = myHead.ComputeFK(rS.headJntAngles,id-JointStart[HEAD]+1);
   g(1,4) = g(1,4) + BASE_HEAD_OFFSET.x;
   g(2,4) = g(2,4) + BASE_HEAD_OFFSET.y;
   g(3,4) = g(3,4) + BASE_HEAD_OFFSET.z;
   return g;
};

NEWMAT::Matrix PR2Robot::ComputeBaseBodyPose(PR2_JOINT_ID id, PR2::PR2State rS)
{
   NEWMAT::Matrix  worldToBase(4,4),baseToBody(4,4);
   NEWMAT::IdentityMatrix I(4);

   worldToBase = GetPose(rS);

   baseToBody = I;
   if (id != PR2::BASE_6DOF)
   {
      baseToBody(1,4) = BASE_BODY_OFFSETS[(int)(id-JointStart[BASE])].x ;
      baseToBody(2,4) = BASE_BODY_OFFSETS[(int)(id-JointStart[BASE])].y ;
      baseToBody(3,4) = BASE_BODY_OFFSETS[(int)(id-JointStart[BASE])].z ;
   }
   return (worldToBase*baseToBody);
};


NEWMAT::Matrix PR2Robot::ComputeArmBodyPose(PR2_JOINT_ID id, PR2::PR2State rS)
{
   NEWMAT::Matrix g(4,4);

   if (IsModel(PR2::PR2_RIGHT_ARM,id))
   {
      g      = myArm.ComputeFK(rS.rightArmJntAngles,id-JointStart[PR2_RIGHT_ARM]+1);
      g(1,4) = g(1,4) + BASE_RIGHT_ARM_OFFSET.x;
      g(2,4) = g(2,4) + BASE_RIGHT_ARM_OFFSET.y;
      g(3,4) = g(3,4) + BASE_RIGHT_ARM_OFFSET.z;
   }

   if (IsModel(PR2::PR2_LEFT_ARM,id))
   {
      g      = myArm.ComputeFK(rS.leftArmJntAngles,id-JointStart[PR2_LEFT_ARM]+1);
      g(1,4) = g(1,4) + BASE_LEFT_ARM_OFFSET.x;
      g(2,4) = g(2,4) + BASE_LEFT_ARM_OFFSET.y;
      g(3,4) = g(3,4) + BASE_LEFT_ARM_OFFSET.z;
   }

   return g;
};


NEWMAT::Matrix PR2Robot::ComputeArmForwardKinematics(PR2_MODEL_ID id, double angles[])
{
   NEWMAT::Matrix g = myArm.ComputeFK(angles);

   if (id == PR2_RIGHT_ARM)
   {
      g(1,4) = g(1,4) + BASE_RIGHT_ARM_OFFSET.x;
      g(2,4) = g(2,4) + BASE_RIGHT_ARM_OFFSET.y;
      g(3,4) = g(3,4) + BASE_RIGHT_ARM_OFFSET.z;
   }

   if (id == PR2_LEFT_ARM)
   {
      g(1,4) = g(1,4) + BASE_LEFT_ARM_OFFSET.x;
      g(2,4) = g(2,4) + BASE_LEFT_ARM_OFFSET.y;
      g(3,4) = g(3,4) + BASE_LEFT_ARM_OFFSET.z;
   }
   return g;
};


PR2_ERROR_CODE PR2Robot::SetBaseControlMode(PR2_CONTROL_MODE mode)
{
   baseControlMode = mode;
   if (mode ==  PR2_CARTESIAN_CONTROL)
   {
      hw.SetJointControlMode(CASTER_FL_DRIVE_L,SPEED_CONTROL);
      hw.SetJointControlMode(CASTER_FL_DRIVE_R,SPEED_CONTROL);
      hw.SetJointControlMode(CASTER_FR_DRIVE_L,SPEED_CONTROL);
      hw.SetJointControlMode(CASTER_FR_DRIVE_R,SPEED_CONTROL);
      hw.SetJointControlMode(CASTER_RL_DRIVE_L,SPEED_CONTROL);
      hw.SetJointControlMode(CASTER_RL_DRIVE_R,SPEED_CONTROL);
      hw.SetJointControlMode(CASTER_RR_DRIVE_L,SPEED_CONTROL);
      hw.SetJointControlMode(CASTER_RR_DRIVE_R,SPEED_CONTROL);

      hw.SetJointControlMode(CASTER_FL_STEER,PD_CONTROL);
      hw.SetJointControlMode(CASTER_FR_STEER,PD_CONTROL);
      hw.SetJointControlMode(CASTER_RL_STEER,PD_CONTROL);
      hw.SetJointControlMode(CASTER_RR_STEER,PD_CONTROL);
   }
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::GetBaseControlMode(PR2_CONTROL_MODE *mode)
{
   *mode = baseControlMode;
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::SetArmControlMode(PR2_MODEL_ID id, PR2_CONTROL_MODE mode)
{
   switch(id)
   {
      case PR2_RIGHT_ARM:
         armControlMode[0] = mode;
         hw.SetJointControlMode(ARM_L_PAN             , PD_CONTROL); 
         hw.SetJointControlMode(ARM_L_SHOULDER_PITCH  , PD_CONTROL);
         hw.SetJointControlMode(ARM_L_SHOULDER_ROLL   , PD_CONTROL); 
         hw.SetJointControlMode(ARM_L_ELBOW_PITCH     , PD_CONTROL);
         hw.SetJointControlMode(ARM_L_ELBOW_ROLL      , PD_CONTROL); 
         hw.SetJointControlMode(ARM_L_WRIST_PITCH     , PD_CONTROL);
         hw.SetJointControlMode(ARM_L_WRIST_ROLL      , PD_CONTROL); 
         break;
      case PR2_LEFT_ARM:
         armControlMode[1] = mode;
         hw.SetJointControlMode(ARM_R_PAN             , PD_CONTROL); 
         hw.SetJointControlMode(ARM_R_SHOULDER_PITCH  , PD_CONTROL);
         hw.SetJointControlMode(ARM_R_SHOULDER_ROLL   , PD_CONTROL); 
         hw.SetJointControlMode(ARM_R_ELBOW_PITCH     , PD_CONTROL);
         hw.SetJointControlMode(ARM_R_ELBOW_ROLL      , PD_CONTROL); 
         hw.SetJointControlMode(ARM_R_WRIST_PITCH     , PD_CONTROL);
         hw.SetJointControlMode(ARM_R_WRIST_ROLL      , PD_CONTROL); 
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmControlMode(PR2_MODEL_ID id, PR2_CONTROL_MODE *mode)
{
   switch(id)
   {
      case PR2_RIGHT_ARM:
         *mode = armControlMode[0];
         break;
      case PR2_LEFT_ARM:
         *mode = armControlMode[1];
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::SetArmJointPosition(PR2_MODEL_ID id, double jointPosition[], double jointSpeed[])
{

   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
   {
      hw.SetJointServoCmd((PR2_JOINT_ID)ii,jointPosition[ii-JointStart[id]],jointSpeed[ii-JointStart[id]]);
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointPositionCmd(PR2_MODEL_ID id, double jointPosition[], double jointSpeed[])
{

   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
   {
      hw.GetJointServoCmd((PR2_JOINT_ID)ii,&(jointPosition[ii-JointStart[id]]),&(jointSpeed[ii-JointStart[id]]));
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointPositionCmd(PR2_MODEL_ID id, KDL::JntArray &q)
{
	if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
		return PR2_ERROR;

	double pos,vel;
	for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
	{
		hw.GetJointServoCmd((PR2_JOINT_ID)ii,&pos,&vel);
		q(ii-JointStart[id]) = pos;
	}
	return PR2_ALL_OK;
};



PR2_ERROR_CODE PR2Robot::GetArmJointPositionActual(PR2_MODEL_ID id, double jointPosition[], double jointSpeed[])
{

   cout << "Entering joint positions " << endl << endl; 

   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   cout << "Getting joint positions " << endl << endl; 
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
   {
      hw.GetJointServoActual((PR2_JOINT_ID)ii, &(jointPosition[ii-JointStart[id]]), &(jointSpeed[ii-JointStart[id]]));
      //cout << "ii" << (ii-JointStart[id]) << endl;
   }
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::SetArmJointTorque(PR2_MODEL_ID id, double torque[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      hw.SetJointTorque((PR2_JOINT_ID)ii,torque[ii-JointStart[id]]);
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointTorqueCmd(PR2_MODEL_ID id, double torque[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      hw.GetJointTorqueCmd((PR2_JOINT_ID)ii,&(torque[ii-JointStart[id]]));
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointTorqueActual(PR2_MODEL_ID id, double torque[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      hw.GetJointTorqueActual((PR2_JOINT_ID)ii,&(torque[ii-JointStart[id]]));
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::SetArmJointSpeed(PR2_MODEL_ID id, double speed[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      hw.SetJointSpeed((PR2_JOINT_ID)ii,speed[ii-JointStart[id]]);
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
     hw.GetJointSpeedCmd((PR2_JOINT_ID)ii,&(speed[ii-JointStart[id]]));
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointSpeedActual(PR2_MODEL_ID id, double speed[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
     hw.GetJointSpeedActual((PR2_JOINT_ID)ii,&(speed[ii-JointStart[id]]));
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::StopRobot()
{
   return PR2_ALL_OK;
};

void ComputePointVelocity(double vx, double vy, double vw, double x_offset, double y_offset, double &pvx, double &pvy)
{
   pvx = vx - y_offset*vw;
   pvy = vy + x_offset*vw;
};


PR2_ERROR_CODE PR2Robot::SetBaseCartesianSpeedCmd(double vx, double vy, double vw)
{
   point drivePointVelocityL, drivePointVelocityR;
   double wheelSpeed[NUM_WHEELS];

   point steerPointVelocity[NUM_CASTERS];
   double steerAngle[NUM_CASTERS];

   point newDriveCenterL, newDriveCenterR;

   for(int ii=0; ii < NUM_CASTERS; ii++)
   {
      ComputePointVelocity(vx,vy,vw,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y);
      steerAngle[ii] = atan2(steerPointVelocity[ii].y,steerPointVelocity[ii].x);
      hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),steerAngle[ii],0);
      // printf("ii: %d, off: (%f, %f), vel: (%f, %f), angle: %f\n",ii,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y,steerAngle[ii]);
   }
   for(int ii = 0; ii < NUM_CASTERS; ii++)
   {

       newDriveCenterL = Rot2D(CASTER_DRIVE_OFFSET[ii*2  ],steerAngle[ii]);
       newDriveCenterR = Rot2D(CASTER_DRIVE_OFFSET[ii*2+1],steerAngle[ii]);

       newDriveCenterL.x += BASE_CASTER_OFFSET[ii].x;
       newDriveCenterL.y += BASE_CASTER_OFFSET[ii].y;
       newDriveCenterR.x += BASE_CASTER_OFFSET[ii].x;
       newDriveCenterR.y += BASE_CASTER_OFFSET[ii].y;

       ComputePointVelocity(vx,vy,vw,newDriveCenterL.x,newDriveCenterL.y,drivePointVelocityL.x,drivePointVelocityL.y);
       ComputePointVelocity(vx,vy,vw,newDriveCenterR.x,newDriveCenterR.y,drivePointVelocityR.x,drivePointVelocityR.y);

       wheelSpeed[ii*2  ] = -GetMagnitude(drivePointVelocityL.x,drivePointVelocityL.y)/WHEEL_RADIUS;
       wheelSpeed[ii*2+1] = -GetMagnitude(drivePointVelocityR.x,drivePointVelocityR.y)/WHEEL_RADIUS;

       // send command
       hw.SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_L+3*ii),wheelSpeed[ii*2  ]);
       hw.SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_R+3*ii),wheelSpeed[ii*2+1]);
   }

   return PR2_ALL_OK;
};


//  PR2_ERROR_CODE PR2Robot::SetBaseUnicycleSpeedCmd(double vx, double vw)
//  {
//     point drivePointVelocityL, drivePointVelocityR;
//     double wheelSpeed[NUM_WHEELS];
// 
//     point steerPointVelocity[NUM_CASTERS];
//     double steerAngle[NUM_CASTERS];
// 
//     point newDriveCenterL, newDriveCenterR;
// 
//        ComputePointVelocity(vx,vy,vw,BASE_CASTER_OFFSET[0].x,BASE_CASTER_OFFSET[0].y,steerPointVelocity[0].x,steerPointVelocity[0].y);
//        steerAngle[0] = atan2(steerPointVelocity[0].y,steerPointVelocity[0].x);
//        ComputePointVelocity(vx,vy,vw,BASE_CASTER_OFFSET[1].x,BASE_CASTER_OFFSET[1].y,steerPointVelocity[1].x,steerPointVelocity[1].y);
//        steerAngle[1] = atan2(steerPointVelocity[1].y,steerPointVelocity[1].x);
//        hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_FL_STEER),steerAngle[0],0);
//        hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_FR_STEER),steerAngle[1],0);
//        hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_RL_STEER),            0,0);
//        hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_RR_STEER),            0,0);
//        // printf("ii: %d, off: (%f, %f), vel: (%f, %f), angle: %f\n",ii,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y,steerAngle[ii]);
// 
//     for(int ii = 0; ii < NUM_CASTERS; ii++)
//     {
// 
//         newDriveCenterL = Rot2D(CASTER_DRIVE_OFFSET[ii*2  ],steerAngle[ii]);
//         newDriveCenterR = Rot2D(CASTER_DRIVE_OFFSET[ii*2+1],steerAngle[ii]);
// 
//         ComputePointVelocity(vx,vy,vw,newDriveCenterL.x,newDriveCenterL.y,drivePointVelocityL.x,drivePointVelocityL.y);
//         ComputePointVelocity(vx,vy,vw,newDriveCenterR.x,newDriveCenterR.y,drivePointVelocityR.x,drivePointVelocityR.y);
// 
//         wheelSpeed[ii*2  ] = -GetMagnitude(drivePointVelocityL.x,drivePointVelocityL.y)/WHEEL_RADIUS;
//         wheelSpeed[ii*2+1] = -GetMagnitude(drivePointVelocityR.x,drivePointVelocityR.y)/WHEEL_RADIUS;
// 
//         // send speed command
//         hw.SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_L),wheelSpeed[ii*2  ]);
//         hw.SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_R),wheelSpeed[ii*2+1]);
//     }
// 
//     return PR2_ALL_OK;
//  };


PR2_ERROR_CODE PR2Robot::SetBaseSteeringAngle(double vx, double vy, double vw)
{
   point steerPointVelocity[NUM_CASTERS];
   double steerAngle[NUM_CASTERS];

   for(int ii=0; ii < NUM_CASTERS; ii++)
   {
      ComputePointVelocity(vx,vy,vw,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y);
      steerAngle[ii] = atan2(steerPointVelocity[ii].y,steerPointVelocity[ii].x);
      hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),steerAngle[ii],0);
   }

   return PR2_ALL_OK;
};

double PR2Robot::BaseSteeringAngleError()
{
  double currentRateCmd[NUM_CASTERS];
  double currentRateAct[NUM_CASTERS];
  double currentCmd[NUM_CASTERS];
  double currentAngle[NUM_CASTERS];
  double currentError[NUM_CASTERS];
  double errorTotal;

  errorTotal = 0.0;

   for(int ii=0; ii < NUM_CASTERS; ii++)
   {
     // do not move forward unless the wheels are aligned...
     // this is a goofy hack, should put a more complicated version later
     hw.GetJointServoCmd((PR2_JOINT_ID)(CASTER_FL_STEER+3*ii),&currentCmd[ii],&currentRateCmd[ii]);
     hw.GetJointServoActual((PR2_JOINT_ID)(CASTER_FL_STEER+3*ii),&currentAngle[ii],&currentRateAct[ii]);
     currentError[ii] = fabs(shortest_angular_distance(currentCmd[ii] , currentAngle[ii]));
     //std::cout << " ii " << ii << " e " << currentError[ii] << std::endl;
     errorTotal = errorTotal + currentError[ii];
     fprintf(stdout," ii %d e %.3f - %.3f \n", ii , currentError[ii],errorTotal);
   }
   fprintf(stdout," ----------------------------------\n");

   return errorTotal;
};


PR2_ERROR_CODE PR2Robot::GetBaseCartesianSpeedCmd(double* vx, double* vy, double* vw)
{
   // FIXME: TODO: not implemented
   std::cout << "WARNING: GetBaseCartesianSpeedCmd is not implemented yet, returns 0's" << std::endl;
   *vx = 0.0;
   *vy = 0.0;
   *vw = 0.0;

   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetBaseCartesianSpeedActual(double* vx, double* vy, double* vw)
{
   double wheelSpeed[NUM_WHEELS];
   double steerAngleRate[NUM_WHEELS];
   double steerAngle[NUM_CASTERS];

   // some variable for estimation
   double casterAvgSpeed[NUM_CASTERS];
   double casterAvgCartesianX[NUM_CASTERS];
   double casterAvgCartesianY[NUM_CASTERS];
   // note, x-forward, y-sideways, vw-right hand rule (either, as long as consistent.)

   point newDriveCenterL, newDriveCenterR;


   // retrieve caster angles
   for(int ii=0; ii < NUM_CASTERS; ii++)
   {
      hw.GetJointServoActual((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),&steerAngle[ii],&steerAngleRate[ii]);
   }
   // retrieve caster wheel speeds
   for(int ii = 0; ii < NUM_CASTERS; ii++)
   {
      newDriveCenterL = Rot2D(CASTER_DRIVE_OFFSET[ii*2]  ,steerAngle[ii]);
      newDriveCenterR = Rot2D(CASTER_DRIVE_OFFSET[ii*2+1],steerAngle[ii]);

      hw.GetJointSpeedActual((PR2_JOINT_ID) (CASTER_FL_DRIVE_L+3*ii),&wheelSpeed[ii*2]);
      hw.GetJointSpeedActual((PR2_JOINT_ID) (CASTER_FL_DRIVE_R+3*ii),&wheelSpeed[ii*2+1]);

      // as a rough estimate, average the two wheels for now.
      casterAvgSpeed[ii] = 0.5*WHEEL_RADIUS*(wheelSpeed[ii*2] + wheelSpeed[ii*2+1]);
      casterAvgCartesianX[ii] = casterAvgSpeed[ii] * cos(steerAngle[ii]);
      casterAvgCartesianY[ii] = casterAvgSpeed[ii] * sin(steerAngle[ii]);
   }

   // estimate base speed based on wheel info
   // as a fist attempt, use averaged velocity, this is by NO MEANS a good approach, but fast for the time being
   *vx = 0.0;
   *vy = 0.0;
   *vw = 0.0;
   // rotational component
   for(int ii = 0; ii < NUM_CASTERS; ii++)
   {
       // average component in the rotational direction is casterAvgCartesian X offset / |offset|^2
       *vw = *vw + (casterAvgCartesianX[ii] * BASE_CASTER_OFFSET[ii].y - casterAvgCartesianY[ii] * BASE_CASTER_OFFSET[ii*2].x)
                  /((double)NUM_CASTERS*((pow(BASE_CASTER_OFFSET[ii].x,2) + pow(BASE_CASTER_OFFSET[ii].y,2))));
   }
   // translational component, simply average all
   for(int ii = 0; ii < NUM_CASTERS; ii++)
   {
       *vx = *vx + casterAvgCartesianX[ii]/(double)NUM_CASTERS;
       *vy = *vy + casterAvgCartesianY[ii]/(double)NUM_CASTERS;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetBasePositionActual(double* x, double* y, double *z, double *roll, double *pitch, double *yaw)
{
   hw.GetBasePositionGroundTruth(x,y,z,roll,pitch,yaw);
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::GetLeftGripperCmd(double *gap,double *force)
{
   hw.GetGripperCmd((PR2_MODEL_ID)PR2::PR2_LEFT_GRIPPER,gap,force);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::GetLeftGripperActual(double *gap,double *force)
{
   hw.GetGripperActual((PR2_MODEL_ID)PR2::PR2_LEFT_GRIPPER,gap,force);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::GetRightGripperCmd(double *gap,double *force)
{
   hw.GetGripperCmd((PR2_MODEL_ID)PR2::PR2_RIGHT_GRIPPER,gap,force);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::GetRightGripperActual(double *gap,double *force)
{
   hw.GetGripperActual((PR2_MODEL_ID)PR2::PR2_RIGHT_GRIPPER,gap,force);
   return PR2_ALL_OK;
}


PR2_ERROR_CODE PR2Robot::GetTime(double *time)
{
   return hw.GetSimTime(time);
}

