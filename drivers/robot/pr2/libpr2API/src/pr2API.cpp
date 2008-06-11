#include <libpr2API/pr2API.h>
#include <math.h>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

using namespace gazebo;
using namespace PR2;

static gazebo::Client *client;
static gazebo::SimulationIface *simIface;

static gazebo::PR2ArrayIface    *pr2Iface;
static gazebo::PR2ArrayIface    *pr2HeadIface;
static gazebo::PR2GripperIface  *pr2GripperLeftIface;
static gazebo::PR2GripperIface  *pr2GripperRightIface;
static gazebo::LaserIface       *pr2LaserIface;
static gazebo::LaserIface       *pr2BaseLaserIface;
static gazebo::CameraIface      *pr2CameraIface;
static gazebo::CameraIface      *pr2CameraGlobalIface;
static gazebo::CameraIface      *pr2CameraHeadLeftIface;
static gazebo::CameraIface      *pr2CameraHeadRightIface;

static gazebo::PositionIface      *pr2LeftWristIface;
static gazebo::PositionIface      *pr2RightWristIface;


point Rot2D(point p,double theta)
{
   point q;
   q.x = cos(theta)*p.x - sin(theta)*p.y;
   q.y = sin(theta)*p.y + cos(theta)*p.x;
   return q;
}

bool IsHead(PR2_MODEL_ID id)
{
   if(id == HEAD)
      return true;
   return false;
}
bool IsGripperLeft(PR2_MODEL_ID id)
{
   if(id == PR2_LEFT_GRIPPER)
      return true;
   return false;
}
bool IsGripperRight(PR2_MODEL_ID id)
{
   if(id == PR2_RIGHT_GRIPPER)
      return true;
   return false;
}

bool IsHead(PR2_JOINT_ID id)
{
   if(id >= JointStart[HEAD] && id <= JointEnd[HEAD])
      return true;
   return false;
}

bool IsGripperLeft(PR2_JOINT_ID id)
{
   if (id >= JointStart[ARM_L_GRIPPER] && id <= JointEnd[ARM_L_GRIPPER])
      return true;
   return false;
}
bool IsGripperRight(PR2_JOINT_ID id)
{
   if (id >= JointStart[ARM_R_GRIPPER] && id <= JointEnd[ARM_R_GRIPPER])
      return true;
   return false;
}

double GetMagnitude(double xl[], int num)
{
   int ii;
   double mag=0;
   for(ii=0; ii < num; ii++)
      mag += (xl[ii]*xl[ii]); 
   return sqrt(mag);
}

double GetMagnitude(double x, double y)
{
   return sqrt(x*x+y*y);
}

PR2Robot::PR2Robot()
{
};

PR2Robot::~PR2Robot(){};

PR2_ERROR_CODE PR2Robot::GetSimTime(double *sim_time)
{
   *sim_time = simIface->data->simTime;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::InitializeRobot()
{
   client                  = new gazebo::Client();
   simIface                = new gazebo::SimulationIface();
   pr2Iface                = new gazebo::PR2ArrayIface();
   pr2HeadIface            = new gazebo::PR2ArrayIface();
   pr2GripperLeftIface     = new gazebo::PR2GripperIface();
   pr2GripperRightIface    = new gazebo::PR2GripperIface();
   pr2LaserIface           = new gazebo::LaserIface();
   pr2BaseLaserIface       = new gazebo::LaserIface();
   pr2CameraGlobalIface    = new gazebo::CameraIface();
   pr2CameraHeadLeftIface  = new gazebo::CameraIface();
   pr2CameraHeadRightIface = new gazebo::CameraIface();

   pr2LeftWristIface       = new gazebo::PositionIface();
   pr2RightWristIface       = new gazebo::PositionIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return PR2_ERROR;
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return  PR2_ERROR;
  }

  /// Open the Position interface
  try
  {
    pr2Iface->Open(client, "pr2_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 interface\n"
    << e << "\n";
    return  PR2_ERROR;
  }

  /// Open the Position interface
  try
  {
    pr2HeadIface->Open(client, "pr2_head_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 head interface\n"
    << e << "\n";
    return  PR2_ERROR;
  }

  /// Open the Position interface for gripper left
  try
  {
    pr2GripperLeftIface->Open(client, "pr2_gripper_left_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 gripper left interface\n"
    << e << "\n";
    return  PR2_ERROR;
  }

  /// Open the Position interface for gripper right
  try
  {
    pr2GripperRightIface->Open(client, "pr2_gripper_right_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 gripper right interface\n"
    << e << "\n";
    return  PR2_ERROR;
  }

  /// Open the laser interface for hokuyo
  try
  {
    pr2LaserIface->Open(client, "laser_iface_1");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 laser interface\n"
    << e << "\n";
    pr2LaserIface = NULL;
    //return  PR2_ERROR;
  }


  /// Open the base laser interface for hokuyo
  try
  {
    pr2BaseLaserIface->Open(client, "base_laser_iface_1");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 base laser interface\n"
    << e << "\n";
    pr2BaseLaserIface = NULL;
    //return  PR2_ERROR;
  }


  /// Open the camera interface for hokuyo
  try
  {
    pr2CameraGlobalIface->Open(client, "cam1_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 camera interface\n"
    << e << "\n";
    pr2CameraGlobalIface = NULL;
    //return  PR2_ERROR;
  }

  try
  {
    pr2CameraHeadLeftIface->Open(client, "head_cam_left_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 camera interface\n"
    << e << "\n";
    pr2CameraHeadLeftIface = NULL;
    //return  PR2_ERROR;
  }

  try
  {
    pr2CameraHeadRightIface->Open(client, "head_cam_right_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 camera interface\n"
    << e << "\n";
    pr2CameraHeadRightIface = NULL;
    //return  PR2_ERROR;
  }

  try
  {
    pr2LeftWristIface->Open(client, "p3d_left_wrist_position");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the left wrist interface\n"
    << e << "\n";
    pr2LeftWristIface = NULL;
  }

  try
  {
    pr2RightWristIface->Open(client, "p3d_right_wrist_position");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the right wrist interface\n"
    << e << "\n";
    pr2RightWristIface = NULL;
  }

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::CalibrateRobot()
{
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::EnableJoint(PR2_JOINT_ID id)
{
   if (IsHead(id))
   {
      pr2HeadIface->Lock(1);
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEnableMotor = 1;
      pr2HeadIface->Unlock();
   }
   else if (IsGripperLeft(id))
   {
      pr2GripperLeftIface ->Lock(1);
      pr2GripperLeftIface ->data->cmdEnableMotor = 1;
      pr2GripperLeftIface ->Unlock();
   }
   else if (IsGripperRight(id))
   {
      pr2GripperRightIface->Lock(1);
      pr2GripperRightIface->data->cmdEnableMotor = 1;
      pr2GripperRightIface->Unlock();
   }
   else
   {
      pr2Iface->Lock(1);
      pr2Iface->data->actuators[id].cmdEnableMotor = 1;
      pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::DisableJoint(PR2_JOINT_ID id)
{
   if (IsHead(id))
   {
     pr2HeadIface->Lock(1);
     pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEnableMotor = 0;
     pr2HeadIface->Unlock();
   }
   else if (IsGripperLeft(id))
   {
     pr2GripperLeftIface ->Lock(1);
     pr2GripperLeftIface ->data->cmdEnableMotor = 0;
     pr2GripperLeftIface ->Unlock();
   }
   else if (IsGripperRight(id))
   {
     pr2GripperRightIface->Lock(1);
     pr2GripperRightIface->data->cmdEnableMotor = 0;
     pr2GripperRightIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     pr2Iface->data->actuators[id].cmdEnableMotor = 0;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::IsEnabledJoint(PR2_JOINT_ID id, int *enabled)
{
   if (IsHead(id))
   {
     pr2HeadIface->Lock(1);
     *enabled = pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEnableMotor;
     pr2HeadIface->Unlock();
   }
   else if (IsGripperLeft(id))
   {
     pr2GripperLeftIface->Lock(1);
     *enabled = pr2GripperLeftIface->data->cmdEnableMotor;
     pr2GripperLeftIface->Unlock();
   }
   else if (IsGripperRight(id))
   {
     pr2GripperRightIface->Lock(1);
     *enabled = pr2GripperRightIface->data->cmdEnableMotor;
     pr2GripperRightIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     *enabled = pr2Iface->data->actuators[id].cmdEnableMotor;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::EnableModel(PR2_MODEL_ID id)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
        // subtrace JointStart[HEAD] since HEAD controllers is a new actarray.
        pr2HeadIface->data->actuators[ii-JointStart[HEAD]].cmdEnableMotor = 1;
     pr2Iface->Unlock();
   }
   else if(IsGripperLeft(id))
   {
     // there is only one... for now
     pr2GripperLeftIface->Lock(1);
     pr2GripperLeftIface->data->cmdEnableMotor = 1;
     pr2GripperLeftIface->Unlock();
   }
   else if(IsGripperRight(id))
   {
     pr2GripperRightIface->Lock(1);
     pr2GripperRightIface->data->cmdEnableMotor = 1;
     pr2GripperRightIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
        pr2Iface->data->actuators[ii].cmdEnableMotor = 1;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::EnableArm(PR2_MODEL_ID id)
{
   switch(id)
   {
      case PR2_LEFT_ARM:
         EnableModel(PR2_LEFT_ARM);
         break;
      case PR2_RIGHT_ARM:
         EnableModel(PR2_RIGHT_ARM);
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
         pr2GripperLeftIface->data->cmdEnableMotor = 1;
         break;
      case PR2_RIGHT_GRIPPER:
         pr2GripperRightIface->data->cmdEnableMotor = 1;
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::EnableHead()
{
   EnableModel(HEAD);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableGripperLeft()
{
   EnableModel(PR2_LEFT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableGripperRight()
{
   EnableModel(PR2_RIGHT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableBase()
{
   EnableModel(PR2_BASE);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::EnableSpine()
{
   EnableModel(PR2_SPINE);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableModel(PR2_MODEL_ID id)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
        pr2HeadIface->data->actuators[ii-JointStart[HEAD]].cmdEnableMotor = 0;
     pr2HeadIface->Unlock();
   }
   else if(IsGripperLeft(id))
   {
     pr2GripperLeftIface->Lock(1);
     pr2GripperLeftIface->data->cmdEnableMotor = 0;
     pr2GripperLeftIface->Unlock();
   }
   else if(IsGripperRight(id))
   {
     pr2GripperRightIface->Lock(1);
     pr2GripperRightIface->data->cmdEnableMotor = 0;
     pr2GripperRightIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
        pr2Iface->data->actuators[ii].cmdEnableMotor = 0;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::DisableArm(PR2_MODEL_ID id)
{
   switch(id)
   {
      case PR2_LEFT_ARM:
         DisableModel(PR2_LEFT_ARM);
         break;
      case PR2_RIGHT_ARM:
         DisableModel(PR2_RIGHT_ARM);
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
         pr2GripperLeftIface->data->cmdEnableMotor = 0;
         break;
      case PR2_RIGHT_GRIPPER:
         pr2GripperRightIface->data->cmdEnableMotor = 0;
         break;
      default:
         break;
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::DisableHead()
{
   DisableModel(HEAD);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableGripperLeft()
{
   DisableModel(PR2_LEFT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableGripperRight()
{
   DisableModel(PR2_RIGHT_GRIPPER);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableBase()
{
   DisableModel(PR2_BASE);
   return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2Robot::DisableSpine()
{
   DisableModel(PR2_SPINE);
   return PR2_ALL_OK;
}


PR2_ERROR_CODE PR2Robot::IsEnabledModel(PR2_MODEL_ID id, int *enabled)
{
   int isEnabled = 1;
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
        isEnabled = pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEnableMotor && isEnabled;
     pr2HeadIface->Unlock();
   }
   else if(IsGripperLeft(id))
   {
     pr2GripperLeftIface->Lock(1);
     isEnabled = pr2GripperLeftIface->data->cmdEnableMotor && isEnabled;
     pr2GripperLeftIface->Unlock();
   }
   else if(IsGripperRight(id))
   {
     pr2GripperRightIface->Lock(1);
     isEnabled = pr2GripperRightIface->data->cmdEnableMotor && isEnabled;
     pr2GripperRightIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
        isEnabled = pr2Iface->data->actuators[id].cmdEnableMotor && isEnabled;
     pr2Iface->Unlock();
   }
   *enabled = isEnabled;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::SetArmCartesianPosition(PR2_MODEL_ID id, KDL::Frame f)
{
	KDL::JntArray q_init = KDL::JntArray(this->pr2_kin.nJnts);
	q_init(0) = 0.1, q_init(1) = 0.0, q_init(2) = 0.0, q_init(3) = 0.0;
	q_init(4) = 0.0, q_init(5) = 0.0, q_init(6) = 0.0;
	KDL::JntArray q_out = KDL::JntArray(this->pr2_kin.nJnts);
	if (this->pr2_kin.IK(q_init, f, q_out) == true)
		cout<<"IK result:"<<q_out<<endl;
	else
		cout<<"Could not compute Inv Kin."<<endl;

	//------ checking that IK returned a valid soln -----
	KDL::Frame f_ik;
	if (this->pr2_kin.FK(q_out,f_ik))
		cout<<"End effector after IK:"<<f_ik<<endl;
	else
		cout<<"Could not compute Fwd Kin. (After IK)"<<endl;

	for(int ii = 0; ii < 7; ii++)
		this->SetJointServoCmd((PR2::PR2_JOINT_ID) (JointStart[id]+ii),q_out(ii),0);
}


PR2_ERROR_CODE PR2Robot::SetArmCartesianPosition(PR2_MODEL_ID id, NEWMAT::Matrix g)
{
   NEWMAT::Matrix theta(8,8);
   double angles[7], speeds[7];
   int validSolution;

   if (id == PR2_RIGHT_ARM)
   {
      g(1,4) = g(1,4) - SPINE_RIGHT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - SPINE_RIGHT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - SPINE_RIGHT_ARM_OFFSET.z;
   }

   if (id == PR2_LEFT_ARM)
   {
      g(1,4) = g(1,4) - SPINE_LEFT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - SPINE_LEFT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - SPINE_LEFT_ARM_OFFSET.z;
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
         this->SetJointServoCmd((PR2::PR2_JOINT_ID) (JointStart[id]+ii),angles[ii],0);
      }
   }
   return PR2_ALL_OK;
};

NEWMAT::Matrix PR2Robot::ComputeArmInverseKinematics(PR2_MODEL_ID id, NEWMAT::Matrix g)
{
   NEWMAT::Matrix theta(8,8);
   double angles[7], speeds[7];
   int validSolution;

   if (id == PR2_RIGHT_ARM)
   {
      g(1,4) = g(1,4) - SPINE_RIGHT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - SPINE_RIGHT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - SPINE_RIGHT_ARM_OFFSET.z;
   }

   if (id == PR2_LEFT_ARM)
   {
      g(1,4) = g(1,4) - SPINE_LEFT_ARM_OFFSET.x;
      g(2,4) = g(2,4) - SPINE_LEFT_ARM_OFFSET.y;
      g(3,4) = g(3,4) - SPINE_LEFT_ARM_OFFSET.z;
   }

   theta = 0;
   theta = myArm.ComputeIK(g,0.1);
   return theta;
};



NEWMAT::Matrix PR2Robot::ComputeArmForwardKinematics(PR2_MODEL_ID id, double angles[])
{
   NEWMAT::Matrix g = myArm.ComputeFK(angles);

   if (id == PR2_RIGHT_ARM)
   {
      g(1,4) = g(1,4) + SPINE_RIGHT_ARM_OFFSET.x;
      g(2,4) = g(2,4) + SPINE_RIGHT_ARM_OFFSET.y;
      g(3,4) = g(3,4) + SPINE_RIGHT_ARM_OFFSET.z;
   }

   if (id == PR2_LEFT_ARM)
   {
      g(1,4) = g(1,4) + SPINE_LEFT_ARM_OFFSET.x;
      g(2,4) = g(2,4) + SPINE_LEFT_ARM_OFFSET.y;
      g(3,4) = g(3,4) + SPINE_LEFT_ARM_OFFSET.z;
   }
   return g;
};


PR2_ERROR_CODE PR2Robot::SetBaseControlMode(PR2_CONTROL_MODE mode)
{
   baseControlMode = mode;
   if (mode ==  PR2_CARTESIAN_CONTROL)
   {
      SetJointControlMode(CASTER_FL_DRIVE_L,SPEED_CONTROL);
      SetJointControlMode(CASTER_FL_DRIVE_R,SPEED_CONTROL);
      SetJointControlMode(CASTER_FR_DRIVE_L,SPEED_CONTROL);
      SetJointControlMode(CASTER_FR_DRIVE_R,SPEED_CONTROL);
      SetJointControlMode(CASTER_RL_DRIVE_L,SPEED_CONTROL);
      SetJointControlMode(CASTER_RL_DRIVE_R,SPEED_CONTROL);
      SetJointControlMode(CASTER_RR_DRIVE_L,SPEED_CONTROL);
      SetJointControlMode(CASTER_RR_DRIVE_R,SPEED_CONTROL);

      SetJointControlMode(CASTER_FL_STEER,PD_CONTROL);
      SetJointControlMode(CASTER_FR_STEER,PD_CONTROL);
      SetJointControlMode(CASTER_RL_STEER,PD_CONTROL);
      SetJointControlMode(CASTER_RR_STEER,PD_CONTROL);
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
         break;
      case PR2_LEFT_ARM:
         armControlMode[1] = mode;
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

PR2_ERROR_CODE PR2Robot::SetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE mode)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     pr2HeadIface->data->actuators[id-JointStart[HEAD]].controlMode = mode;
     pr2HeadIface->Unlock();
   }
   else if(IsGripperLeft(id))
   {
     pr2GripperLeftIface->Lock(1);
     pr2GripperLeftIface->data->controlMode = mode;
     pr2GripperLeftIface->Unlock();
   }
   else if(IsGripperRight(id))
   {
     pr2GripperRightIface->Lock(1);
     pr2GripperRightIface->data->controlMode = mode;
     pr2GripperRightIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     pr2Iface->data->actuators[id].controlMode = mode;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE *mode)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     *mode = (PR2_JOINT_CONTROL_MODE) pr2HeadIface->data->actuators[id-JointStart[HEAD]].controlMode;
     pr2HeadIface->Unlock();
   }
   else if(IsGripperLeft(id))
   {
     pr2GripperLeftIface->Lock(1);
     *mode = (PR2_JOINT_CONTROL_MODE) pr2GripperLeftIface->data->controlMode;
     pr2GripperLeftIface->Unlock();
   }
   else if(IsGripperRight(id))
   {
     pr2GripperRightIface->Lock(1);
     *mode = (PR2_JOINT_CONTROL_MODE) pr2GripperRightIface->data->controlMode;
     pr2GripperRightIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     *mode = (PR2_JOINT_CONTROL_MODE) pr2Iface->data->actuators[id].controlMode;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::SetJointGains(PR2_JOINT_ID id, double pGain, double iGain, double dGain)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     pr2HeadIface->data->actuators[id-JointStart[HEAD]].pGain = pGain;
     pr2HeadIface->data->actuators[id-JointStart[HEAD]].iGain = iGain;
     pr2HeadIface->data->actuators[id-JointStart[HEAD]].dGain = dGain;
     pr2HeadIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     pr2Iface->data->actuators[id].pGain = pGain;
     pr2Iface->data->actuators[id].iGain = iGain;
     pr2Iface->data->actuators[id].dGain = dGain;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;

};

PR2_ERROR_CODE PR2Robot::GetJointGains(PR2_JOINT_ID id, double *pGain, double *iGain, double *dGain)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     *pGain = pr2HeadIface->data->actuators[id-JointStart[HEAD]].pGain;
     *iGain = pr2HeadIface->data->actuators[id-JointStart[HEAD]].iGain;
     *dGain = pr2HeadIface->data->actuators[id-JointStart[HEAD]].dGain;
     pr2HeadIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     *pGain = pr2Iface->data->actuators[id].pGain;
     *iGain = pr2Iface->data->actuators[id].iGain;
     *dGain = pr2Iface->data->actuators[id].dGain;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;

};

PR2_ERROR_CODE PR2Robot::SetJointServoCmd(PR2_JOINT_ID id, double jointPosition, double jointSpeed)
{
  if(IsHead(id))
  {
    pr2HeadIface->Lock(1);
    pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdPosition = jointPosition;
    pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdSpeed = jointSpeed;
    pr2HeadIface->Unlock();
  }
  else
  {
    pr2Iface->Lock(1);
    pr2Iface->data->actuators[id].cmdPosition = jointPosition;
    pr2Iface->data->actuators[id].cmdSpeed = jointSpeed;
    pr2Iface->Unlock();
  }
  return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::GetJointServoCmd(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{
  if(IsHead(id))
  {
    pr2HeadIface->Lock(1);
    *jointPosition =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdPosition;
    *jointSpeed =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdSpeed;
    pr2HeadIface->Unlock();
  }
  else
  {
    pr2Iface->Lock(1);
    *jointPosition =  pr2Iface->data->actuators[id].cmdPosition;
    *jointSpeed = pr2Iface->data->actuators[id].cmdSpeed;
    pr2Iface->Unlock();
  }
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetJointServoActual(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{
  if(IsHead(id))
  {
    pr2HeadIface->Lock(1);
    *jointPosition =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualPosition;
    *jointSpeed =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualSpeed;
    pr2HeadIface->Unlock();
  }
  else
  {
    pr2Iface->Lock(1);
    *jointPosition =  pr2Iface->data->actuators[id].actualPosition;
    *jointSpeed = pr2Iface->data->actuators[id].actualSpeed;
    pr2Iface->Unlock();
  }
  return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::GetJointPositionActual(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{
   if(IsHead(id))
      {
   pr2HeadIface->Lock(1);
 *jointPosition =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualPosition;
 *jointSpeed =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualSpeed;
   pr2HeadIface->Unlock();
      }
      else{
   pr2Iface->Lock(1);
  *jointPosition =  pr2Iface->data->actuators[id].actualPosition;
  *jointSpeed = pr2Iface->data->actuators[id].actualSpeed;
   pr2Iface->Unlock();
      }
   return PR2_ALL_OK;
};



PR2_ERROR_CODE PR2Robot::SetArmJointPosition(PR2_MODEL_ID id, double jointPosition[], double jointSpeed[])
{

   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
   {
      pr2Iface->data->actuators[ii].cmdPosition = jointPosition[ii-JointStart[id]];
      pr2Iface->data->actuators[ii].cmdSpeed = jointSpeed[ii-JointStart[id]];
   }
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointPositionCmd(PR2_MODEL_ID id, double jointPosition[], double jointSpeed[])
{

   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
   {
      jointPosition[ii-JointStart[id]] = pr2Iface->data->actuators[ii].cmdPosition;
      jointSpeed[ii-JointStart[id]] = pr2Iface->data->actuators[ii].cmdSpeed;
   }
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointPositionActual(PR2_MODEL_ID id, double jointPosition[], double jointSpeed[])
{

   cout << "Entering joint positions " << endl << endl; 

   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   cout << "Getting joint positions " << endl << endl; 
   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
   {
      jointPosition[ii-JointStart[id]] = pr2Iface->data->actuators[ii].actualPosition;
      jointSpeed[ii-JointStart[id]] = pr2Iface->data->actuators[ii].actualSpeed;
      cout << "ii" << (ii-JointStart[id]) << endl;
   }
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::SetJointTorque(PR2_JOINT_ID id, double torque)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEffectorForce = torque;
     pr2HeadIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     pr2Iface->data->actuators[id].cmdEffectorForce = torque;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetJointTorqueCmd(PR2_JOINT_ID id, double *torque)
{
   if(IsHead(id))
   {
   pr2HeadIface->Lock(1);
   *torque = pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEffectorForce;
   pr2HeadIface->Unlock();
   }
   else
   {
   pr2Iface->Lock(1);
   *torque = pr2Iface->data->actuators[id].cmdEffectorForce;
   pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetJointTorqueActual(PR2_JOINT_ID id, double *torque)
{
   if(IsHead(id))
   {
   pr2HeadIface->Lock(1);
   *torque = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualEffectorForce;
   pr2HeadIface->Unlock();
   }
   else
   {
   pr2Iface->Lock(1);
   *torque = pr2Iface->data->actuators[id].actualEffectorForce;
   pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::SetArmJointTorque(PR2_MODEL_ID id, double torque[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      pr2Iface->data->actuators[ii].cmdEffectorForce = torque[ii-JointStart[id]];
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointTorqueCmd(PR2_MODEL_ID id, double torque[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      torque[ii-JointStart[id]] = pr2Iface->data->actuators[ii].cmdEffectorForce;
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointTorqueActual(PR2_MODEL_ID id, double torque[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;

   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      torque[ii-JointStart[id]] = pr2Iface->data->actuators[ii].actualEffectorForce;
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::SetJointSpeed(PR2_JOINT_ID id, double speed)
{
   if(IsHead(id))
   {
     pr2HeadIface->Lock(1);
     pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdSpeed = speed;
     pr2HeadIface->Unlock();
   }
   else
   {
     pr2Iface->Lock(1);
     pr2Iface->data->actuators[id].cmdSpeed = speed;
     pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetJointSpeedCmd(PR2_JOINT_ID id, double *speed)
{
   if(IsHead(id))
   {
   pr2HeadIface->Lock(1);
   *speed = pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdSpeed;
   pr2HeadIface->Unlock();
   }
   else{
   pr2Iface->Lock(1);
   *speed = pr2Iface->data->actuators[id].cmdSpeed;
   pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetJointSpeedActual(PR2_JOINT_ID id, double *speed)
{
   if(IsHead(id))
   {
   pr2HeadIface->Lock(1);
   *speed = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualSpeed;
   pr2HeadIface->Unlock();
   }
   else{
   pr2Iface->Lock(1);
   *speed = pr2Iface->data->actuators[id].actualSpeed;
   pr2Iface->Unlock();
   }
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::SetArmJointSpeed(PR2_MODEL_ID id, double speed[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;
   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      pr2Iface->data->actuators[ii].cmdSpeed = speed[ii-JointStart[id]];
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointSpeedCmd(PR2_MODEL_ID id, double speed[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;
   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
       speed[ii-JointStart[id]] = pr2Iface->data->actuators[ii].cmdSpeed;
   pr2Iface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetArmJointSpeedActual(PR2_MODEL_ID id, double speed[])
{
   if (id != PR2_RIGHT_ARM && id != PR2_LEFT_ARM)
      return PR2_ERROR;
   pr2Iface->Lock(1);
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
       speed[ii-JointStart[id]] = pr2Iface->data->actuators[ii].actualSpeed;
   pr2Iface->Unlock();
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
      SetJointServoCmd((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),steerAngle[ii],0);
//      printf("ii: %d, off: (%f, %f), vel: (%f, %f), angle: %f\n",ii,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y,steerAngle[ii]);
   }
   for(int ii = 0; ii < NUM_CASTERS; ii++)
   {
      newDriveCenterL = Rot2D(CASTER_DRIVE_OFFSET[ii*2  ],steerAngle[ii]);
      newDriveCenterR = Rot2D(CASTER_DRIVE_OFFSET[ii*2+1],steerAngle[ii]);

      ComputePointVelocity(vx,vy,vw,newDriveCenterL.x,newDriveCenterL.y,drivePointVelocityL.x,drivePointVelocityL.y);
      ComputePointVelocity(vx,vy,vw,newDriveCenterR.x,newDriveCenterR.y,drivePointVelocityR.x,drivePointVelocityR.y);

      wheelSpeed[ii*2  ] = -GetMagnitude(drivePointVelocityL.x,drivePointVelocityL.y)/WHEEL_RADIUS;
      wheelSpeed[ii*2+1] = -GetMagnitude(drivePointVelocityR.x,drivePointVelocityR.y)/WHEEL_RADIUS;

      this->SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_L+3*ii),wheelSpeed[ii*2  ]);
      this->SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_R+3*ii),wheelSpeed[ii*2+1]);
      this->SetJointTorque((PR2_JOINT_ID) (CASTER_FL_DRIVE_L+3*ii),100.0 );
      this->SetJointTorque((PR2_JOINT_ID) (CASTER_FL_DRIVE_R+3*ii),100.0 );
   }

   return PR2_ALL_OK;
};



PR2_ERROR_CODE PR2Robot::SetBaseSteeringAngle(double vx, double vy, double vw)
{
   point steerPointVelocity[NUM_CASTERS];
   double steerAngle[NUM_CASTERS];

   point newDriveCenterL, newDriveCenterR;


   for(int ii=0; ii < NUM_CASTERS; ii++)
   {
      ComputePointVelocity(vx,vy,vw,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y);
      steerAngle[ii] = atan2(steerPointVelocity[ii].y,steerPointVelocity[ii].x);
      SetJointServoCmd((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),steerAngle[ii],0);
   }

   return PR2_ALL_OK;
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
      GetJointServoActual((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),&steerAngle[ii],&steerAngleRate[ii]);
   }
   // retrieve caster wheel speeds
   for(int ii = 0; ii < NUM_CASTERS; ii++)
   {
      newDriveCenterL = Rot2D(CASTER_DRIVE_OFFSET[ii*2]  ,steerAngle[ii]);
      newDriveCenterR = Rot2D(CASTER_DRIVE_OFFSET[ii*2+1],steerAngle[ii]);

      GetJointSpeedActual((PR2_JOINT_ID) (CASTER_FL_DRIVE_L+3*ii),&wheelSpeed[ii*2]);
      GetJointSpeedActual((PR2_JOINT_ID) (CASTER_FL_DRIVE_R+3*ii),&wheelSpeed[ii*2+1]);

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

PR2_ERROR_CODE PR2Robot::GetBasePositionActual(double* x, double* y, double* th)
{
   simIface->Lock(1);
   *x = simIface->data->model_pose.pos.x;
   *y = simIface->data->model_pose.pos.y;
   *th = simIface->data->model_pose.yaw;
   simIface->Unlock();
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2Robot::GetWristPoseGroundTruth(PR2_MODEL_ID id, double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
   switch(id)
   {
      case PR2::PR2_LEFT_ARM:
         pr2LeftWristIface->Lock(1);
         *x = pr2LeftWristIface->data->pose.pos.x;
         *y = pr2LeftWristIface->data->pose.pos.y;
         *z = pr2LeftWristIface->data->pose.pos.z;
         *roll = pr2LeftWristIface->data->pose.roll;
         *pitch = pr2LeftWristIface->data->pose.pitch;
         *yaw = pr2LeftWristIface->data->pose.yaw;
         pr2LeftWristIface->Unlock();
         break;
      case PR2::PR2_RIGHT_ARM:
         pr2RightWristIface->Lock(1);
         *x = pr2RightWristIface->data->pose.pos.x;
         *y = pr2RightWristIface->data->pose.pos.y;
         *z = pr2RightWristIface->data->pose.pos.z;
         *roll = pr2RightWristIface->data->pose.roll;
         *pitch = pr2RightWristIface->data->pose.pitch;
         *yaw = pr2RightWristIface->data->pose.yaw;
         pr2RightWristIface->Unlock();
         break;
      default:
         *x = 0;
         *y = 0;
         *z = 0;
         *roll = 0;
         *pitch = 0;
         *yaw = 0;
         return PR2_ERROR;
   }
   return PR2_ALL_OK;
};


NEWMAT::Matrix PR2Robot::GetWristPoseGroundTruth(PR2_MODEL_ID id)
{
   NEWMAT::Matrix g(4,4);
   double *x = new double[3];
   double roll,pitch,yaw;
   g = 0;
   switch(id)
   {
      case PR2::PR2_LEFT_ARM:
         pr2LeftWristIface->Lock(1);
         x[0] = pr2LeftWristIface->data->pose.pos.x;
         x[1] = pr2LeftWristIface->data->pose.pos.y;
         x[2] = pr2LeftWristIface->data->pose.pos.z;
         roll = pr2LeftWristIface->data->pose.roll;
         pitch = pr2LeftWristIface->data->pose.pitch;
         yaw = pr2LeftWristIface->data->pose.yaw;

         pr2LeftWristIface->Unlock();
         break;
      case PR2::PR2_RIGHT_ARM:
         pr2RightWristIface->Lock(1);
         x[0] = pr2RightWristIface->data->pose.pos.x;
         x[1] = pr2RightWristIface->data->pose.pos.y;
         x[2] = pr2RightWristIface->data->pose.pos.z;
         roll = pr2RightWristIface->data->pose.roll;
         pitch = pr2RightWristIface->data->pose.pitch;
         yaw = pr2RightWristIface->data->pose.yaw;
         pr2RightWristIface->Unlock();
         break;
      default:
         x[0] = 0;
         x[1] = 0;
         x[2] = 0;
         roll = 0;
         pitch = 0;
         yaw = 0;
         break;
   }
   cout << "Transform::" << x[0] << "," << x[1] << "," << x[2] << endl;
   cout << "rpy::" << roll << "," << pitch << "," << yaw << endl;
   g = Transform(x,roll,pitch,yaw);
   delete x;
   return g;
};

PR2_ERROR_CODE PR2Robot::GetLaserRanges(PR2_SENSOR_ID id,
    float* angle_min, float* angle_max, float* angle_increment,
    float* range_max,uint32_t* ranges_size     ,uint32_t* ranges_alloc_size,
                     uint32_t* intensities_size,uint32_t* intensities_alloc_size,
                     float*    ranges          ,uint8_t*  intensities)
{

  gazebo::LaserIface       *tmpLaserIface;
  switch (id)
  {
    case LASER_HEAD:
      tmpLaserIface = pr2LaserIface;
      break;
    case LASER_BASE:
      tmpLaserIface = pr2BaseLaserIface;
      break;
    default:
      tmpLaserIface = NULL;
      break;
  }
  if (tmpLaserIface == NULL)
  {
    return PR2_ERROR;
  }
  else
  {
    *angle_min               = (float)tmpLaserIface->data->min_angle;
    *angle_max               = (float)tmpLaserIface->data->max_angle;

    *range_max               = (float)tmpLaserIface->data->max_range;


    *ranges_size             = (uint32_t)tmpLaserIface->data->range_count;
    *ranges_alloc_size       = (uint32_t)GZ_LASER_MAX_RANGES;
    *intensities_size        = (uint32_t)tmpLaserIface->data->range_count;
    *intensities_alloc_size  = (uint32_t)GZ_LASER_MAX_RANGES;


    //*angle_increment         = (float)tmpLaserIface->data->res_angle;
    *angle_increment         = (*angle_max - *angle_min)/((double)(*ranges_size -1));

    // std::cout << "max " << *angle_max << std::endl;
    // std::cout << "min " << *angle_min << std::endl;
    // std::cout << "siz " << *ranges_size << std::endl;
    // std::cout << "inc " << *angle_increment << std::endl;
    // std::cout << "in2 " << (float)tmpLaserIface->data->res_angle << std::endl;
    // std::cout << "getting laser ranges" << std::endl;

    //ranges                   = cast(float*)tmpLaserIface->data->ranges;
    //intensities              = cast(float*)tmpLaserIface->data->intensity;

    //std::cout << "range count = " << count << std::endl;
    for (int i=0; i<(int)*ranges_size ; i++)
    {
      //std::cout << ranges[i] << " ";
      // fill in the needed messages
      ranges[i] = (float)tmpLaserIface->data->ranges[i];
      intensities[i] = (uint8_t)tmpLaserIface->data->intensity[i];
    }
    //std::cout << std::endl;
    return PR2_ALL_OK;
  }
};

PR2_ERROR_CODE PR2Robot::OpenGripper(PR2_MODEL_ID id,double gap,double force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
          pr2GripperLeftIface->data->cmd = GAZEBO_PR2GRIPPER_CMD_OPEN;
          pr2GripperLeftIface->data->cmdPositionRate  = 1.0;
          pr2GripperLeftIface->data->cmdForce         = force;
          pr2GripperLeftIface->data->cmdGap           = gap;
          break;
       case PR2_RIGHT_GRIPPER:
          pr2GripperRightIface->data->cmd = GAZEBO_PR2GRIPPER_CMD_OPEN;
          pr2GripperRightIface->data->cmdPositionRate = 1.0;
          pr2GripperRightIface->data->cmdForce        = force;
          pr2GripperRightIface->data->cmdGap          = gap;
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::CloseGripper(PR2_MODEL_ID id,double gap,double force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
          pr2GripperLeftIface->data->cmd = GAZEBO_PR2GRIPPER_CMD_CLOSE;
          pr2GripperLeftIface->data->cmdPositionRate  = 1.0;
          pr2GripperLeftIface->data->cmdForce         = force;
          pr2GripperLeftIface->data->cmdGap           = gap;
          break;
       case PR2_RIGHT_GRIPPER:
          pr2GripperRightIface->data->cmd = GAZEBO_PR2GRIPPER_CMD_CLOSE;
          pr2GripperRightIface->data->cmdPositionRate = 1.0;
          pr2GripperRightIface->data->cmdForce        = force;
          pr2GripperRightIface->data->cmdGap          = gap;
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::SetGripperGains(PR2_MODEL_ID id,double p,double i, double d)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
          pr2GripperLeftIface->data->pGain  = p;
          pr2GripperLeftIface->data->iGain  = i;
          pr2GripperLeftIface->data->dGain  = d;
          break;
       case PR2_RIGHT_GRIPPER:
          pr2GripperRightIface->data->pGain = p;
          pr2GripperRightIface->data->iGain = i;
          pr2GripperRightIface->data->dGain = d;
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2Robot::GetCameraImage(PR2_SENSOR_ID id ,
                     uint32_t*    width                 ,uint32_t*    height                ,
                     uint32_t*    depth                 ,
                     std::string* compression           ,std::string* colorspace            ,
                     uint32_t*    data_size             ,void*        buf                   )
{

    switch(id)
    {
      case CAMERA_GLOBAL:
          pr2CameraIface = pr2CameraGlobalIface;
          break;
      case CAMERA_HEAD_LEFT:
          pr2CameraIface = pr2CameraHeadLeftIface;
          break;
      case CAMERA_HEAD_RIGHT:
          pr2CameraIface = pr2CameraHeadRightIface;
          break;
      default:
          pr2CameraIface = pr2CameraGlobalIface;
          break;
    }

    pr2CameraIface->Lock(1);
    *width        = (uint32_t)pr2CameraIface->data->width;
    *height       = (uint32_t)pr2CameraIface->data->height;
    *compression  = "jpeg";
    *colorspace   = "rgb"; //"mono";
    *data_size    = pr2CameraIface->data->image_size;
    *depth        = (*data_size)/((*width)*(*height));

    uint32_t       buf_size = (*width) * (*height) * (*depth);

    // copy the image into local buffer
#if 0
    //buf = (void*)(pr2CameraIface->data->image);
    memcpy(buf,pr2CameraIface->data->image,buf_size);
#else
    for (uint32_t i = 0; i < buf_size ; i=i+3)
    {
      // flip red and blue
      ((unsigned char*)buf)[i  ] = pr2CameraIface->data->image[i+2];
      ((unsigned char*)buf)[i+1] = pr2CameraIface->data->image[i+1];
      ((unsigned char*)buf)[i+2] = pr2CameraIface->data->image[i  ];
      //printf("%d %d\n",i,pr2CameraIface->data->image[i]);
    }
#endif
    pr2CameraIface->Unlock();

    return PR2_ALL_OK;
};



