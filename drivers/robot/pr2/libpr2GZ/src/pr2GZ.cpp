
#include <pr2Core/pr2Core.h>
#include <pr2Core/pr2Misc.h>
#include <libpr2GZ/pr2GZ.h>
#include <math.h>
#include <list>
#include <vector>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

using namespace gazebo;
using namespace PR2;

////////////////////////////////////////////////////////////////////
//                                                                //
//  Gazebo Model and Joint interfaces                             //
//                                                                //
////////////////////////////////////////////////////////////////////
// Gazebo/ODE joints
// static std::vector<gazebo::Joint> myJoints;
// Gazebo interface for this actarray
// static gazebo::PR2ArrayIface *myIface;
// Gazebo parent model
// static gazebo::Model *myParent;

////////////////////////////////////////////////////////////////////
//                                                                //
//  Gazebo Client Interfaces                                      //
//                                                                //
//  these are the "hardware" interfaces                           //
//                                                                //
////////////////////////////////////////////////////////////////////
gazebo::Client           *client;
gazebo::SimulationIface  *simIface;
gazebo::PR2ArrayIface    *pr2Iface;
gazebo::PR2ArrayIface    *pr2HeadIface;
gazebo::PR2GripperIface  *pr2GripperLeftIface;
gazebo::PR2GripperIface  *pr2GripperRightIface;
gazebo::LaserIface       *pr2LaserIface;
gazebo::LaserIface       *pr2BaseLaserIface;
gazebo::CameraIface      *pr2CameraIface;
gazebo::CameraIface      *pr2CameraGlobalIface;
gazebo::CameraIface      *pr2CameraHeadLeftIface;
gazebo::CameraIface      *pr2CameraHeadRightIface;
gazebo::PositionIface    *pr2LeftWristIface;
gazebo::PositionIface    *pr2RightWristIface;
gazebo::PositionIface    *pr2BaseIface;

////////////////////////////////////////////////////////////////////
//                                                                //
//  Helper functions                                              //
//                                                                //
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
//                                                                //
//  PR2GZ Class                                                   //
//                                                                //
////////////////////////////////////////////////////////////////////
PR2GZ::PR2GZ()
{


}

PR2GZ::~PR2GZ()
{

}

PR2_ERROR_CODE PR2GZ::Init()
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
   pr2RightWristIface      = new gazebo::PositionIface();
   pr2BaseIface            = new gazebo::PositionIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
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

  try
  {
    pr2BaseIface->Open(client, "p3d_base_position");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the base position interface\n"
    << e << "\n";
    pr2BaseIface = NULL;
  }
  return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2GZ::GetSimTime(double *sim_time)
{
   *sim_time = simIface->data->simTime;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2GZ::AddJoint(PR2_JOINT_ID id)
{
  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2GZ::EnableModel(PR2_MODEL_ID id)
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

PR2_ERROR_CODE PR2GZ::DisableModel(PR2_MODEL_ID id)
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

PR2_ERROR_CODE PR2GZ::IsEnabledModel(PR2_MODEL_ID id, int *enabled)
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

PR2_ERROR_CODE PR2GZ::SetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE mode)
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

PR2_ERROR_CODE PR2GZ::GetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE *mode)
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


PR2_ERROR_CODE PR2GZ::SetJointGains(PR2_JOINT_ID id, double pGain, double iGain, double dGain)
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

PR2_ERROR_CODE PR2GZ::GetJointGains(PR2_JOINT_ID id, double *pGain, double *iGain, double *dGain)
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

PR2_ERROR_CODE PR2GZ::EnableJoint(PR2_JOINT_ID id)
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

PR2_ERROR_CODE PR2GZ::DisableJoint(PR2_JOINT_ID id)
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

PR2_ERROR_CODE PR2GZ::IsEnabledJoint(PR2_JOINT_ID id, int *enabled)
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

PR2_ERROR_CODE PR2GZ::SetJointParams(PR2_JOINT_ID id, PR2_JOINT_PARAM_ID pId, double value)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2GZ::GetJointParams(PR2_JOINT_ID id, PR2_JOINT_PARAM_ID pId, double *value)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2GZ::SetJointServoCmd(PR2_JOINT_ID id, double jointPosition, double jointSpeed)
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


PR2_ERROR_CODE PR2GZ::GetJointServoCmd(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
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

PR2_ERROR_CODE PR2GZ::GetJointServoActual(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
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

PR2_ERROR_CODE PR2GZ::GetJointPositionActual(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
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


PR2_ERROR_CODE PR2GZ::SetJointTorque(PR2_JOINT_ID id, double torque)
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

PR2_ERROR_CODE PR2GZ::GetJointTorqueCmd(PR2_JOINT_ID id, double *torque)
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

PR2_ERROR_CODE PR2GZ::GetJointTorqueActual(PR2_JOINT_ID id, double *torque)
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

PR2_ERROR_CODE PR2GZ::SetJointSpeed(PR2_JOINT_ID id, double speed)
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

PR2_ERROR_CODE PR2GZ::GetJointSpeedCmd(PR2_JOINT_ID id, double *speed)
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

PR2_ERROR_CODE PR2GZ::GetJointSpeedActual(PR2_JOINT_ID id, double *speed)
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

PR2_ERROR_CODE PR2GZ::GetLaserRanges(PR2_SENSOR_ID id,
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
    tmpLaserIface->Lock(1);
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
    tmpLaserIface->Unlock();
    return PR2_ALL_OK;
  }
};

PR2_ERROR_CODE PR2GZ::ClientWait()
{
  // block until simulator update.
  client->Wait();
  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2GZ::UpdateGZ()
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2GZ::OpenGripper(PR2_MODEL_ID id,double gap,double force)
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

PR2_ERROR_CODE PR2GZ::CloseGripper(PR2_MODEL_ID id,double gap,double force)
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

PR2_ERROR_CODE PR2GZ::GetGripperCmd(PR2_MODEL_ID id,double *gap,double *force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
          *force = pr2GripperLeftIface->data->cmdForce;
          *gap   = pr2GripperLeftIface->data->cmdGap  ;
          break;
       case PR2_RIGHT_GRIPPER:
          *force = pr2GripperRightIface->data->cmdForce;
          *gap   = pr2GripperRightIface->data->cmdGap  ;
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2GZ::GetGripperActual(PR2_MODEL_ID id,double *gap,double *force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
          *force = pr2GripperLeftIface->data->gripperForce; // FIXME: this is saturation force
          *gap   = ( pr2GripperLeftIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                  +(-pr2GripperLeftIface->data->actualFingerPosition[1] + 0.015);
          break;
       case PR2_RIGHT_GRIPPER:
          *force = pr2GripperRightIface->data->gripperForce; // FIXME: this is saturation force
          *gap   = ( pr2GripperRightIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                  +(-pr2GripperRightIface->data->actualFingerPosition[1] + 0.015);
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2GZ::SetGripperGains(PR2_MODEL_ID id,double p,double i, double d)
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

PR2_ERROR_CODE PR2GZ::GetCameraImage(PR2_SENSOR_ID id ,
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

    // on first pass, the sensor does not update after cameraIface is opened.
    if (*data_size > 0)
    {
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
    }
    pr2CameraIface->Unlock();

    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2GZ::GetWristPoseGroundTruth(PR2_MODEL_ID id, double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
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


NEWMAT::Matrix PR2GZ::GetWristPoseGroundTruth(PR2_MODEL_ID id)
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
   g = kinematics::Transform(x,roll,pitch,yaw);
   delete x;
   return g;
};

PR2_ERROR_CODE PR2GZ::GetBasePositionGroundTruth(double* x, double* y, double *z, double *roll, double *pitch, double *yaw)
{
   pr2BaseIface->Lock(1);
   *x     = pr2BaseIface->data->pose.pos.x;
   *y     = pr2BaseIface->data->pose.pos.y;
   *z     = pr2BaseIface->data->pose.pos.z;
   *roll  = pr2BaseIface->data->pose.roll;
   *pitch = pr2BaseIface->data->pose.pitch;
   *yaw   = pr2BaseIface->data->pose.yaw;
   pr2BaseIface->Unlock();
   return PR2_ALL_OK;
};



