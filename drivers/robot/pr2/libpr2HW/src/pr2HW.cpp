
#include <libpr2HW/pr2HW.h>
#include <math.h>
#include <list>
#include <vector>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

#include <pr2Core/pr2Core.h>

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
static gazebo::Client           *client;
static gazebo::SimulationIface  *simIface;
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
static gazebo::PositionIface    *pr2LeftWristIface;
static gazebo::PositionIface    *pr2RightWristIface;
static gazebo::PositionIface    *pr2BaseIface;

////////////////////////////////////////////////////////////////////
//                                                                //
//  Helper functions                                              //
//                                                                //
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
//                                                                //
//  PR2HW Class                                                   //
//                                                                //
////////////////////////////////////////////////////////////////////
PR2HW::PR2HW()
{


}

PR2HW::~PR2HW()
{

}

PR2_ERROR_CODE PR2HW::AddJoint(PR2_JOINT_ID id)
{
  Pid tmpPid;
  this->pids.push_back(tmpPid);
  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::SetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE mode)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE *mode)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::SetJointGains(PR2_JOINT_ID id, double pGain, double iGain, double dGain)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointGains(PR2_JOINT_ID id, double *pGain, double *iGain, double *dGain)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::EnableJoint(PR2_JOINT_ID id)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::DisableJoint(PR2_JOINT_ID id)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::IsEnabledJoint(PR2_JOINT_ID id, int *enabled)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::SetJointParams(PR2_JOINT_ID id, PR2_JOINT_PARAM_ID pId, double value)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointParams(PR2_JOINT_ID id, PR2_JOINT_PARAM_ID pId, double *value)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::SetJointServoCmd(PR2_JOINT_ID id, double jointPosition, double jointSpeed)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointServoCmd(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointPositionActual(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::SetJointTorque(PR2_JOINT_ID id, double torque)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointTorqueCmd(PR2_JOINT_ID id, double *torque)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointTorqueActual(PR2_JOINT_ID id, double *torque)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::SetJointSpeed(PR2_JOINT_ID id, double speed)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointSpeedCmd(PR2_JOINT_ID id, double *speed)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetJointSpeedActual(PR2_JOINT_ID id, double *speed)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetLaserRanges(PR2_SENSOR_ID id,
    float* angle_min, float* angle_max, float* angle_increment,
    float* range_max,uint32_t* ranges_size     ,uint32_t* ranges_alloc_size,
    uint32_t* intensities_size,uint32_t* intensities_alloc_size,
    float*    ranges          ,uint8_t*  intensities)
{

  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::GetCameraImage(PR2_SENSOR_ID id ,
           uint32_t*    width                 ,uint32_t*    height                ,
           uint32_t*    depth                 ,
           std::string* compression           ,std::string* colorspace            ,
           uint32_t*    data_size             ,void*        data                  )
{

  return PR2_ALL_OK;
}


PR2_ERROR_CODE PR2HW::UpdateHW()
{

  return PR2_ALL_OK;
}


