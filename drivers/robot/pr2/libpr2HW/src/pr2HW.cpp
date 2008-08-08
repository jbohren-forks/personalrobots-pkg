
#include <pr2Core/pr2Core.h>
#include <pr2Core/pr2Misc.h>
#include <libpr2HW/pr2HW.h>
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
gazebo::LaserIface       *pr2LaserIface;
gazebo::LaserIface       *pr2BaseLaserIface;
gazebo::CameraIface      *pr2CameraGlobalIface;
gazebo::CameraIface      *pr2PTZCameraLeftIface;
gazebo::CameraIface      *pr2PTZCameraRightIface;
gazebo::PTZIface         *pr2PTZLeftIface;
gazebo::PTZIface         *pr2PTZRightIface;
gazebo::PositionIface    *pr2LeftWristIface;
gazebo::PositionIface    *pr2RightWristIface;
gazebo::PositionIface    *pr2BaseIface;
gazebo::PositionIface    *posObjectIface;
gazebo::CameraIface      *pr2WristCameraLeftIface;
gazebo::CameraIface      *pr2WristCameraRightIface;
gazebo::CameraIface      *pr2ForearmCameraLeftIface;
gazebo::CameraIface      *pr2ForearmCameraRightIface;

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

PR2_ERROR_CODE PR2HW::Init()
{
   client                  = new gazebo::Client();
   simIface                = new gazebo::SimulationIface();
   pr2Iface                = new gazebo::PR2ArrayIface();
   pr2LaserIface           = new gazebo::LaserIface();
   pr2BaseLaserIface       = new gazebo::LaserIface();
   pr2CameraGlobalIface    = new gazebo::CameraIface();
   pr2PTZCameraLeftIface   = new gazebo::CameraIface();
   pr2PTZCameraRightIface  = new gazebo::CameraIface();
   pr2PTZLeftIface         = new gazebo::PTZIface();
   pr2PTZRightIface        = new gazebo::PTZIface();
   pr2WristCameraLeftIface     = new gazebo::CameraIface();
   pr2WristCameraRightIface    = new gazebo::CameraIface();
   pr2ForearmCameraLeftIface   = new gazebo::CameraIface();
   pr2ForearmCameraRightIface  = new gazebo::CameraIface();

   pr2LeftWristIface       = new gazebo::PositionIface();
   pr2RightWristIface      = new gazebo::PositionIface();
   pr2BaseIface            = new gazebo::PositionIface();

   posObjectIface            = new gazebo::PositionIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    client = NULL;
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    simIface = NULL;
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
    pr2Iface = NULL;
  }

  /// Open the PTZ Position interface
  try
  {
    pr2PTZLeftIface->Open(client, "ptz_left_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 Left PTZ interface\n"
    << e << "\n";
    pr2PTZLeftIface = NULL;
  }

  /// Open the PTZ Position interface
  try
  {
    pr2PTZRightIface->Open(client, "ptz_right_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 Right PTZ interface\n"
    << e << "\n";
    pr2PTZRightIface = NULL;
  }

  /// Open the wrist and forearm cameras
  try
  {
    pr2WristCameraLeftIface->Open(client, "wrist_cam_left_iface_1");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 Left wrist camera interface\n"
    << e << "\n";
    pr2WristCameraLeftIface = NULL;
  }

  try
  {
    pr2WristCameraRightIface->Open(client, "wrist_cam_right_iface_1");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 Right wrist camera interface\n"
    << e << "\n";
    pr2WristCameraRightIface = NULL;
  }

  try
  {
    pr2ForearmCameraLeftIface->Open(client, "forearm_cam_left_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 Left forearm camera interface\n"
    << e << "\n";
    pr2ForearmCameraLeftIface = NULL;
  }

  try
  {
    pr2ForearmCameraRightIface->Open(client, "forearm_cam_right_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 Right forearm camera interface\n"
    << e << "\n";
    pr2ForearmCameraRightIface = NULL;
  }

  /// Open the laser interface for hokuyo
  try
  {
    pr2LaserIface->Open(client, "tilt_laser_iface_1");
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


  /// Open the camera interfaces
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
    pr2PTZCameraLeftIface->Open(client, "ptz_cam_left_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 camera interface\n"
    << e << "\n";
    pr2PTZCameraLeftIface = NULL;
  }

  try
  {
    pr2PTZCameraRightIface->Open(client, "ptz_cam_right_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 camera interface\n"
    << e << "\n";
    pr2PTZCameraRightIface = NULL;
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

  try
  {
    posObjectIface->Open(client, "p3d_object_position");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the object position interface\n"
    << e << "\n";
    posObjectIface = NULL;
  }

  std::cout << "initial HW reads\n" << std::endl;
  // fill in actuator data
  if (pr2Iface)
  for (int id = 0; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    pr2Iface->Lock(1);
    this->jointData[id].cmdEnableMotor    =  pr2Iface->data->actuators[id].cmdEnableMotor    ;
    this->jointData[id].controlMode       =  pr2Iface->data->actuators[id].controlMode       ;
    this->jointData[id].pGain             =  pr2Iface->data->actuators[id].pGain             ;
    this->jointData[id].iGain             =  pr2Iface->data->actuators[id].iGain             ;
    this->jointData[id].dGain             =  pr2Iface->data->actuators[id].dGain             ;
    this->jointData[id].cmdPosition       =  pr2Iface->data->actuators[id].cmdPosition       ;
    this->jointData[id].cmdSpeed          =  pr2Iface->data->actuators[id].cmdSpeed          ;
    this->jointData[id].cmdEffectorForce  =  pr2Iface->data->actuators[id].cmdEffectorForce  ;
    pr2Iface->Unlock();
  }


  GetSimTime(&(this->lastTiltLaserTime));
  GetSimTime(&(this->lastBaseLaserTime));
  GetSimTime(&(this->lastCameraGlobalTime       ));
  GetSimTime(&(this->lastPTZCameraLeftTime      ));
  GetSimTime(&(this->lastPTZCameraRightTime     ));
  GetSimTime(&(this->lastWristCameraLeftTime    ));
  GetSimTime(&(this->lastWristCameraRightTime   ));
  GetSimTime(&(this->lastForearmCameraLeftTime  ));
  GetSimTime(&(this->lastForearmCameraRightTime ));

  return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2HW::GetSimTime(double *sim_time)
{
   *sim_time = simIface->data->simTime;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::AddJoint(PR2_JOINT_ID id)
{
  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::EnableModel(PR2_MODEL_ID id)
{
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      this->jointData[ii].cmdEnableMotor = 1;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::DisableModel(PR2_MODEL_ID id)
{
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      this->jointData[ii].cmdEnableMotor = 0;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::IsEnabledModel(PR2_MODEL_ID id, int *enabled)
{
   int isEnabled = 1;
   for(int ii = JointStart[id]; ii <= JointEnd[id]; ii++)
      isEnabled = this->jointData[ii].cmdEnableMotor && isEnabled;
   *enabled = isEnabled;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::SetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE mode)
{
   this->jointData[id].controlMode = mode;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetJointControlMode(PR2_JOINT_ID id, PR2_JOINT_CONTROL_MODE *mode)
{
   *mode = (PR2::PR2_JOINT_CONTROL_MODE)(this->jointData[id].controlMode);
   return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2HW::SetJointGains(PR2_JOINT_ID id, double pGain, double iGain, double dGain)
{
   this->jointData[id].pGain = pGain;
   this->jointData[id].iGain = iGain;
   this->jointData[id].dGain = dGain;
   return PR2_ALL_OK;

};

PR2_ERROR_CODE PR2HW::GetJointGains(PR2_JOINT_ID id, double *pGain, double *iGain, double *dGain)
{
   *pGain = this->jointData[id].pGain;
   *iGain = this->jointData[id].iGain;
   *dGain = this->jointData[id].dGain;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::EnableJoint(PR2_JOINT_ID id)
{
   this->jointData[id].cmdEnableMotor = 1;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::DisableJoint(PR2_JOINT_ID id)
{
   this->jointData[id].cmdEnableMotor = 0;
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::IsEnabledJoint(PR2_JOINT_ID id, int *enabled)
{
   *enabled = this->jointData[id].cmdEnableMotor;
   return PR2_ALL_OK;
};

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
  this->jointData[id].cmdPosition = jointPosition;
  this->jointData[id].cmdSpeed    = jointSpeed;
  return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2HW::GetJointServoCmd(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{
  *jointPosition =  this->jointData[id].cmdPosition;
  *jointSpeed    =  this->jointData[id].cmdSpeed;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetJointServoActual(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{
  *jointPosition =  this->jointData[id].actualPosition;
  *jointSpeed    =  this->jointData[id].actualSpeed;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetJointPositionActual(PR2_JOINT_ID id, double *jointPosition, double *jointSpeed)
{
  *jointPosition =  this->jointData[id].actualPosition;
  *jointSpeed    =  this->jointData[id].actualSpeed;
  return PR2_ALL_OK;
};


PR2_ERROR_CODE PR2HW::SetJointTorque(PR2_JOINT_ID id, double torque)
{
  this->jointData[id].cmdEffectorForce = torque;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetJointTorqueCmd(PR2_JOINT_ID id, double *torque)
{
  *torque = this->jointData[id].cmdEffectorForce;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetJointTorqueActual(PR2_JOINT_ID id, double *torque)
{
  *torque = this->jointData[id].actualEffectorForce;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::SetJointSpeed(PR2_JOINT_ID id, double speed)
{
  this->jointData[id].cmdSpeed = speed;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetJointSpeedCmd(PR2_JOINT_ID id, double *speed)
{
  *speed = this->jointData[id].cmdSpeed;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetJointSpeedActual(PR2_JOINT_ID id, double *speed)
{
  *speed = this->jointData[id].cmdSpeed;
  return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetLaserRanges(PR2_SENSOR_ID id,
    float* angle_min, float* angle_max, float* angle_increment,
    float* range_max,uint32_t* ranges_size     ,uint32_t* ranges_alloc_size,
                     uint32_t* intensities_size,uint32_t* intensities_alloc_size,
                     float*    ranges          ,uint8_t*  intensities,
    double* simTime)
{

  gazebo::LaserIface       *tmpLaserIface;

  switch (id)
  {
    case LASER_HEAD:
      if (pr2LaserIface->Lock(1))
      {
        if (pr2LaserIface->data->head.time == this->lastTiltLaserTime)
        {
          tmpLaserIface = NULL;
        }
        else
        {
          tmpLaserIface = pr2LaserIface;
          this->lastTiltLaserTime = pr2LaserIface->data->head.time;
        }
        pr2LaserIface->Unlock();
      }
      else
        tmpLaserIface = NULL;
      break;
    case LASER_BASE:
      if (pr2BaseLaserIface->Lock(1))
      {
        if (pr2BaseLaserIface->data->head.time == this->lastBaseLaserTime)
        {
          tmpLaserIface = NULL;
        }
        else
        {
          tmpLaserIface = pr2BaseLaserIface;
          this->lastBaseLaserTime = pr2BaseLaserIface->data->head.time;
        }
        pr2BaseLaserIface->Unlock();
      }
      else
        tmpLaserIface = NULL;
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
    *simTime                 = tmpLaserIface->data->head.time;
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

PR2_ERROR_CODE PR2HW::GetCameraImage(PR2_SENSOR_ID id ,
                     uint32_t*    width                 ,uint32_t*    height                ,
                     uint32_t*    depth                 ,
                     std::string* compression           ,std::string* colorspace            ,
                     uint32_t*    data_size             ,void*        buf                   ,
                     double* simTime)
{

    gazebo::CameraIface      *tmpCameraIface;

    switch(id)
    {
      case CAMERA_GLOBAL:
          if (pr2CameraGlobalIface && pr2CameraGlobalIface->Lock(1))
          {
            if (pr2CameraGlobalIface->data->head.time == this->lastCameraGlobalTime)
            {
              tmpCameraIface = NULL;
            }
            else
            {
              tmpCameraIface = pr2CameraGlobalIface;
              this->lastCameraGlobalTime = pr2CameraGlobalIface->data->head.time;
            }
            pr2CameraGlobalIface->Unlock();
          }
          else
            tmpCameraIface = NULL;

          break;
      case CAMERA_HEAD_LEFT:
          if (pr2PTZCameraLeftIface && pr2PTZCameraLeftIface->Lock(1))
          {
            if (pr2PTZCameraLeftIface->data->head.time == this->lastPTZCameraLeftTime)
            {
              tmpCameraIface = NULL;
            }
            else
            {
              tmpCameraIface = pr2PTZCameraLeftIface;
              this->lastPTZCameraLeftTime = pr2PTZCameraLeftIface->data->head.time;
            }
            pr2PTZCameraLeftIface->Unlock();
          }
          else
            tmpCameraIface = NULL;

          break;
      case CAMERA_HEAD_RIGHT:
          if (pr2PTZCameraRightIface && pr2PTZCameraRightIface->Lock(1))
          {
            if (pr2PTZCameraRightIface->data->head.time == this->lastPTZCameraRightTime)
            {
              tmpCameraIface = NULL;
            }
            else
            {
              tmpCameraIface = pr2PTZCameraRightIface;
              this->lastPTZCameraRightTime = pr2PTZCameraRightIface->data->head.time;
            }
            pr2PTZCameraRightIface->Unlock();
          }
          else
            tmpCameraIface = NULL;

          break;
      case CAMERA_WRIST_LEFT:
          if (pr2WristCameraLeftIface && pr2WristCameraLeftIface->Lock(1))
          {
            if (pr2WristCameraLeftIface->data->head.time == this->lastWristCameraLeftTime)
            {
              tmpCameraIface = NULL;
            }
            else
            {
              tmpCameraIface = pr2WristCameraLeftIface;
              this->lastWristCameraLeftTime = pr2WristCameraLeftIface->data->head.time;
            }
            pr2WristCameraLeftIface->Unlock();
          }
          else
            tmpCameraIface = NULL;

          break;
      case CAMERA_WRIST_RIGHT:
          if (pr2WristCameraRightIface && pr2WristCameraRightIface->Lock(1))
          {
            if (pr2WristCameraRightIface->data->head.time == this->lastWristCameraRightTime)
            {
              tmpCameraIface = NULL;
            }
            else
            {
              tmpCameraIface = pr2WristCameraRightIface;
              this->lastWristCameraRightTime = pr2WristCameraRightIface->data->head.time;
            }
            pr2WristCameraRightIface->Unlock();
          }
          else
            tmpCameraIface = NULL;

          break;
      case CAMERA_FOREARM_LEFT:
          if (pr2ForearmCameraLeftIface && pr2ForearmCameraLeftIface->Lock(1))
          {
            if (pr2ForearmCameraLeftIface->data->head.time == this->lastForearmCameraLeftTime)
            {
              tmpCameraIface = NULL;
            }
            else
            {
              tmpCameraIface = pr2ForearmCameraLeftIface;
              this->lastForearmCameraLeftTime = pr2ForearmCameraLeftIface->data->head.time;
            }
            pr2ForearmCameraLeftIface->Unlock();
          }
          else
            tmpCameraIface = NULL;

          break;
      case CAMERA_FOREARM_RIGHT:
          if (pr2ForearmCameraRightIface && pr2ForearmCameraRightIface->Lock(1))
          {
            if (pr2ForearmCameraRightIface->data->head.time == this->lastForearmCameraRightTime)
            {
              tmpCameraIface = NULL;
            }
            else
            {
              tmpCameraIface = pr2ForearmCameraRightIface;
              this->lastForearmCameraRightTime = pr2ForearmCameraRightIface->data->head.time;
            }
            pr2ForearmCameraRightIface->Unlock();
          }
          else
            tmpCameraIface = NULL;

          break;
      default:
          tmpCameraIface = NULL;
          break;
    }

  if (tmpCameraIface == NULL)
  {
    return PR2_ERROR;
  }
  else
  {
    tmpCameraIface->Lock(1);
    *simTime      = tmpCameraIface->data->head.time;
    *width        = (uint32_t)tmpCameraIface->data->width;
    *height       = (uint32_t)tmpCameraIface->data->height;
    *compression  = "raw";
    *colorspace   = "rgb24"; //"mono";
    *data_size    = tmpCameraIface->data->image_size;

    // on first pass, the sensor does not update after cameraIface is opened.
    if (*data_size > 0)
    {
      *depth        = (*data_size)/((*width)*(*height));

      uint32_t       buf_size = (*width) * (*height) * (*depth);

      // copy the image into local buffer
#if 1
      //buf = (void*)(tmpCameraIface->data->image);
      memcpy(buf,tmpCameraIface->data->image,buf_size);
#else
      for (uint32_t i = 0; i < buf_size ; i=i+3)
      {
        // flip red and blue
        ((unsigned char*)buf)[i  ] = tmpCameraIface->data->image[i+2];
        ((unsigned char*)buf)[i+1] = tmpCameraIface->data->image[i+1];
        ((unsigned char*)buf)[i+2] = tmpCameraIface->data->image[i  ];
        //printf("%d %d\n",i,tmpCameraIface->data->image[i]);
      }
#endif
    }
    tmpCameraIface->Unlock();

    return PR2_ALL_OK;
  }
};

PR2_ERROR_CODE PR2HW::GetWristPoseGroundTruth(PR2_MODEL_ID id, double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
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


NEWMAT::Matrix PR2HW::GetWristPoseGroundTruth(PR2_MODEL_ID id)
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

PR2_ERROR_CODE PR2HW::GetBasePositionGroundTruth(double* x, double* y, double *z, double *roll, double *pitch, double *yaw)
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

PR2_ERROR_CODE PR2HW::GetObjectPositionGroundTruth(double* x, double* y, double *z, double *roll, double *pitch, double *yaw)
{
	if(posObjectIface == NULL)
		return PR2_ALL_OK;

   posObjectIface->Lock(1);
   *x     = posObjectIface->data->pose.pos.x;
   *y     = posObjectIface->data->pose.pos.y;
   *z     = posObjectIface->data->pose.pos.z;
   *roll  = posObjectIface->data->pose.roll;
   *pitch = posObjectIface->data->pose.pitch;
   *yaw   = posObjectIface->data->pose.yaw;
   posObjectIface->Unlock();
   return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::ClientWait()
{
  // block until simulator update.
  client->Wait();
  return PR2_ALL_OK;
}

PR2_ERROR_CODE PR2HW::UpdateHW()
{
  //std::cout << "updating HW receive\n" << std::endl;
  // receive data from hardware
  if (pr2Iface)
  for (int id = 0; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    pr2Iface->Lock(1);
    this->jointData[id].actualPosition              = pr2Iface->data->actuators[id].actualPosition     ;
    this->jointData[id].actualSpeed                 = pr2Iface->data->actuators[id].actualSpeed        ;
    this->jointData[id].actualEffectorForce         = pr2Iface->data->actuators[id].actualEffectorForce;
    pr2Iface->Unlock();
  }


  //std::cout << "updating HW send\n" << std::endl;
  // send commands to hardware
  if (pr2Iface)
  for (int id = 0; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    pr2Iface->Lock(1);
    pr2Iface->data->actuators[id].cmdEnableMotor        = this->jointData[id].cmdEnableMotor  ;
    pr2Iface->data->actuators[id].controlMode           = this->jointData[id].controlMode     ;
    pr2Iface->data->actuators[id].pGain                 = this->jointData[id].pGain           ;
    pr2Iface->data->actuators[id].iGain                 = this->jointData[id].iGain           ;
    pr2Iface->data->actuators[id].dGain                 = this->jointData[id].dGain           ;
    pr2Iface->data->actuators[id].cmdPosition           = this->jointData[id].cmdPosition     ;
    pr2Iface->data->actuators[id].cmdSpeed              = this->jointData[id].cmdSpeed        ;
    pr2Iface->data->actuators[id].cmdEffectorForce      = this->jointData[id].cmdEffectorForce;
    pr2Iface->Unlock();
  }


  return PR2_ALL_OK;
}

//Updates the joint array using the new architecture
PR2_ERROR_CODE PR2HW::UpdateJointArray(mechanism::Joint** jointArray)
{
 // std::cout << "updating Joint receive\n" << std::endl;
  // receive data from hardware
  if (pr2Iface)
  for (int id = 0; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    pr2Iface->Lock(1);
    jointArray[id]->position              = pr2Iface->data->actuators[id].actualPosition     ;
    jointArray[id]->velocity                 = pr2Iface->data->actuators[id].actualSpeed        ;
    jointArray[id]->appliedEffort         = pr2Iface->data->actuators[id].actualEffectorForce;
    pr2Iface->Unlock();
  }


 // std::cout << "updating Joint send\n" << std::endl;
  // send commands to hardware
  if (pr2Iface)
  for (int id = 0; id < PR2::HEAD_PTZ_R_TILT; id++)
  {

    pr2Iface->Lock(1);
    pr2Iface->data->actuators[id].cmdEnableMotor     = jointArray[id]->initialized  ;
    /*
    pr2Iface->data->actuators[id].controlMode        = this->jointData[id].controlMode     ;
    pr2Iface->data->actuators[id].pGain              = this->jointData[id].pGain           ;
    pr2Iface->data->actuators[id].iGain              = this->jointData[id].iGain           ;
    pr2Iface->data->actuators[id].dGain              = this->jointData[id].dGain           ;
    pr2Iface->data->actuators[id].cmdPosition        = this->jointData[id].cmdPosition     ;
    pr2Iface->data->actuators[id].cmdSpeed           = this->jointData[id].cmdSpeed        ;
    */
    pr2Iface->data->actuators[id].cmdEffectorForce   = jointArray[id]->commandedEffort;
    pr2Iface->Unlock();
  }


  return PR2_ALL_OK;
}

