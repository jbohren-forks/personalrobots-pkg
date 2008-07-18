
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

  std::cout << "initial HW reads\n" << std::endl;
  // fill in actuator data
  for (int id = PR2::CASTER_FL_STEER; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    if(IsHead((PR2::PR2_JOINT_ID)id))
    {
      pr2HeadIface->Lock(1);
      this->jointData[id].cmdEnableMotor    =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEnableMotor     ;
      this->jointData[id].controlMode       =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].controlMode        ;
      this->jointData[id].pGain             =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].pGain              ;
      this->jointData[id].iGain             =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].iGain              ;
      this->jointData[id].dGain             =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].dGain              ;
      this->jointData[id].cmdPosition       =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdPosition        ;
      this->jointData[id].cmdSpeed          =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdSpeed           ;
      this->jointData[id].cmdEffectorForce  =  pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEffectorForce   ;
      pr2HeadIface->Unlock();
    }
    else if(IsGripperLeft((PR2::PR2_JOINT_ID)id))
    {
      pr2GripperLeftIface->Lock(1);
      this->jointData[id].cmdEnableMotor    =  pr2GripperLeftIface->data->cmdEnableMotor   ;
      this->jointData[id].controlMode       =  pr2GripperLeftIface->data->controlMode      ;
      this->jointData[id].pGain             =  pr2GripperLeftIface->data->pGain            ;
      this->jointData[id].iGain             =  pr2GripperLeftIface->data->iGain            ;
      this->jointData[id].dGain             =  pr2GripperLeftIface->data->dGain            ;
      this->jointData[id].cmdGap            =  pr2GripperLeftIface->data->cmdGap           ;
      this->jointData[id].cmdEffectorForce  =  pr2GripperLeftIface->data->cmdForce         ;
      this->jointData[id].cmdSpeed          =  pr2GripperLeftIface->data->cmdPositionRate  ;
      pr2GripperLeftIface->Unlock();
    }
    else if(IsGripperRight((PR2::PR2_JOINT_ID)id))
    {
      pr2GripperRightIface->Lock(1);
      this->jointData[id].cmdEnableMotor    =  pr2GripperRightIface->data->cmdEnableMotor   ;
      this->jointData[id].controlMode       =  pr2GripperRightIface->data->controlMode      ;
      this->jointData[id].pGain             =  pr2GripperRightIface->data->pGain            ;
      this->jointData[id].iGain             =  pr2GripperRightIface->data->iGain            ;
      this->jointData[id].dGain             =  pr2GripperRightIface->data->dGain            ;
      this->jointData[id].cmdGap            =  pr2GripperRightIface->data->cmdGap           ;
      this->jointData[id].cmdEffectorForce  =  pr2GripperRightIface->data->cmdForce         ;
      this->jointData[id].cmdSpeed          =  pr2GripperRightIface->data->cmdPositionRate  ;
      pr2GripperRightIface->Unlock();
    }
    else // corresponds to Pr2_Actarray from CASTER to ARM_R_GRIPPER
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
  }


  GetSimTime(&(this->lastTiltLaserTime));
  GetSimTime(&(this->lastBaseLaserTime));

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
   this->jointData[id-JointStart[HEAD]].pGain = pGain;
   this->jointData[id-JointStart[HEAD]].iGain = iGain;
   this->jointData[id-JointStart[HEAD]].dGain = dGain;
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
                     float*    ranges          ,uint8_t*  intensities)
{

  gazebo::LaserIface       *tmpLaserIface;

  double tmpTime;
  GetSimTime(&tmpTime);

  switch (id)
  {
    case LASER_HEAD:
      if (tmpTime == this->lastTiltLaserTime)
        tmpLaserIface = NULL;
      else
      {
        tmpLaserIface = pr2LaserIface;
        this->lastTiltLaserTime = tmpTime;
      }
      break;
    case LASER_BASE:
      if (tmpTime == this->lastBaseLaserTime)
        tmpLaserIface = NULL;
      else
      {
        tmpLaserIface = pr2BaseLaserIface;
        this->lastBaseLaserTime = tmpTime;
      }
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

PR2_ERROR_CODE PR2HW::OpenGripper(PR2_MODEL_ID id,double gap,double force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
       case PR2_RIGHT_GRIPPER:
          for (int ii = JointStart[id];ii <= JointEnd[id];ii++)
          {
            this->jointData[ii].cmdMode          = GAZEBO_PR2GRIPPER_CMD_OPEN;
            this->jointData[ii].cmdSpeed         = 1.0;
            this->jointData[ii].cmdEffectorForce = force;
            this->jointData[ii].cmdGap           = gap;
          }
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::CloseGripper(PR2_MODEL_ID id,double gap,double force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
       case PR2_RIGHT_GRIPPER:
	 std::cout << "JointStart " << JointStart[id] << " joint end " << JointEnd[id] << std::endl;
         for (int ii = JointStart[id];ii <= JointEnd[id];ii++)
          {
            this->jointData[ii].cmdMode          = GAZEBO_PR2GRIPPER_CMD_OPEN;
            this->jointData[ii].cmdSpeed         = 1.0;
            this->jointData[ii].cmdEffectorForce = force;
            this->jointData[ii].cmdGap           = gap;
	    std::cout << "Setting gap for " << ii << " " << gap << std::endl;
          }
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetGripperCmd(PR2_MODEL_ID id,double *gap,double *force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
       case PR2_RIGHT_GRIPPER:
          for (int ii = JointStart[id];ii <= JointEnd[id];ii++)
          {
            *force = this->jointData[ii].cmdEffectorForce;
            *gap   = this->jointData[ii].cmdGap          ;
          }
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetGripperActual(PR2_MODEL_ID id,double *gap,double *force)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
       case PR2_RIGHT_GRIPPER:
          for (int ii = JointStart[id];ii <= JointEnd[id];ii++)
          {
            *force = this->jointData[ii].actualEffectorForce;
            *gap   = this->jointData[ii].actualGap          ;
          }
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::SetGripperGains(PR2_MODEL_ID id,double p,double i, double d)
{
    switch(id)
    {
       case PR2_LEFT_GRIPPER:
       case PR2_RIGHT_GRIPPER:
          for (int ii = JointStart[id];ii <= JointEnd[id];ii++)
          {
            this->jointData[ii].pGain = p;
            this->jointData[ii].iGain = i;
            this->jointData[ii].dGain = d;
          }
          break;
       default:
          break;
    }
    return PR2_ALL_OK;
};

PR2_ERROR_CODE PR2HW::GetCameraImage(PR2_SENSOR_ID id ,
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
  for (int id = PR2::CASTER_FL_STEER; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    if(IsHead((PR2::PR2_JOINT_ID)id))
    {
      pr2HeadIface->Lock(1);
      this->jointData[id].actualPosition              = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualPosition     ;
      this->jointData[id].actualSpeed                 = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualSpeed        ;
      this->jointData[id].actualEffectorForce         = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualEffectorForce;
      pr2HeadIface->Unlock();
    }
    else if(IsGripperLeft((PR2::PR2_JOINT_ID)id))
    {
      pr2GripperLeftIface->Lock(1);
      this->jointData[id].actualPosition              = ( pr2GripperLeftIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperLeftIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualGap                   = ( pr2GripperLeftIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperLeftIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualSpeed                 = ( pr2GripperLeftIface->data->actualFingerPositionRate[0])  // FIXME: not debugged
                                                       +(-pr2GripperLeftIface->data->actualFingerPositionRate[1]);
      this->jointData[id].actualSpeed                 = pr2GripperLeftIface->data->actualFingerPositionRate[0]; // FIXME: temporary numbers

      this->jointData[id].actualEffectorForce         = pr2GripperLeftIface->data->gripperForce;
      this->jointData[id].actualFingerPosition[0]     = pr2GripperLeftIface->data->actualFingerPosition[0];
      this->jointData[id].actualFingerPosition[1]     = pr2GripperLeftIface->data->actualFingerPosition[1];
      this->jointData[id].actualFingerPositionRate[0] = pr2GripperLeftIface->data->actualFingerPositionRate[0];
      this->jointData[id].actualFingerPositionRate[1] = pr2GripperLeftIface->data->actualFingerPositionRate[1];
      pr2GripperLeftIface->Unlock();
    }
    else if(IsGripperRight((PR2::PR2_JOINT_ID)id))
    {
      pr2GripperRightIface->Lock(1);
      this->jointData[id].actualPosition              = ( pr2GripperRightIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperRightIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualGap                   = ( pr2GripperRightIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperRightIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualSpeed                 = ( pr2GripperRightIface->data->actualFingerPositionRate[0])  // FIXME: not debugged
                                                       +(-pr2GripperRightIface->data->actualFingerPositionRate[1]);
      this->jointData[id].actualSpeed                 = pr2GripperRightIface->data->actualFingerPositionRate[0]; // FIXME: temporary numbers

      this->jointData[id].actualEffectorForce         = pr2GripperRightIface->data->gripperForce;
      this->jointData[id].actualFingerPosition[0]     = pr2GripperRightIface->data->actualFingerPosition[0];
      this->jointData[id].actualFingerPosition[1]     = pr2GripperRightIface->data->actualFingerPosition[1];
      this->jointData[id].actualFingerPositionRate[0] = pr2GripperRightIface->data->actualFingerPositionRate[0];
      this->jointData[id].actualFingerPositionRate[1] = pr2GripperRightIface->data->actualFingerPositionRate[1];
      pr2GripperRightIface->Unlock();
    }
    else // corresponds to Pr2_Actarray from CASTER to ARM_R_GRIPPER
    {
      pr2Iface->Lock(1);
      this->jointData[id].actualPosition              = pr2Iface->data->actuators[id].actualPosition     ;
      this->jointData[id].actualSpeed                 = pr2Iface->data->actuators[id].actualSpeed        ;
      this->jointData[id].actualEffectorForce         = pr2Iface->data->actuators[id].actualEffectorForce;
      pr2Iface->Unlock();
    }
  }


  //std::cout << "updating HW send\n" << std::endl;
  // send commands to hardware
  for (int id = PR2::CASTER_FL_STEER; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    if(IsHead((PR2::PR2_JOINT_ID)id))
    {
      pr2HeadIface->Lock(1);
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEnableMotor   = this->jointData[id].cmdEnableMotor  ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].controlMode      = this->jointData[id].controlMode     ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].pGain            = this->jointData[id].pGain           ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].iGain            = this->jointData[id].iGain           ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].dGain            = this->jointData[id].dGain           ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdPosition      = this->jointData[id].cmdPosition     ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdSpeed         = this->jointData[id].cmdSpeed        ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEffectorForce = this->jointData[id].cmdEffectorForce;
      pr2HeadIface->Unlock();
    }
    else if(IsGripperLeft((PR2::PR2_JOINT_ID)id))
    {
      pr2GripperLeftIface->Lock(1);
      pr2GripperLeftIface->data->cmdEnableMotor                           = this->jointData[id].cmdEnableMotor  ;
      pr2GripperLeftIface->data->controlMode                              = this->jointData[id].controlMode     ;
      pr2GripperLeftIface->data->pGain                                    = this->jointData[id].pGain           ;
      pr2GripperLeftIface->data->iGain                                    = this->jointData[id].iGain           ;
      pr2GripperLeftIface->data->dGain                                    = this->jointData[id].dGain           ;
      pr2GripperLeftIface->data->cmdGap                                   = this->jointData[id].cmdGap          ;
      pr2GripperLeftIface->data->cmdForce                                 = this->jointData[id].cmdEffectorForce;
      pr2GripperLeftIface->data->cmdPositionRate                          = this->jointData[id].cmdSpeed;
      pr2GripperLeftIface->Unlock();
    }
    else if(IsGripperRight((PR2::PR2_JOINT_ID)id))
    {
      pr2GripperRightIface->Lock(1);
      pr2GripperRightIface->data->cmdEnableMotor                          = this->jointData[id].cmdEnableMotor  ;
      pr2GripperRightIface->data->controlMode                             = this->jointData[id].controlMode     ;
      pr2GripperRightIface->data->pGain                                   = this->jointData[id].pGain           ;
      pr2GripperRightIface->data->iGain                                   = this->jointData[id].iGain           ;
      pr2GripperRightIface->data->dGain                                   = this->jointData[id].dGain           ;
      pr2GripperRightIface->data->cmdGap                                  = this->jointData[id].cmdGap          ;
      pr2GripperRightIface->data->cmdForce                                = this->jointData[id].cmdEffectorForce;
      pr2GripperRightIface->data->cmdPositionRate                         = this->jointData[id].cmdSpeed;
      pr2GripperRightIface->Unlock();
    }
    else // corresponds to Pr2_Actarray from CASTER to ARM_R_GRIPPER
    {
      pr2Iface->Lock(1);
      pr2Iface->data->actuators[id].cmdEnableMotor                        = this->jointData[id].cmdEnableMotor  ;
      pr2Iface->data->actuators[id].controlMode                           = this->jointData[id].controlMode     ;
      pr2Iface->data->actuators[id].pGain                                 = this->jointData[id].pGain           ;
      pr2Iface->data->actuators[id].iGain                                 = this->jointData[id].iGain           ;
      pr2Iface->data->actuators[id].dGain                                 = this->jointData[id].dGain           ;
      pr2Iface->data->actuators[id].cmdPosition                           = this->jointData[id].cmdPosition     ;
      pr2Iface->data->actuators[id].cmdSpeed                              = this->jointData[id].cmdSpeed        ;
      pr2Iface->data->actuators[id].cmdEffectorForce                      = this->jointData[id].cmdEffectorForce;
      pr2Iface->Unlock();
    }
  }


  return PR2_ALL_OK;
}

//Updates the joint array using the new architecture
PR2_ERROR_CODE PR2HW::UpdateJointArray(mechanism::Joint** jointArray)
{
 // std::cout << "updating Joint receive\n" << std::endl;
  // receive data from hardware
  for (int id = PR2::CASTER_FL_STEER; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    if(IsHead((PR2::PR2_JOINT_ID)id))
    {
      
      pr2HeadIface->Lock(1);
      jointArray[id]->position              = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualPosition     ;
      jointArray[id]->velocity                = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualSpeed        ;
     jointArray[id]->appliedEffort         = pr2HeadIface->data->actuators[id-JointStart[HEAD]].actualEffectorForce;
      pr2HeadIface->Unlock();
      
    }
    else if(IsGripperLeft((PR2::PR2_JOINT_ID)id))
    {
     /* //Don't look at grippers for now
       pr2GripperLeftIface->Lock(1);
      this->jointArray[id].actualPosition              = ( pr2GripperLeftIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperLeftIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualGap                   = ( pr2GripperLeftIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperLeftIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualSpeed                 = ( pr2GripperLeftIface->data->actualFingerPositionRate[0])  // FIXME: not debugged
                                                       +(-pr2GripperLeftIface->data->actualFingerPositionRate[1]);
      this->jointData[id].actualSpeed                 = pr2GripperLeftIface->data->actualFingerPositionRate[0]; // FIXME: temporary numbers

      this->jointData[id].actualEffectorForce         = pr2GripperLeftIface->data->gripperForce;
      this->jointData[id].actualFingerPosition[0]     = pr2GripperLeftIface->data->actualFingerPosition[0];
      this->jointData[id].actualFingerPosition[1]     = pr2GripperLeftIface->data->actualFingerPosition[1];
      this->jointData[id].actualFingerPositionRate[0] = pr2GripperLeftIface->data->actualFingerPositionRate[0];
      this->jointData[id].actualFingerPositionRate[1] = pr2GripperLeftIface->data->actualFingerPositionRate[1];
      
      pr2GripperLeftIface->Unlock();
      */
    }
    else if(IsGripperRight((PR2::PR2_JOINT_ID)id))
    {
      /*
      pr2GripperRightIface->Lock(1);
      this->jointData[id].actualPosition              = ( pr2GripperRightIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperRightIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualGap                   = ( pr2GripperRightIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
                                                       +(-pr2GripperRightIface->data->actualFingerPosition[1] + 0.015);
      this->jointData[id].actualSpeed                 = ( pr2GripperRightIface->data->actualFingerPositionRate[0])  // FIXME: not debugged
                                                       +(-pr2GripperRightIface->data->actualFingerPositionRate[1]);
      this->jointData[id].actualSpeed                 = pr2GripperRightIface->data->actualFingerPositionRate[0]; // FIXME: temporary numbers

      this->jointData[id].actualEffectorForce         = pr2GripperRightIface->data->gripperForce;
      this->jointData[id].actualFingerPosition[0]     = pr2GripperRightIface->data->actualFingerPosition[0];
      this->jointData[id].actualFingerPosition[1]     = pr2GripperRightIface->data->actualFingerPosition[1];
      this->jointData[id].actualFingerPositionRate[0] = pr2GripperRightIface->data->actualFingerPositionRate[0];
      this->jointData[id].actualFingerPositionRate[1] = pr2GripperRightIface->data->actualFingerPositionRate[1];
      pr2GripperRightIface->Unlock();
      */
    }
    else // corresponds to Pr2_Actarray from CASTER to ARM_R_GRIPPER
    {
      pr2Iface->Lock(1);
      jointArray[id]->position              = pr2Iface->data->actuators[id].actualPosition     ;
      jointArray[id]->velocity                 = pr2Iface->data->actuators[id].actualSpeed        ;
      jointArray[id]->appliedEffort         = pr2Iface->data->actuators[id].actualEffectorForce;
      pr2Iface->Unlock();
    }
  }


 // std::cout << "updating Joint send\n" << std::endl;
  // send commands to hardware
  for (int id = PR2::CASTER_FL_STEER; id < PR2::HEAD_PTZ_R_TILT; id++)
  {
    if(IsHead((PR2::PR2_JOINT_ID)id))
    {
      pr2HeadIface->Lock(1);
      
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEnableMotor   = jointArray[id]->initialized;
      /*
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].controlMode      = this->jointData[id].controlMode     ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].pGain            = this->jointData[id].pGain           ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].iGain            = this->jointData[id].iGain           ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].dGain            = this->jointData[id].dGain           ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdPosition      = this->jointData[id].cmdPosition     ;
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdSpeed         = this->jointData[id].cmdSpeed        ;
      */
      pr2HeadIface->data->actuators[id-JointStart[HEAD]].cmdEffectorForce = jointArray[id]->commandedEffort;
      pr2HeadIface->Unlock();
    }
    else if(IsGripperLeft((PR2::PR2_JOINT_ID)id))
    {
      /*
      pr2GripperLeftIface->Lock(1);
      pr2GripperLeftIface->data->cmdEnableMotor                           = this->jointData[id].cmdEnableMotor  ;
      pr2GripperLeftIface->data->controlMode                              = this->jointData[id].controlMode     ;
      pr2GripperLeftIface->data->pGain                                    = this->jointData[id].pGain           ;
      pr2GripperLeftIface->data->iGain                                    = this->jointData[id].iGain           ;
      pr2GripperLeftIface->data->dGain                                    = this->jointData[id].dGain           ;
      pr2GripperLeftIface->data->cmdGap                                   = this->jointData[id].cmdGap          ;
      pr2GripperLeftIface->data->cmdForce                                 = this->jointData[id].cmdEffectorForce;
      pr2GripperLeftIface->data->cmdPositionRate                          = this->jointData[id].cmdSpeed;
      pr2GripperLeftIface->Unlock();
      */
    }
    else if(IsGripperRight((PR2::PR2_JOINT_ID)id))
    {
      /*
      pr2GripperRightIface->Lock(1);
      pr2GripperRightIface->data->cmdEnableMotor                          = this->jointData[id].cmdEnableMotor  ;
      pr2GripperRightIface->data->controlMode                             = this->jointData[id].controlMode     ;
      pr2GripperRightIface->data->pGain                                   = this->jointData[id].pGain           ;
      pr2GripperRightIface->data->iGain                                   = this->jointData[id].iGain           ;
      pr2GripperRightIface->data->dGain                                   = this->jointData[id].dGain           ;
      pr2GripperRightIface->data->cmdGap                                  = this->jointData[id].cmdGap          ;
      pr2GripperRightIface->data->cmdForce                                = this->jointData[id].cmdEffectorForce;
      pr2GripperRightIface->data->cmdPositionRate                         = this->jointData[id].cmdSpeed;
      pr2GripperRightIface->Unlock();
      */
    }
    else // corresponds to Pr2_Actarray from CASTER to ARM_R_GRIPPER
    {
      pr2Iface->Lock(1);
      pr2Iface->data->actuators[id].cmdEnableMotor                        = jointArray[id]->initialized  ;
      /*
      pr2Iface->data->actuators[id].controlMode                           = this->jointData[id].controlMode     ;
      pr2Iface->data->actuators[id].pGain                                 = this->jointData[id].pGain           ;
      pr2Iface->data->actuators[id].iGain                                 = this->jointData[id].iGain           ;
      pr2Iface->data->actuators[id].dGain                                 = this->jointData[id].dGain           ;
      pr2Iface->data->actuators[id].cmdPosition                           = this->jointData[id].cmdPosition     ;
      pr2Iface->data->actuators[id].cmdSpeed                              = this->jointData[id].cmdSpeed        ;
      */
      pr2Iface->data->actuators[id].cmdEffectorForce                      = jointArray[id]->commandedEffort;
      pr2Iface->Unlock();
    }
  }


  return PR2_ALL_OK;
}

