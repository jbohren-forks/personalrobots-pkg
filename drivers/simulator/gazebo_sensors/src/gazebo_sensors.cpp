
#include <gazebo_sensors/gazebo_sensors.h>

using namespace gazebo;

////////////////////////////////////////////////////////////////////
//                                                                //
//  TODOS:                                                        //
//                                                                //
//    IMPLEMENT TIME STAMP IN THE MESSAGES RETURNED BY SENSORS    //
//    CHECK TO SEE IF MULTIPLE INSTANCES OF CLIENT IS OK          //
//                                                                //
////////////////////////////////////////////////////////////////////


GazeboSensors::GazeboSensors()
{
  ////////////////////////////////////////////////////////////////////
  //                                                                //
  //  GazeboSensors Class                                           //
  //                                                                //
  ////////////////////////////////////////////////////////////////////
  client                  = new gazebo::Client();
  simIface                = new gazebo::SimulationIface();

  pr2LaserIface           = new gazebo::LaserIface();
  pr2BaseLaserIface       = new gazebo::LaserIface();

  pr2CameraGlobalIface    = new gazebo::CameraIface();
  pr2CameraHeadLeftIface  = new gazebo::CameraIface();
  pr2CameraHeadRightIface = new gazebo::CameraIface();

  pr2LeftWristIface       = new gazebo::PositionIface();
  pr2RightWristIface      = new gazebo::PositionIface();
  pr2BaseIface            = new gazebo::PositionIface();
}

GazeboSensors::~GazeboSensors()
{

}

PR2::PR2_ERROR_CODE GazeboSensors::Init()
{
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

  // ground truths, position interface (can be used for fake IMU)
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


  return PR2::PR2_ALL_OK;
};


PR2::PR2_ERROR_CODE GazeboSensors::GetSimTime(double *sim_time)
{
   *sim_time = simIface->data->simTime;
   return PR2::PR2_ALL_OK;
};

PR2::PR2_ERROR_CODE GazeboSensors::GetLaserRanges(PR2::PR2_SENSOR_ID id,
    float* angle_min, float* angle_max, float* angle_increment,
    float* range_max,uint32_t* ranges_size     ,uint32_t* ranges_alloc_size,
                     uint32_t* intensities_size,uint32_t* intensities_alloc_size,
                     float*    ranges          ,uint8_t*  intensities)
{

  gazebo::LaserIface       *tmpLaserIface;
  switch (id)
  {
    case PR2::LASER_HEAD:
      tmpLaserIface = pr2LaserIface;
      break;
    case PR2::LASER_BASE:
      tmpLaserIface = pr2BaseLaserIface;
      break;
    default:
      tmpLaserIface = NULL;
      break;
  }
  if (tmpLaserIface == NULL)
  {
    return PR2::PR2_ERROR;
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
    return PR2::PR2_ALL_OK;
  }
};


PR2::PR2_ERROR_CODE GazeboSensors::GetCameraImage(PR2::PR2_SENSOR_ID id ,
                     uint32_t*    width                 ,uint32_t*    height                ,
                     uint32_t*    depth                 ,
                     std::string* compression           ,std::string* colorspace            ,
                     uint32_t*    data_size             ,void*        buf                   )
{

    switch(id)
    {
      case PR2::CAMERA_GLOBAL:
          pr2CameraIface = pr2CameraGlobalIface;
          break;
      case PR2::CAMERA_HEAD_LEFT:
          pr2CameraIface = pr2CameraHeadLeftIface;
          break;
      case PR2::CAMERA_HEAD_RIGHT:
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

    return PR2::PR2_ALL_OK;
};



/**********************************************************************************/
/*                                                                                */
/*                                                                                */
/*   position interfaces, returns positions                                       */
/*                                                                                */
/*                                                                                */
/**********************************************************************************/
PR2::PR2_ERROR_CODE GazeboSensors::GetWristPoseGroundTruth(PR2::PR2_MODEL_ID id, double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
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
         return PR2::PR2_ERROR;
   }
   return PR2::PR2_ALL_OK;
};


PR2::PR2_ERROR_CODE GazeboSensors::GetBasePositionGroundTruth(double* x, double* y, double *z, double *roll, double *pitch, double *yaw)
{
   pr2BaseIface->Lock(1);
   *x     = pr2BaseIface->data->pose.pos.x;
   *y     = pr2BaseIface->data->pose.pos.y;
   *z     = pr2BaseIface->data->pose.pos.z;
   *roll  = pr2BaseIface->data->pose.roll;
   *pitch = pr2BaseIface->data->pose.pitch;
   *yaw   = pr2BaseIface->data->pose.yaw;
   pr2BaseIface->Unlock();
   return PR2::PR2_ALL_OK;
};

PR2::PR2_ERROR_CODE GazeboSensors::ClientWait()
{
  // block until simulator update.
  client->Wait();
  return PR2::PR2_ALL_OK;
}




