

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef GAZEBO_SENSORS_API_HH
#define GAZEBO_SENSORS_API_HH

#include <pr2Core/pr2Core.h>
#include <pr2Core/pr2Misc.h>
#include <math.h>
#include <list>
#include <vector>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

#include <sys/types.h>
#include <stdint.h>
#include <string>
#include <list>
#include <vector>

namespace PR2
{
  /*! \class 
    \brief A low-level function call based API for PR2
  */
  class GazeboSensors
  {


  private:
    ////////////////////////////////////////////////////////////////////
    //                                                                //
    //  Gazebo Client Interfaces                                      //
    //                                                                //
    //  these are the "hardware" interfaces                           //
    //                                                                //
    ////////////////////////////////////////////////////////////////////
    gazebo::Client           *client;
    gazebo::SimulationIface  *simIface;

    gazebo::LaserIface       *pr2LaserIface;
    gazebo::LaserIface       *pr2BaseLaserIface;

    gazebo::CameraIface      *pr2CameraIface;
    gazebo::CameraIface      *pr2CameraGlobalIface;
    gazebo::CameraIface      *pr2CameraHeadLeftIface;
    gazebo::CameraIface      *pr2CameraHeadRightIface;

    gazebo::PositionIface    *pr2LeftWristIface;
    gazebo::PositionIface    *pr2RightWristIface;
    gazebo::PositionIface    *pr2BaseIface;


    /*! \fn 
      \brief Constructor
    */
    public: GazeboSensors();

    /*! \fn 
      \brief Destructor
    */
    public: virtual ~GazeboSensors();
       
    /*! \fn
      \brief This is where hardware/gazebo interfaces should be initialized 
    */
    public: PR2_ERROR_CODE Init();

    /*! \fn
      \brief - returns simulation time
    */
    public:    PR2_ERROR_CODE GetSimTime(double *sim_time);

    /*! \fn
      \brief Enable the model (i.e. enable all actuators corresponding to a particular part of the robot)
      \param id - modelID, see pr2Core.h for a list of model IDs
    */
    public: PR2_ERROR_CODE EnableModel(PR2_MODEL_ID id);

    /*! \fn
      \brief Disable the model (i.e. disable all actuators corresponding to a particular part of the robot)
      \param id - model ID, see pr2Core.h for a list of model IDs
    */
    public: PR2_ERROR_CODE DisableModel(PR2_MODEL_ID id);

    /*! \fn
      \brief Check whether all actuators in a particular part of the robot have been enabled
      \param id - model ID, see pr2Core.h for a list of model IDs
      \param enabled - pointer to return value - 0 if any actuator in that part of the robot is disabled, 1 if all actuators in that part of the robot are enabled 
    */
    public: PR2_ERROR_CODE IsEnabledModel(PR2_MODEL_ID id, int *enabled);    

    /*! \fn
      \brief - Get laser range data
    */
    public:    PR2_ERROR_CODE GetLaserRanges(PR2_SENSOR_ID id,
        float* angle_min, float* angle_max, float* angle_increment,
        float* range_max,uint32_t* ranges_size     ,uint32_t* ranges_alloc_size,
                         uint32_t* intensities_size,uint32_t* intensities_alloc_size,
                         float*    ranges          ,uint8_t*  intensities);

    /*! \fn
      \brief - Get camera data
    */
    public:    PR2_ERROR_CODE GetCameraImage(PR2_SENSOR_ID id ,
                   uint32_t*    width                 ,uint32_t*    height                ,
                   uint32_t*    depth                 ,
                   std::string* compression           ,std::string* colorspace            ,
                   uint32_t*    data_size             ,void*        data                  );

    /*! \fn
      \brief Retrieve caster/wheel properties and estimate velocity of the base in cartesian space in body coordinates 
      \param vx - forward speed
      \param vy - sideways speed
      \param vw - rotational speed
    */
    public: PR2_ERROR_CODE GetBaseCartesianSpeedGroundTruth(double* vx, double* vy, double* vw);

    /*! \fn
      \brief Retrieve base box position
      \param x - forward
      \param y - sideways (left)
      \param w - upward
    */
    public: PR2_ERROR_CODE GetBasePositionGroundTruth(double* x, double* y, double* z);


    /*! \fn
      \brief - Get ground truth base position
    */
    public: PR2_ERROR_CODE GetBasePositionGroundTruth(double* x, double* y, double *z, double *roll, double *pitch, double *yaw);

    /*! \fn
      \brief Get the actual wrist pose 
      \param id - model ID, see pr2Core.h for a list of model IDs
      \param *x pointer to return value of x position of the wrist 
      \param *y pointer to return value of y position of the wrist 
      \param *z pointer to return value of z position of the wrist
      \param *roll pointer to return value of roll of the wrist 
      \param *pitch pointer to return value of pitch of the wrist 
      \param *yaw pointer to return value of yaw of the wrist
    */
    public: PR2_ERROR_CODE GetWristPoseGroundTruth(PR2_MODEL_ID id, double *x, double *y, double *z, double *roll, double *pitch, double *yaw);


    /*! \fn
      \brief Wait for Gazebo to update
      FIXME:  wait time too long
    */
    public: PR2_ERROR_CODE ClientWait();

  };

}
#endif
