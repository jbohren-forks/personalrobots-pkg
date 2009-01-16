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

/**

@mainpage

@htmlinclude manifest.html

Stereodcam is a driver primarily for communicating with the Videre stereocameras.

<hr>

@section behavior Behavior

The Stereodcam node outputs a "raw_stereo" message, defined in the
"image_msgs" package.  This message may either contain a left and
right image, or, in the event of STOC processing, will contain a left
image and disparity image.  It additionally contains the relevant
intrinsic and extrinsic parameters for computing stereo.

<hr>

@section names Names

The default name for the node is "stereodcam", however, this private
namespace is not actually used internally to the node.  This node
primarily makes use of the "stereo" namespace, which it shares with
the "stereoproc" node.  This namespace is both where it looks for
parameters and publishes topics.

The "stereo" name can be remapped through standard topic remapping in
the event that two cameras are sharing the same ROS system.

<hr>

@par Example

Running in the stereo namespace:
@verbatim
$ rosrun dcam stereodcam
@endverbatim

Running in a different namespace
@verbatim
$ rosrun dcam stereodcam stereo:=head_cam
@endverbatim

<hr>

@section topics Topics

Subscribes to (name/type):
- @b "stereo/check_params" : std_msgs/Empty : signal to recheck all of the parameters

Publishes to (name : type : description):
- @b "stereo/raw_stereo" : image_msgs/RawStereo : raw stereo information from camera

<hr>

@section parameters Parameters

The camera will set the following parameters after running:
- @b stereo/exposure_max (int) : maximum value for exposure
- @b stereo/exposure_min (int) : maximum value for exposure
- @b stereo/brightness_max (int) : maximum value for brightness
- @b stereo/brightness_min (int) : maximum value for brightness
- @b stereo/gain_max (int) : maximum value for gain
- @b stereo/gain_min (int) : maximum value for gain

The camera will read from the following parameters:
- @b stereo/videre_mode (string) : The processing type that a Videre
  STOC will use.  Value can be "none", "rectified", "disparity" or
  "disparity_raw"
- @b stereo/fps         (double) : Target fps of the camera
- @b stereo/speed       (string) : Firewire isospeed: S100, S200, or S400
- @b stereo/exposure (int)
- @b stereo/gain     (int)
- @b stereo/brightness (int)
- @b stereo/exposure_auto (bool)
- @b stereo/gain_auto  (bool)
- @b stereo/brightness_auto (bool)
- @b stereo/companding (bool)
- @b stereo/hdr (bool)
- @b stereo/unique_check (bool)
- @b stereo/texture_thresh (int)
- @b stereo/unique_thresh (int)
- @b stereo/smoothness_thresh (int)
- @b stereo/horopter (int)
- @b stereo/speckle_size (int)
- @b stereo/speckle_diff (int)
- @b stereo/corr_size (int)
- @b stereo/num_disp (int)

**/


#include <cstdio>

#include "dcam/dcam.h"
#include "dcam/stereodcam.h"

#include "ros/node.h"

#include "image_msgs/RawStereo.h"
#include "cam_bridge.h"

#include "diagnostic_updater/diagnostic_updater.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "std_msgs/Empty.h"

using namespace std;

void sigsegv_handler(int sig);

class StereoDcamNode : public ros::Node
{
  image_msgs::RawStereo    raw_stereo_;
  DiagnosticUpdater<StereoDcamNode> diagnostic_;

  int count_;
  double desired_freq_;

  string frame_id_;

  std::map<std::string, int> paramcache_;

  std_msgs::Empty check_params_msg_;

  std::string stereo_name_;

public:

  dcam::StereoDcam* stcam_;

  StereoDcamNode() : ros::Node("stereodcam"), diagnostic_(this), count_(0)
  {
    // Set up segfault handler
    signal(SIGSEGV, &sigsegv_handler);

    // Initialize the dcam system
    dcam::init();

    // Look for cameras
    int num_cams = dcam::numCameras();

    // Register a frequency status updater
    diagnostic_.addUpdater( &StereoDcamNode::freqStatus );

    stereo_name_ = map_name("stereo") + std::string("/");

    // If there is a camera...
    if (num_cams > 0)
    {
      // Check our gui parameter, or else use first camera
      uint64_t guid;
      if (has_param(stereo_name_+ std::string("guid")))
      {
        string guid_str;
        get_param(stereo_name_ + std::string("guid"), guid_str);
        
        guid = strtoll(guid_str.c_str(), NULL, 16);
      } else {
        guid = dcam::getGuid(0);
      }

      param(stereo_name_ + std::string("frame_id"), frame_id_, string("stereo"));


      // Get the ISO speed parameter if available
      string str_speed;
      dc1394speed_t speed;
      param(stereo_name_ + std::string("speed"), str_speed, string("S400"));
      if (str_speed == string("S100"))
        speed = DC1394_ISO_SPEED_100;
      else if (str_speed == string("S200"))
        speed = DC1394_ISO_SPEED_200;
      else
        speed = DC1394_ISO_SPEED_400;

      // Get the FPS parameter if available;
      double dbl_fps;
      dc1394framerate_t fps;
      param(stereo_name_ + std::string("fps"), dbl_fps, 30.0);
      desired_freq_ = dbl_fps;
      if (dbl_fps >= 240.0)
        fps = DC1394_FRAMERATE_240;
      else if (dbl_fps >= 120.0)
        fps = DC1394_FRAMERATE_120;
      else if (dbl_fps >= 60.0)
        fps = DC1394_FRAMERATE_60;
      else if (dbl_fps >= 30.0)
        fps = DC1394_FRAMERATE_30;
      else if (dbl_fps >= 15.0)
        fps = DC1394_FRAMERATE_15;
      else if (dbl_fps >= 7.5)
        fps = DC1394_FRAMERATE_7_5;
      else if (dbl_fps >= 3.75)
        fps = DC1394_FRAMERATE_3_75;
      else
        fps = DC1394_FRAMERATE_1_875;
      
      // For right now we ONLY support Videre stereo cameras:
      dc1394video_mode_t mode = VIDERE_STEREO_640x480;

      // Get the videre processing mode if available:
      string str_videre_mode;
      videre_proc_mode_t videre_mode = PROC_MODE_NONE;  
      param(stereo_name_ + std::string("videre_mode"), str_videre_mode, string("none"));
      if (str_videre_mode == string("rectified"))
        videre_mode = PROC_MODE_RECTIFIED;
      else if (str_videre_mode == string("disparity"))
        videre_mode = PROC_MODE_DISPARITY;
      else if (str_videre_mode == string("disparity_raw"))
        videre_mode = PROC_MODE_DISPARITY_RAW;
      else
        videre_mode = PROC_MODE_NONE;


      // Create the StereoDcam
      stcam_ = new dcam::StereoDcam(guid);

      // Fetch the camera string and send it to the parameter server if people want it (they shouldn't)
      std::string params(stcam_->getParameters());
      set_param(stereo_name_ + std::string("params"), params);

      set_param(stereo_name_ + std::string("exposure_max"), (int)stcam_->expMax);
      set_param(stereo_name_ + std::string("exposure_min"), (int)stcam_->expMin);
      set_param(stereo_name_ + std::string("gain_max"), (int)stcam_->gainMax);
      set_param(stereo_name_ + std::string("gain_min"), (int)stcam_->gainMin);
      set_param(stereo_name_ + std::string("brightness_max"), (int)stcam_->brightMax);
      set_param(stereo_name_ + std::string("brightness_min"), (int)stcam_->brightMin);

      // Configure camera
      stcam_->setFormat(mode, fps, speed);
      stcam_->setProcMode(videre_mode);
      stcam_->setUniqueThresh(36);
      stcam_->setTextureThresh(30);
      stcam_->setSpeckleSize(100);
      stcam_->setSpeckleDiff(10);
      stcam_->setCompanding(true);

      advertise<image_msgs::RawStereo>(stereo_name_ + std::string("raw_stereo"), 1);

      subscribe(stereo_name_ + std::string("check_params"), check_params_msg_, &StereoDcamNode::checkParams, 1);

      // Start the camera
      stcam_->start();

      usleep(200000);
      
      checkAndSetAll();

    } else {
      ROS_FATAL("stereodcam: No cameras found\n");
      self_destruct();
    }
  }


  void checkParams()
  {
    checkAndSetAll();
  }

  void checkAndSetAll()
  {
    checkAndSetIntBool("exposure",       boost::bind(&dcam::Dcam::setExposure,      stcam_, _1, _2));
    checkAndSetIntBool("gain",           boost::bind(&dcam::Dcam::setGain,          stcam_, _1, _2));
    checkAndSetIntBool("brightness",     boost::bind(&dcam::Dcam::setBrightness,    stcam_, _1, _2));
    checkAndSetBool("companding",        boost::bind(&dcam::Dcam::setCompanding,    stcam_, _1));
    checkAndSetBool("hdr",               boost::bind(&dcam::Dcam::setHDR,           stcam_, _1));
    checkAndSetBool("unique_check",      boost::bind(&dcam::StereoDcam::setUniqueCheck,      stcam_, _1));
    checkAndSetInt("texture_thresh",     boost::bind(&dcam::StereoDcam::setTextureThresh, stcam_, _1));
    checkAndSetInt("unique_thresh",      boost::bind(&dcam::StereoDcam::setUniqueThresh,  stcam_, _1));
    checkAndSetInt("smoothness_thresh",  boost::bind(&dcam::StereoDcam::setSmoothnessThresh,  stcam_, _1));
    checkAndSetInt("horopter",           boost::bind(&dcam::StereoDcam::setHoropter,      stcam_, _1));
    checkAndSetInt("speckle_size",       boost::bind(&dcam::StereoDcam::setSpeckleSize,      stcam_, _1));
    checkAndSetInt("speckle_diff",       boost::bind(&dcam::StereoDcam::setSpeckleDiff,      stcam_, _1));
    checkAndSetInt("corr_size",          boost::bind(&dcam::StereoDcam::setCorrsize,      stcam_, _1));
    checkAndSetInt("num_disp",           boost::bind(&dcam::StereoDcam::setNumDisp,      stcam_, _1));
  }

  void checkAndSetIntBool(std::string param_name, boost::function<void(int, bool)> setfunc)
  {
    if (has_param(stereo_name_ +  param_name) || has_param(stereo_name_ +  param_name + std::string("_auto")))
    {

      int val = 0;
      bool isauto = false;

      param( stereo_name_ +  param_name, val, 0);
      param( stereo_name_ +  param_name + std::string("_auto"), isauto, false);
    
      int testval = (val * (!isauto));

      std::map<std::string, int>::iterator cacheval = paramcache_.find(param_name);

      if ( (cacheval == paramcache_.end())
           || (cacheval->second != testval) )
        setfunc(val, isauto);

      paramcache_[param_name] = testval;
    }
  }

  void checkAndSetBool(std::string param_name, boost::function<bool(bool)> setfunc)
  {
    if (has_param(stereo_name_ +  param_name))
    {
      bool on = false;

      param(stereo_name_ +  param_name, on, false);
    

      std::map<std::string, int>::iterator cacheval = paramcache_.find(param_name);

      if ( (cacheval == paramcache_.end())
           || (cacheval->second != on) )
        setfunc(on);

      paramcache_[param_name] = on;
    }
  }

  void checkAndSetInt(std::string param_name, boost::function<bool(int)> setfunc)
  {
    if (has_param(stereo_name_ +  param_name))
    {

      int val = 0;

      param(stereo_name_ +  param_name, val, 0);

      std::map<std::string, int>::iterator cacheval = paramcache_.find(param_name);

      if ( (cacheval == paramcache_.end())
           || (cacheval->second != val) )
        setfunc(val);

      paramcache_[param_name] = val;
    }
  }

  void cleanup()
  {
    if (stcam_)
    {
      stcam_->stop();
      delete stcam_;
    }

    dcam::fini();  
  }

  ~StereoDcamNode()
  {
    cleanup();
  }

  bool serviceCam()
  {
    if (!stcam_->getImage(100 + (int)(1.0/desired_freq_ * 1000)))
    {
      ROS_WARN("Timed out waiting for camera.");
      return false;
    }
    
    cam_bridge::StereoDataToRawStereo(stcam_->stIm, raw_stereo_);
    raw_stereo_.header.frame_id = frame_id_;
    publish(stereo_name_ + std::string("raw_stereo"), raw_stereo_);

    count_++;
    return true;
  }

  void freqStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Frequency Status";

    double freq = (double)(count_)/diagnostic_.getPeriod();

    if (freq < (.9*desired_freq_))
    {
      status.level = 2;
      status.message = "Desired frequency not met";
    }
    else
    {
      status.level = 0;
      status.message = "Desired frequency met";
    }

    status.set_values_size(3);
    status.values[0].label = "Images in interval";
    status.values[0].value = count_;
    status.values[1].label = "Desired frequency";
    status.values[1].value = desired_freq_;
    status.values[2].label = "Actual frequency";
    status.values[2].value = freq;

    printf("%g fps\n", freq);

    count_ = 0;
  }

  bool spin()
  {
    // Start up the camera
    while (ok())
    {
      serviceCam();
      diagnostic_.update();
    }

    return true;
  }
};

StereoDcamNode* g_sdc = 0;

void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  printf("System segfaulted, stopping camera nicely\n");
  if (g_sdc)
  {
    g_sdc->cleanup();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv);

  g_sdc = new StereoDcamNode;

  g_sdc->spin();

  delete g_sdc;

  ros::fini();
  return 0;
}

