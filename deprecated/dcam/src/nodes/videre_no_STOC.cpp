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

Videre_no_STOC is a driver for communicating with the Videre stereocameras.
It is intended to be identical to communicating with two other cameras.

<hr>

@section topics Topics

Subscribes to (name/type):
- @b "videre/check_params" : std_msgs/Empty : signal to recheck all of the parameters

Publishes to (name : type : description):
- @b "left/image" : sensor_msgs/Image : Left unprocessed image
- @b "left/camera_info" : sensor_msgs/CameraInfo : Left camera information message
- @b "right/image" : sensor_msgs/Image : Right unprocessed image
- @b "right/camera_info" : sensor_msgs/CameraInfo : Right camera information message

<hr>

@section parameters Parameters

The camera will set the following parameters after running:
- @b videre/exposure_max (int) : maximum value for exposure
- @b videre/exposure_min (int) : maximum value for exposure
- @b videre/brightness_max (int) : maximum value for brightness
- @b videre/brightness_min (int) : maximum value for brightness
- @b videre/gain_max (int) : maximum value for gain
- @b videre/gain_min (int) : maximum value for gain

**/


#include <cstdio>

#include "dcam/dcam.h"
#include "dcam/stereodcam.h"
#include "cam_bridge.h"

#include "ros/node.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "std_msgs/Empty.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "sensor_msgs/fill_image.h"

using namespace std;

void sigsegv_handler(int sig);

class VidereNode
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle camera_nh_;
  ros::NodeHandle left_nh_;
  ros::NodeHandle right_nh_;

  sensor_msgs::Image left_img_;
  sensor_msgs::Image right_img_;
  sensor_msgs::CameraInfo left_info_;
  sensor_msgs::CameraInfo right_info_;

  ros::Publisher left_img_pub_;
  ros::Publisher right_img_pub_;
  ros::Publisher left_info_pub_;
  ros::Publisher right_info_pub_;

  ros::Subscriber check_param_sub_;

  DiagnosticUpdater<VidereNode> diagnostic_;
  diagnostic_updater::TimeStampStatus timestamp_diag_;

  int count_;
  double desired_freq_;

  string frame_id_;

  std::map<std::string, int> paramcache_;

public:

  // Clearly this will need to go away to sort out our dependencies,
  // but for right now it's the only library we have to talk to the
  // Videre.
  dcam::StereoDcam* stcam_;

  VidereNode() : private_nh_("~"), camera_nh_("camera"), left_nh_(camera_nh_, "left"), right_nh_(camera_nh_, "right"), diagnostic_(this), count_(0)
  {
    // Set up segfault handler
    signal(SIGSEGV, &sigsegv_handler);

    // Initialize the dcam system
    dcam::init();

    // Look for cameras
    int num_cams = dcam::numCameras();

    // Register a frequency status updater
    diagnostic_.add( timestamp_diag_ );
    diagnostic_.addUpdater( &VidereNode::freqStatus );

    // If there is a camera...
    if (num_cams > 0)
    {
      // Check our gui parameter, or else use first camera
      uint64_t guid;
      if (private_nh_.hasParam("guid"))
      {
        string guid_str;
        private_nh_.getParam("guid", guid_str);
        
        guid = strtoll(guid_str.c_str(), NULL, 16);
      } else {
        guid = dcam::getGuid(0);
      }

      private_nh_.param("frame_id", frame_id_, string("videre"));


      // Get the ISO speed parameter if available
      string str_speed;
      dc1394speed_t speed;
      private_nh_.param("speed", str_speed, string("S400"));
      if (str_speed == string("S100"))
        speed = DC1394_ISO_SPEED_100;
      else if (str_speed == string("S200"))
        speed = DC1394_ISO_SPEED_200;
      else
        speed = DC1394_ISO_SPEED_400;

      // Get the FPS parameter if available;
      double dbl_fps;
      dc1394framerate_t fps;
      private_nh_.param("fps", dbl_fps, 30.0);
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

      // Create the StereoDcam
      stcam_ = new dcam::StereoDcam(guid);

      // Fetch the camera string and send it to the parameter server if people want it (they shouldn't)
      std::string params(stcam_->getParameters());
      private_nh_.setParam("params", params);

      private_nh_.setParam("exposure_max", (int)stcam_->expMax);
      private_nh_.setParam("exposure_min", (int)stcam_->expMin);
      private_nh_.setParam("gain_max", (int)stcam_->gainMax);
      private_nh_.setParam("gain_min", (int)stcam_->gainMin);
      private_nh_.setParam("brightness_max", (int)stcam_->brightMax);
      private_nh_.setParam("brightness_min", (int)stcam_->brightMin);

      // Get the videre processing mode if available:
      videre_proc_mode_t videre_mode = PROC_MODE_NONE;  

      // Configure camera
      stcam_->setFormat(mode, fps, speed);
      stcam_->setProcMode(videre_mode);
      stcam_->setCompanding(true);

      left_img_pub_ = left_nh_.advertise<sensor_msgs::Image>("image", 1);
      right_img_pub_ = right_nh_.advertise<sensor_msgs::Image>("image", 1);
      left_info_pub_ = left_nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
      right_info_pub_ = right_nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

      check_param_sub_ = private_nh_.subscribe("check_params", 1, &VidereNode::checkParams, this);

      // Start the camera
      stcam_->start();

      usleep(200000);
      
      checkAndSetAll();

    } else {
      ROS_FATAL("videre_no_STOC: No cameras found\n");
      nh_.shutdown();
    }
  }

  void checkParams(const std_msgs::Empty::ConstPtr& msg)
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
  }

  void checkAndSetIntBool(std::string param_name, boost::function<void(int, bool)> setfunc)
  {
    if (private_nh_.hasParam(param_name) || private_nh_.hasParam(param_name + std::string("_auto")))
    {

      int val = 0;
      bool isauto = false;

      private_nh_.param( param_name, val, 0);
      private_nh_.param( param_name + std::string("_auto"), isauto, false);
    
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
    if (private_nh_.hasParam(param_name))
    {
      bool on = false;

      private_nh_.param(param_name, on, false);
    

      std::map<std::string, int>::iterator cacheval = paramcache_.find(param_name);

      if ( (cacheval == paramcache_.end())
           || (cacheval->second != on) )
        setfunc(on);

      paramcache_[param_name] = on;
    }
  }

  void checkAndSetInt(std::string param_name, boost::function<bool(int)> setfunc)
  {
    if (private_nh_.hasParam(param_name))
    {

      int val = 0;

      private_nh_.param( param_name, val, 0);

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

  ~VidereNode()
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
    
    uint8_t scratch_type;

    cam_bridge::CamDataToRawStereo(stcam_->stIm->imLeft, left_img_, left_info_, scratch_type);
    cam_bridge::CamDataToRawStereo(stcam_->stIm->imRight, right_img_, right_info_, scratch_type);

    left_img_.header.frame_id = frame_id_;
    left_info_.header.frame_id = frame_id_;
    right_img_.header.frame_id = frame_id_;
    right_info_.header.frame_id = frame_id_;

    left_img_.header.stamp = ros::Time::now(); // Timestamp from the low level driver is bogus. 
                                                 // Doing this as a stopgap
                                                 // measure.
    left_info_.header.stamp = left_img_.header.stamp;
    right_img_.header.stamp = left_img_.header.stamp;
    right_info_.header.stamp = left_img_.header.stamp;

    timestamp_diag_.tick(left_img_.header.stamp);

    left_img_pub_.publish(left_img_);
    left_info_pub_.publish(left_info_);
    right_img_pub_.publish(right_img_);
    right_info_pub_.publish(right_info_);

    count_++;
    return true;
  }

  void freqStatus(diagnostic_msgs::DiagnosticStatus& status)
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
    status.values[0].key = "Images in interval";
    status.values[0].value = count_;
    status.values[1].key = "Desired frequency";
    status.values[1].value = desired_freq_;
    status.values[2].key = "Actual frequency";
    status.values[2].value = freq;

    printf("%g fps\n", freq);

    count_ = 0;
  }

  bool spin()
  {
    // Start up the camera
    while (nh_.ok())
    {
      serviceCam();
      diagnostic_.update();
    }

    return true;
  }
};

VidereNode* g_sdc = 0;

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
  ros::init(argc, argv, "videre");

  g_sdc = new VidereNode;

  g_sdc->spin();

  delete g_sdc;

  return 0;
}

