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

#include <cstdio>

#include "dcam/dcam.h"
#include "dcam/stereodcam.h"

#include "ros/node.h"

#include "image_msgs/RawStereo.h"
#include "cam_bridge.h"

#include "diagnostic_updater/diagnostic_updater.h"

using namespace std;

void sigsegv_handler(int sig);

class StereoDcamNode : public ros::node
{
  image_msgs::RawStereo    raw_stereo_;
  DiagnosticUpdater<StereoDcamNode> diagnostic_;

  int count_;
  double desired_freq_;

  string frame_id_;

public:

  dcam::StereoDcam* stcam_;

  StereoDcamNode() : ros::node("stereodcam"), diagnostic_(this), count_(0)
  {
    // Set up segfault handler
    signal(SIGSEGV, &sigsegv_handler);

    // Initialize the dcam system
    dcam::init();

    // Look for cameras
    int num_cams = dcam::numCameras();

    // Register a frequency status updater
    diagnostic_.addUpdater( &StereoDcamNode::freqStatus );

    // If there is a camera...
    if (num_cams > 0)
    {
      // Check our gui parameter, or else use first camera
      uint64_t guid;
      if (has_param("~guid"))
      {
        string guid_str;
        get_param("~guid", guid_str);
        
        guid = strtoll(guid_str.c_str(), NULL, 16);
      } else {
        guid = dcam::getGuid(0);
      }

      param("~frame_id", frame_id_, string("stereo"));


      // Get the ISO speed parameter if available
      string str_speed;
      dc1394speed_t speed;
      param("~speed", str_speed, string("S400"));
      if (str_speed == string("S100"))
        speed = DC1394_ISO_SPEED_100;
      else if (str_speed == string("S200"))
        speed = DC1394_ISO_SPEED_200;
      else
        speed = DC1394_ISO_SPEED_400;

      // Get the FPS parameter if available;
      double dbl_fps;
      dc1394framerate_t fps;
      param("~fps", dbl_fps, 30.0);
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
      param("~videre_mode", str_videre_mode, string("none"));
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
      set_param("~/params", params);

      // Configure camera
      stcam_->setFormat(mode, fps, speed);
      stcam_->setProcMode(videre_mode);
      stcam_->setUniqueThresh(12);
      stcam_->setTextureThresh(10);
      stcam_->setCompanding(true);

      advertise<image_msgs::RawStereo>("~raw_stereo", 1);
      
      // Start the camera
      stcam_->start();

    } else {
      ROS_FATAL("stereodcam: No cameras found\n");
      self_destruct();
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
    publish("~raw_stereo", raw_stereo_);

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

