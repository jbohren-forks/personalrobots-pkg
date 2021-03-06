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
#include "stereo_image_proc/cam_bridge_old.h"

#include "ros/node.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

using namespace std;

void sigsegv_handler(int sig);

class DcamNode : public ros::Node
{
  bool do_rectify_;

  sensor_msgs::Image        img_;
  sensor_msgs::CameraInfo   cam_info_;

  DiagnosticUpdater<DcamNode> diagnostic_;
  diagnostic_updater::TimeStampStatus timestamp_diag_;

  int count_;
  double desired_freq_;
  string frame_id_;

public:

  static dcam::Dcam* cam_;

  DcamNode() : ros::Node("dcam"), diagnostic_(this), count_(0)
  {
    signal(SIGSEGV, &sigsegv_handler);

    dcam::init();

    int num_cams = dcam::numCameras();

    diagnostic_.add( timestamp_diag_ );
    diagnostic_.addUpdater( &DcamNode::freqStatus );

    if (num_cams > 0)
    {
      uint64_t guid;
      if (hasParam("~guid"))
      {
        string guid_str;
        getParam("~guid", guid_str);
        
        guid = strtoll(guid_str.c_str(), NULL, 16);
      } else {
        guid = dcam::getGuid(0);
      }

      string str_speed;
      dc1394speed_t speed;

      param("~speed", str_speed, string("S400"));

      param("~frame_id", frame_id_, string("stereo"));

      if (str_speed == string("S100"))
        speed = DC1394_ISO_SPEED_100;
      else if (str_speed == string("S200"))
        speed = DC1394_ISO_SPEED_200;
      else
        speed = DC1394_ISO_SPEED_400;


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
      
      string str_mode;
      dc1394video_mode_t mode = DC1394_VIDEO_MODE_640x480_MONO8;

      param("~video_mode", str_mode, string("640x480_mono8"));

      if (str_mode == string("640x480_rgb24"))
        mode = DC1394_VIDEO_MODE_640x480_RGB8;
      else if (str_mode == string("1024x768_rgb24"))
        mode = DC1394_VIDEO_MODE_1024x768_RGB8;
      else if (str_mode == string("1280x960_rgb24"))
        mode = DC1394_VIDEO_MODE_1280x960_RGB8;
      else if (str_mode == string("1600x1200_rgb24"))
        mode = DC1394_VIDEO_MODE_1600x1200_RGB8;
      else if (str_mode == string("640x480_mono8"))
        mode = DC1394_VIDEO_MODE_640x480_MONO8;
      else if (str_mode == string("1024x768_mono8"))
        mode = DC1394_VIDEO_MODE_1024x768_MONO8;
      else if (str_mode == string("1280x960_mono8"))
        mode = DC1394_VIDEO_MODE_1280x960_MONO8;
      else if (str_mode == string("1600x1200_mono8"))
        mode = DC1394_VIDEO_MODE_1600x1200_MONO8;

      param("~do_rectify", do_rectify_, false);

      cam_ = new dcam::Dcam(guid);
      cam_->setFormat(mode, fps, speed);
      cam_->start();

      while (ok() && !serviceCam())
        diagnostic_.update();
        
      advertiseCam();
    } else {
      ROS_FATAL("Found no cameras on firewire bus\n");
      shutdown();
    }
  }


  ~DcamNode()
  {
    if (cam_)
    {
      cam_->stop();
      delete cam_;
      cam_ = NULL;
    }

    dcam::fini();  
  }

  bool serviceCam()
  {
    if (!cam_->getImage(100 + (int)(1.0/desired_freq_) * 1000))
    {
      ROS_WARN("Timed out waiting for camera.");
      return false;
    }

    count_++;
    return true;
  }

  void publishCam()
  {
    publishImages("~", cam_->camIm);
  }

  void publishImage(const std::string& topic, color_coding_t type,
                    int height, int width, void* data, size_t data_size)
  {
    if (type == COLOR_CODING_NONE)
      return;
    std::string encoding = cam_bridge::ColorCodingToImageEncoding(type);
    fillImage(img_, encoding, height, width, data_size / height, data);
    img_.header.stamp = ros::Time().fromNSec(cam_->camIm->im_time * 1000);
    timestamp_diag_.tick(img_.header.stamp);
    publish(topic, img_);
  }

  void publishImages(const std::string& base_name, cam::ImageData* img_data)
  {
    publishImage(base_name + "image_raw", img_data->imRawType, img_data->imHeight,
                 img_data->imWidth, img_data->imRaw, img_data->imRawSize);
    publishImage(base_name + "image", img_data->imType, img_data->imHeight,
                 img_data->imWidth, img_data->im, img_data->imSize);
    publishImage(base_name + "image_color", img_data->imColorType, img_data->imHeight,
                 img_data->imWidth, img_data->imColor, img_data->imColorSize);
    publishImage(base_name + "image_rect", img_data->imRectType, img_data->imHeight,
                 img_data->imWidth, img_data->imRect, img_data->imRectSize);
    publishImage(base_name + "image_rect_color", img_data->imRectColorType, img_data->imHeight,
                 img_data->imWidth, img_data->imRectColor, img_data->imRectColorSize);

    cam_info_.header.stamp = ros::Time().fromNSec(cam_->camIm->im_time * 1000);
    cam_info_.height = img_data->imHeight;
    cam_info_.width  = img_data->imWidth;

    memcpy((char*)(&cam_info_.D[0]), (char*)(img_data->D),  5*sizeof(double));
    memcpy((char*)(&cam_info_.K[0]), (char*)(img_data->K),  9*sizeof(double));
    memcpy((char*)(&cam_info_.R[0]), (char*)(img_data->R),  9*sizeof(double));
    memcpy((char*)(&cam_info_.P[0]), (char*)(img_data->P), 12*sizeof(double));

    timestamp_diag_.tick(img_.header.stamp);
    publish(base_name + std::string("cam_info"), cam_info_);
  }


  void advertiseImages(std::string base_name, cam::ImageData* img_data)
  {
    advertise<sensor_msgs::CameraInfo>(base_name + std::string("cam_info"), 1);

    if (img_data->imRawType != COLOR_CODING_NONE)
      advertise<sensor_msgs::Image>(base_name + std::string("image_raw"), 1);

    if (img_data->imType != COLOR_CODING_NONE)
      advertise<sensor_msgs::Image>(base_name + std::string("image"), 1);

    if (img_data->imColorType != COLOR_CODING_NONE)
      advertise<sensor_msgs::Image>(base_name + std::string("image_color"), 1);

    if (img_data->imRectType != COLOR_CODING_NONE)
      advertise<sensor_msgs::Image>(base_name + std::string("image_rect"), 1);

    if (img_data->imRectColorType != COLOR_CODING_NONE)
      advertise<sensor_msgs::Image>(base_name + std::string("image_rect_color"), 1);

  }

  void advertiseCam()
  {
    advertiseImages("~", cam_->camIm);
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
    while (ok())
    {
      if (serviceCam())
        publishCam();
      diagnostic_.update();
    }

    return true;
  }
};

dcam::Dcam* DcamNode::cam_ = NULL;

void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  printf("System segfaulted, stopping camera nicely\n");
  if (DcamNode::cam_)
  {
    DcamNode::cam_->stop();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv);

  DcamNode dc;

  dc.spin();

  
  return 0;
}

