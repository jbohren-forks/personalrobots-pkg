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

#include "ros/node.h"
#include "image_msgs/ImageWrapper.h"
#include "dcam.h"
#include "stereocam.h"

using namespace std;

class DcamNode : public ros::node
{

  dcam::Dcam* cam_;
  bool stereo_cam;
  bool do_stereo;
  bool do_rectify;

  image_msgs::ImageWrapper img;

public:

  DcamNode() : ros::node("dcam")
  {
    dcam::init();

    int num_cams = dcam::numCameras();

    if (num_cams > 0)
    {
      uint64_t guid;
      if (has_param("~guid"))
      {
        string guid_str;
        get_param("~guid", guid_str);
        
        guid = strtoll(guid_str.c_str(), NULL, 16);
      } else {
        guid = dcam::getGuid(0);
      }

      string str_speed;
      dc1394speed_t speed;

      param("~speed", str_speed, string("S400"));

      if (str_speed == string("S100"))
        speed = DC1394_ISO_SPEED_100;
      else if (str_speed == string("S200"))
        speed = DC1394_ISO_SPEED_200;
      else
        speed = DC1394_ISO_SPEED_400;


      double dbl_fps;
      dc1394framerate_t fps;

      param("~fps", dbl_fps, 30.0);

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
      dc1394video_mode_t mode;
      videre_proc_mode_t videre_mode;  
      stereo_cam = false;

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
      else if (str_mode == string("640x480_videre_none"))
      {
        stereo_cam = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_NONE;
      }
      else if (str_mode == string("640x480_videre_rectified"))
      {
        stereo_cam = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_RECTIFIED;
      }
      else if (str_mode == string("640x480_videre_disparity"))
      {
        stereo_cam = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_DISPARITY;
      }
      else if (str_mode == string("640x480_videre_disparity_raw"))
      {
        stereo_cam = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_DISPARITY_RAW;
      }
      else
        mode = DC1394_VIDEO_MODE_640x480_MONO8;

      param("~do_rectify", do_rectify, false);

      param("~do_stereo", do_stereo, false);

      // Only do stereo 
      do_stereo = do_stereo && stereo_cam;

      // Must rectify if doing stereo
      do_rectify = do_rectify || do_stereo;

      // Right now can only rectify if a StereoDcam -- this is wrong
      do_rectify = do_rectify && stereo_cam;

      // This switch might not be necessary... can maybe read
      // from regular cam with StereoDcam, but then the name
      // is definitely wrong.
      if (stereo_cam)
      {
        cam_ = new cam::StereoDcam(guid);
        cam_->setFormat(mode, fps, speed);
        cam_->setProcMode(videre_mode);
      } else {
        cam_ = new dcam::Dcam(guid);
        cam_->setFormat(mode, fps, speed);
      }

      cam_->start();

      serviceCam();

      advertiseCam();
    }
  }


  ~DcamNode()
  {
    if (cam_)
    {
      cam_->stop();
      delete cam_;
    }

    dcam::fini();  
  }

  void serviceCam()
  {
    printf("Servicing camera\n");

    cam_->getImage(500);

    if (do_rectify)
      ( (cam::StereoDcam*)(cam_) )->doRectify();

    if (do_stereo)
      ( (cam::StereoDcam*)(cam_) )->doDisparity();
  }

  void publishCam()
  {
    if (stereo_cam)
    {

      StereoCam* stcam = ( (StereoDcam*)(cam_) );

      if (stcam->stIm->imLeft->im)
      {
        img.fromData( "image",
                      stcam->stIm->imLeft->imHeight, stcam->stIm->imLeft->imWidth, 1,
                      "mono", "byte",
                      stcam->stIm->imLeft->im );

        //        img.fromDcamImageData(cam_->camIm);
        publish("~left_image", img);
      }

    } else {
      if (cam_->camIm->im)
      {
        img.fromData( "image",
                      cam_->camIm->imHeight, cam_->camIm->imWidth, 1,
                      "mono", "byte",
                      cam_->camIm->im );

        //        img.fromDcamImageData(cam_->camIm);
        publish("~image", img);
      }
    }
  }

  void advertiseCam()
  {
    if (stereo_cam)
    {

      StereoCam* stcam = ( (StereoDcam*)(cam_) );

      if (stcam->stIm->imLeft->im)
        advertise<image_msgs::ImageWrapper>("~left_image",1);

      if (stcam->stIm->imRight->im)
        advertise<image_msgs::ImageWrapper>("~right_image",1);

      if (stcam->stIm->imDisp)
        advertise<image_msgs::ImageWrapper>("~disparity",1);

      // Add checks for other image types...

    } else {

      printf("Checking if camera has Im\n");
      if (cam_->camIm->im)
      {
        printf("It does... advertising\n");
        advertise<image_msgs::ImageWrapper>("~image", 1);
      }

      // Add checks for other image types...

    }
  }

  bool spin()
  {
    // Start up the laser
    while (ok())
    {
      serviceCam();
      publishCam();
    }

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  //Keep things from dying poorly
  DcamNode dc;

  dc.spin();


  ros::fini();
  return 0;
}

