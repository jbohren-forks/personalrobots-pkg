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
#include "image_msgs/Image.h"
#include "image_msgs/FillImage.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/StereoInfo.h"
#include "dcam.h"
#include "stereocam.h"

using namespace std;

class DcamNode : public ros::node
{

  dcam::Dcam* cam_;
  bool stereo_cam_;
  bool do_stereo_;
  bool do_rectify_;

  image_msgs::Image        img_;
  image_msgs::CamInfo      cam_info_;
  image_msgs::StereoInfo   stereo_info_;

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
      stereo_cam_ = false;

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
        stereo_cam_ = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_NONE;
      }
      else if (str_mode == string("640x480_videre_rectified"))
      {
        stereo_cam_ = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_RECTIFIED;
      }
      else if (str_mode == string("640x480_videre_disparity"))
      {
        stereo_cam_ = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_DISPARITY;
      }
      else if (str_mode == string("640x480_videre_disparity_raw"))
      {
        stereo_cam_ = true;
        mode = VIDERE_STEREO_640x480;
        videre_mode = PROC_MODE_DISPARITY_RAW;
      }
      else
        mode = DC1394_VIDEO_MODE_640x480_MONO8;

      param("~do_rectify", do_rectify_, false);

      param("~do_stereo", do_stereo_, false);

      // Only do stereo 
      do_stereo_ = do_stereo_ && stereo_cam_;

      // Must rectify if doing stereo
      do_rectify_ = do_rectify_ || do_stereo_;

      // Right now can only rectify if a StereoDcam -- this is wrong
      do_rectify_ = do_rectify_ && stereo_cam_;

      // This switch might not be necessary... can maybe read
      // from regular cam with StereoDcam, but then the name
      // is definitely wrong.
      if (stereo_cam_)
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

    // THIS should be possible for all cameras, not just stereo
    if (do_rectify_)
    {
      ( (cam::StereoDcam*)(cam_) )->doRectify();
    }

    if (do_stereo_)
      ( (cam::StereoDcam*)(cam_) )->doDisparity();
  }

  void publishCam()
  {
    if (stereo_cam_)
    {
      StereoCam* stcam = ( (StereoDcam*)(cam_) );

      publishImages("~left/", stcam->stIm->imLeft);
      publishImages("~right/", stcam->stIm->imRight);
      
      if (stcam->stIm->imDisp)
      {
        fillImage(img_,  "disparity",
                  stcam->stIm->imHeight, stcam->stIm->imWidth, 1,
                  "mono", "uint16",
                  stcam->stIm->imDisp );
        publish("~disparity", img_);
        
        stereo_info_.has_disparity = true;
      } else {
        stereo_info_.has_disparity = false;
      }

      stereo_info_.height = stcam->stIm->imHeight;
      stereo_info_.width = stcam->stIm->imWidth;

      stereo_info_.dpp = stcam->stIm->dpp;
      stereo_info_.numDisp = stcam->stIm->numDisp;
      stereo_info_.imDtop = stcam->stIm->imDtop;
      stereo_info_.imDleft = stcam->stIm->imDleft;
      stereo_info_.imDwidth = stcam->stIm->imDwidth;
      stereo_info_.imDheight = stcam->stIm->imDheight;
      stereo_info_.corrSize = stcam->stIm->corrSize;
      stereo_info_.filterSize = stcam->stIm->filterSize;
      stereo_info_.horOffset = stcam->stIm->horOffset;
      stereo_info_.textureThresh = stcam->stIm->textureThresh;
      stereo_info_.uniqueThresh = stcam->stIm->uniqueThresh;

      memcpy((char*)(&stereo_info_.T[0]),  (char*)(stcam->stIm->T),   3*sizeof(double));
      memcpy((char*)(&stereo_info_.Om[0]), (char*)(stcam->stIm->Om),  3*sizeof(double));
      memcpy((char*)(&stereo_info_.RP[0]), (char*)(stcam->stIm->RP), 12*sizeof(double));

      publish("~stereo_info", stereo_info_);

    } else {
      publishImages("~", cam_->camIm);
    }
  }

  void publishImages(std::string base_name, cam::ImageData* img)
  {

    if (img->imRawType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image_raw",
                img->imHeight, img->imWidth, 1,
                "mono", "byte",
                img->imRaw );
      publish(base_name + std::string("image_raw"), img_);
      cam_info_.has_image = true;
    } else {
      cam_info_.has_image = false;
    }

    if (img->imType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image",
                img->imHeight, img->imWidth, 1,
                "mono", "byte",
                img->im );
      publish(base_name + std::string("image"), img_);
      cam_info_.has_image = true;
    } else {
      cam_info_.has_image = false;
    }

    if (img->imColorType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image_color",
                img->imHeight, img->imWidth, 4,
                "rgba", "byte",
                img->imColor );
      publish(base_name + std::string("image_color"), img_);
      cam_info_.has_image_color = true;
    } else {
      cam_info_.has_image_color = false;
    }

    if (img->imRectType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image_rect",
                img->imHeight, img->imWidth, 1,
                "mono", "byte",
                img->imRect );
      publish(base_name + std::string("image_rect"), img_);
      cam_info_.has_image_rect = true;
    } else {
      cam_info_.has_image_rect = false;
    }

    if (img->imRectColorType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image_rect_color",
                img->imHeight, img->imWidth, 4,
                "rgba", "byte",
                img->imRectColor );
      publish(base_name + std::string("image_rect_color"), img_);
      cam_info_.has_image_rect_color = true;
    }else {
      cam_info_.has_image_rect_color = false;
    }

    cam_info_.height = img->imHeight;
    cam_info_.width  = img->imWidth;

    memcpy((char*)(&cam_info_.D[0]), (char*)(img->D),  5*sizeof(double));
    memcpy((char*)(&cam_info_.K[0]), (char*)(img->K),  9*sizeof(double));
    memcpy((char*)(&cam_info_.R[0]), (char*)(img->R),  9*sizeof(double));
    memcpy((char*)(&cam_info_.P[0]), (char*)(img->P), 12*sizeof(double));

    publish(base_name + std::string("cam_info"), cam_info_);

  }


  void advertiseImages(std::string base_name, cam::ImageData* img)
  {
    advertise<image_msgs::CamInfo>(base_name + std::string("cam_info"), 1);

    if (img->imRawType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_raw"), 1);

    if (img->imType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image"), 1);

    if (img->imColorType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_color"), 1);

    if (img->imRectType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_rect"), 1);

    if (img->imRectColorType != COLOR_CODING_NONE)
      advertise<image_msgs::Image>(base_name + std::string("image_rect_color"), 1);

  }

  void advertiseCam()
  {
    if (stereo_cam_)
    {
      StereoCam* stcam = ( (StereoDcam*)(cam_) );

      advertise<image_msgs::StereoInfo>("~stereo_info", 1);

      advertiseImages("~left/", stcam->stIm->imLeft);
      advertiseImages("~right/", stcam->stIm->imRight);

      if (stcam->stIm->imDisp)
        advertise<image_msgs::Image>("~disparity", 1);

    }
    else
    {
      advertiseImages("~", cam_->camIm);
    }
  }

  bool spin()
  {
    // Start up the camera
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

  DcamNode dc;

  dc.spin();

  ros::fini();
  return 0;
}

