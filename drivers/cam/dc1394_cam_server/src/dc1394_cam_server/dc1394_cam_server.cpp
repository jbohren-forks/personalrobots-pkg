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
#include "std_msgs/Image.h"
#include "std_msgs/ImageArray.h"
#include "std_msgs/PointCloudFloat32.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

#include "dc1394_cam/dc1394_cam.h"
#include "videre_cam/videre_cam.h"

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>

#include <vector>
#include <list>
#include <sstream>

enum CamTypes {NORMAL, VIDERE};

class CamData
{
public:
  CamData() : name(""), cam(NULL), cam_type(NORMAL) {}

  void cleanup()
  {
    if (cam)
      delete cam;
  }

  string name;
  dc1394_cam::Cam* cam;
  CamTypes  cam_type;
};

class Dc1394CamServer : public ros::node
{
public:

  std_msgs::ImageArray img_;
  std_msgs::PointCloudFloat32 cloud_;

  ros::Time next_time_;

  list<CamData> cams_;
  
  int count_;

  void checkAndSetFeature(CamData& cd, string param_name, dc1394feature_t feature)
  {
    string p = cd.name + string("/") + param_name;
    if (has_param(p))
    {
      XmlRpc::XmlRpcValue val;
      get_param(p, val);
      
      if (val.getType() == XmlRpc::XmlRpcValue::TypeString)
        if (val == string("auto"))
          cd.cam->setFeatureMode(feature, DC1394_FEATURE_MODE_AUTO);
      
      if (val.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        cd.cam->setFeatureMode(feature, DC1394_FEATURE_MODE_MANUAL);
        cd.cam->setFeature(feature, (int)(val));
      }

      if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {
        cd.cam->setFeatureMode(feature, DC1394_FEATURE_MODE_MANUAL);
        cd.cam->setFeatureAbsolute(feature, (double)(val));
      }
    }
  }


  Dc1394CamServer() : ros::node("dc1394_node")
  {
    dc1394_cam::init();

    int num_cams;
    param("num_cams", num_cams, 1);

    for (int i = 0; i < num_cams; i++)
    {
      CamData cd;

      ostringstream oss;
      oss << "cam" << i;

      param(oss.str(), cd.name, oss.str());

      uint64_t guid;
      if (has_param(cd.name + string("/guid")))
      {
        string guid_str;
        get_param(cd.name + string("/guid"), guid_str);
        
        guid = strtoll(guid_str.c_str(), NULL, 16);
      } else {
        guid = dc1394_cam::getGuid(i);
      }

      string str_speed;
      dc1394speed_t speed;
      param(cd.name + string("/speed"), str_speed, string("S400"));
      if (str_speed == string("S100"))
        speed = DC1394_ISO_SPEED_100;
      else if (str_speed == string("S200"))
        speed = DC1394_ISO_SPEED_200;
      else
        speed = DC1394_ISO_SPEED_400;

      double dbl_fps;
      dc1394framerate_t fps;
      param(cd.name + string("/fps"), dbl_fps, 30.0);
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

      cd.cam_type = NORMAL;

      string str_mode;
      dc1394video_mode_t mode;
      param(cd.name + string("/video_mode"), str_mode, string("640x480_mono8"));
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
      else if (str_mode == string("640x480_videre"))
      {
        mode = DC1394_VIDEO_MODE_640x480_MONO8;
        cd.cam_type = VIDERE;
      }
      else
        mode = DC1394_VIDEO_MODE_640x480_MONO8;

      int buffer_size;
      param(cd.name + string("/buffer_size"), buffer_size, 8);

      dc1394color_filter_t bayer;
      bool colorize = false;

      if ( mode == DC1394_VIDEO_MODE_640x480_MONO8 ||
           mode == DC1394_VIDEO_MODE_1024x768_MONO8 ||
           mode == DC1394_VIDEO_MODE_1280x960_MONO8 ||
           mode == DC1394_VIDEO_MODE_1600x1200_MONO8)
      {
        colorize = true;
        string str_bayer;
        param(cd.name + string("/bayer"), str_bayer, string("none"));

        if (str_bayer == string("rggb"))
          bayer = DC1394_COLOR_FILTER_RGGB;
        else if (str_bayer == string("gbrg"))
          bayer = DC1394_COLOR_FILTER_GBRG;
        else if (str_bayer == string("grbg"))
          bayer = DC1394_COLOR_FILTER_GRBG;
        else if (str_bayer == string("bggr"))
          bayer = DC1394_COLOR_FILTER_BGGR;
        else
          colorize = false;
      }

      bool rectify = false;
      param(cd.name + string("/rectify"), rectify, false);

      videre_cam::VidereMode videre_mode;
      int texture_thresh = 12;;
      int unique_thresh = 12;
      bool companding = false;
      bool hdr = false;
      
      if (cd.cam_type == VIDERE)
      {
        string str_videre_mode;
        param(cd.name + string("/videre_param/mode"), str_videre_mode, string("none"));
        if (str_videre_mode == string("none"))
          videre_mode = videre_cam::PROC_MODE_NONE;
        else if (str_videre_mode == string("rectified"))
          videre_mode = videre_cam::PROC_MODE_RECTIFIED;
        else if (str_videre_mode == string("disparity"))
          videre_mode = videre_cam::PROC_MODE_DISPARITY;
        else if (str_videre_mode == string("disparity_raw"))
          videre_mode = videre_cam::PROC_MODE_DISPARITY_RAW;
        else
          videre_mode = videre_cam::PROC_MODE_NONE;

        param(cd.name + string("/videre_param/companding"), companding, false);

        param(cd.name + string("/videre_param/HDR"), hdr, false);

        param(cd.name + string("/videre_param/texture_thresh"), texture_thresh, 12);
        param(cd.name + string("/videre_param/unique_thresh"), unique_thresh, 12);
      }

      printf("Opening camera with guid: %llx\n", guid);
      
      try
      {
        if (cd.cam_type == VIDERE)
        {
          cd.cam = new videre_cam::VidereCam(guid,
                                             videre_mode,
                                             rectify,
                                             speed,
                                             fps,
                                             buffer_size);

        }
        else
        {
          cd.cam = new dc1394_cam::Cam(guid,
                                       speed,
                                       mode,
                                       fps,
                                       buffer_size);

          if (colorize)
            cd.cam->enableColorization(bayer);
        }
      } catch(dc1394_cam::CamException e)
      {
        printf("Failed opening camera: %s\n", e.what());
        cd.cleanup();
        continue;
      }

      checkAndSetFeature(cd, "brightness", DC1394_FEATURE_BRIGHTNESS);
      checkAndSetFeature(cd, "exposure", DC1394_FEATURE_EXPOSURE);
      checkAndSetFeature(cd, "shutter", DC1394_FEATURE_SHUTTER);
      checkAndSetFeature(cd, "gamma", DC1394_FEATURE_GAMMA);
      checkAndSetFeature(cd, "gain", DC1394_FEATURE_GAIN);

      cd.cam->start();


      if (cd.cam_type == VIDERE)
      {
        advertise<std_msgs::String>(cd.name + string("/cal_params"));
        advertise<std_msgs::ImageArray>(cd.name + string("/images"));
      } else {
        advertise<std_msgs::Image>(cd.name + string("/image"));
      }

      cams_.push_back(cd);
    }

    next_time_ = ros::Time::now();
    count_ = 0;
  }


  ~Dc1394CamServer()
  {
    for (list<CamData>::iterator c = cams_.begin(); c != cams_.end(); c++)
      c->cleanup();

    dc1394_cam::fini();  
  }

  //TODO: Move this into image codec?
  void frameToImage(dc1394_cam::FrameWrapper& fw, std_msgs::Image& img)
  {
    uint8_t *buf      = fw.getFrame()->image;
    uint32_t width    = fw.getFrame()->size[0];
    uint32_t height   = fw.getFrame()->size[1];
    uint32_t buf_size = width * height;
      
    img.width  = width;
    img.height = height;
    img.compression = "raw";
    img.label = fw.getName();

    if (fw.getFrame()->color_coding == DC1394_COLOR_CODING_RGB8)
    {
      img.colorspace = "rgb24";
      buf_size *= 3;
    } else {
      img.colorspace = "mono8";
    }
      
    img.set_data_size(buf_size);

    memcpy(img.data, buf, buf_size);
  }


  void serviceCam(CamData& cd)
  {

    dc1394_cam::FrameSet fs = cd.cam->getFrames(DC1394_CAPTURE_POLICY_POLL);

    if (fs.size() > 0)
    {
      img_.set_images_size(fs.size());
      int i = 0;

      for (dc1394_cam::FrameSet::iterator fs_iter = fs.begin();
           fs_iter != fs.end();
           fs_iter++)
      {
        frameToImage(*fs_iter, img_.images[i++]);
        fs_iter->releaseFrame();
        count_++;
      }

      if (cd.cam_type == VIDERE)
      {
        publish(cd.name + "/images", img_);
      } else {
        publish(cd.name + "/image", img_.images[0]);
      }
    }    
  }

  bool getAndSend()
  {

    dc1394_cam::waitForData(1000000);

    for (list<CamData>::iterator c = cams_.begin(); c != cams_.end(); c++)
      serviceCam(*c);

    ros::Time now_time = ros::Time::now();
    if (now_time > next_time_) {
      std::cout << count_ << " imgs/sec at " << now_time << std::endl;
      count_ = 0;
      next_time_ = next_time_ + ros::Duration(1,0);
    }

    return true;

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  //Keep things from dying poorly
  signal(SIGHUP, ros::basic_sigint_handler);
  signal(SIGPIPE, ros::basic_sigint_handler);

  
  Dc1394CamServer dc;

  while (dc.ok() && dc.cams_.size() > 0) {
    dc.getAndSend();
  }

  ros::fini();
  return 0;
}

