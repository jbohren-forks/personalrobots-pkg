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
#include "dc1394_cam/dc1394_cam.h"

#include <vector>
#include <list>
#include <sstream>

using namespace std;

enum CamTypes {NORMAL, VIDERE};

class OtherData
{
public:
  OtherData() {}
  virtual ~OtherData() {}
};

class VidereData : public OtherData
{
public:
  int32_t mode;
  int32_t textureThresh;
  int32_t uniqueThresh;
  virtual ~VidereData() {}
};

class CamData
{
public:
  CamData() : cam(NULL), otherData(NULL) {}

  void cleanup()
  {
    if (cam)
      delete cam;
    if (otherData)
      delete otherData;
  }

  string name;
  dc1394_cam::Cam* cam;
  std_msgs::Image img;
  dc1394color_filter_t bayer;
  bool colorize;
  CamTypes  camType;
  OtherData* otherData;
};

class Dc1394Node : public ros::node
{
public:
  ros::Time next_time;

  list<CamData> cams;
  
  int count;


  void checkAndSetFeature(CamData& cd, string paramName, dc1394feature_t feature)
  {
    string p = cd.name + string("/") + paramName;
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


  Dc1394Node() : ros::node("dc1394_node")
  {

    dc1394_cam::init();

    int numCams;
    param("numCams", numCams, 1);

    for (int i = 0; i < numCams; i++)
    {
      CamData cd;

      ostringstream oss;
      oss << "cam" << i;

      param(oss.str(), cd.name, oss.str());

      advertise<std_msgs::Image>(cd.name + string("/image")); //Do this later?

      uint64_t guid;
      if (has_param(cd.name + string("/guid")))
      {
        string guidStr;
        get_param(cd.name + string("/guid"), guidStr);
        
        guid = strtoll(guidStr.c_str(), NULL, 16);
      } else {
        guid = dc1394_cam::getGuid(i);
      }

      string strSpeed;
      dc1394speed_t speed;
      param(cd.name + string("/speed"), strSpeed, string("S400"));
      if (strSpeed == string("S100"))
        speed = DC1394_ISO_SPEED_100;
      else if (strSpeed == string("S200"))
        speed = DC1394_ISO_SPEED_200;
      else
        speed = DC1394_ISO_SPEED_400;

      double dblFps;
      dc1394framerate_t fps;
      param(cd.name + string("/fps"), dblFps, 30.0);
      if (dblFps >= 240.0)
        fps = DC1394_FRAMERATE_240;
      else if (dblFps >= 120.0)
        fps = DC1394_FRAMERATE_120;
      else if (dblFps >= 60.0)
        fps = DC1394_FRAMERATE_60;
      else if (dblFps >= 30.0)
        fps = DC1394_FRAMERATE_30;
      else if (dblFps >= 15.0)
        fps = DC1394_FRAMERATE_15;
      else if (dblFps >= 7.5)
        fps = DC1394_FRAMERATE_7_5;
      else if (dblFps >= 3.75)
        fps = DC1394_FRAMERATE_3_75;
      else
        fps = DC1394_FRAMERATE_1_875;

      cd.camType = NORMAL;

      string strMode;
      dc1394video_mode_t mode;
      param(cd.name + string("/videoMode"), strMode, string("640x480mono8"));
      if (strMode == string("640x480rgb24"))
        mode = DC1394_VIDEO_MODE_640x480_RGB8;
      else if (strMode == string("1024x768rgb24"))
        mode = DC1394_VIDEO_MODE_1024x768_RGB8;
      else if (strMode == string("1280x960rgb24"))
        mode = DC1394_VIDEO_MODE_1280x960_RGB8;
      else if (strMode == string("1600x1200rgb24"))
        mode = DC1394_VIDEO_MODE_1600x1200_RGB8;
      else if (strMode == string("640x480mono8"))
        mode = DC1394_VIDEO_MODE_640x480_MONO8;
      else if (strMode == string("1024x768mono8"))
        mode = DC1394_VIDEO_MODE_1024x768_MONO8;
      else if (strMode == string("1280x960mono8"))
        mode = DC1394_VIDEO_MODE_1280x960_MONO8;
      else if (strMode == string("1600x1200mono8"))
        mode = DC1394_VIDEO_MODE_1600x1200_MONO8;
      else if (strMode == string("640x480videre"))
      {
        mode = DC1394_VIDEO_MODE_640x480_YUV422;
        cd.camType = VIDERE;
      }
      else
        mode = DC1394_VIDEO_MODE_640x480_MONO8;

      int bufferSize;
      param(cd.name + string("/bufferSize"), bufferSize, 8);

      cd.colorize = false;

      if ( mode == DC1394_VIDEO_MODE_640x480_MONO8 ||
           mode == DC1394_VIDEO_MODE_1024x768_MONO8 ||
           mode == DC1394_VIDEO_MODE_1280x960_MONO8 ||
           mode == DC1394_VIDEO_MODE_1600x1200_MONO8)
      {
        cd.colorize = true;
        string bayer;
        param(cd.name + string("/bayer"), bayer, string("none"));

        if (bayer == string("rggb"))
          cd.bayer = DC1394_COLOR_FILTER_RGGB;
        else if (bayer == string("gbrg"))
          cd.bayer = DC1394_COLOR_FILTER_GBRG;
        else if (bayer == string("grbg"))
          cd.bayer = DC1394_COLOR_FILTER_GRBG;
        else if (bayer == string("bggr"))
          cd.bayer = DC1394_COLOR_FILTER_BGGR;
        else
          cd.colorize = false;
      }

      if (cd.camType == VIDERE)
      {
        cd.otherData = new VidereData;

        string strVidereMode;
        param(cd.name + string("/videreParam/mode"), strVidereMode, string("none"));
        if (strVidereMode == string("none"))
          ((VidereData*)(cd.otherData))->mode = 1;
        else if (strVidereMode == string("rectified"))
          ((VidereData*)(cd.otherData))->mode = 3;
        else if (strVidereMode == string("disparity"))
          ((VidereData*)(cd.otherData))->mode = 4;
        else if (strVidereMode == string("disparity_raw"))
          ((VidereData*)(cd.otherData))->mode = 5;
        else
          ((VidereData*)(cd.otherData))->mode = 4;

        int textureThresh;
        param(cd.name + string("/videreParam/textureThresh"), textureThresh, 12);
        if (textureThresh < 0)
          textureThresh = 0;
        if (textureThresh > 63)
          textureThresh = 63;
        ((VidereData*)(cd.otherData))->textureThresh = textureThresh;

        int uniqueThresh;
        param(cd.name + string("/videreParam/uniqueThresh"), uniqueThresh, 12);
        if (uniqueThresh < 0)
          uniqueThresh = 0;
        if (uniqueThresh > 63)
          uniqueThresh = 63;
        ((VidereData*)(cd.otherData))->uniqueThresh = uniqueThresh;
      }

      printf("Opening camera with guid: %llx\n", guid);
      
      try
      {

      cd.cam = new dc1394_cam::Cam(guid,
                                   speed,
                                   mode,
                                   fps,
                                   bufferSize);
      } catch(dc1394_cam::CamException e)
      {
        printf("Failed opening camera: %s\n", e.what());
        cd.cleanup();
        continue;
      }

      // Make sure camera is actually a videre...
      if (cd.camType == VIDERE)
      {
        if (cd.cam->dcCam->vendor_id != 0x5505)
        {
          printf("Not a videre camera!\n");
          cd.cleanup();
          continue;
        }
      }

      checkAndSetFeature(cd, "brightness", DC1394_FEATURE_BRIGHTNESS);
      checkAndSetFeature(cd, "exposure", DC1394_FEATURE_EXPOSURE);
      checkAndSetFeature(cd, "shutter", DC1394_FEATURE_SHUTTER);
      checkAndSetFeature(cd, "gamma", DC1394_FEATURE_GAMMA);
      checkAndSetFeature(cd, "gain", DC1394_FEATURE_GAIN);

      cd.cam->start();

      //  VIDERE mode setup has to happen AFTER starting the camera

      if (cd.camType == VIDERE)
      {
        usleep(50000);
        uint32_t t_thresh = 0x08000000 | (0x40 << 16) | ( ((VidereData*)(cd.otherData))->textureThresh << 16);
        cd.cam->setControlRegister(0xFF000, t_thresh);

        usleep(50000);
        uint32_t u_thresh = 0x08000000 | (0x00 << 16) | ( ((VidereData*)(cd.otherData))->uniqueThresh << 16);
        cd.cam->setControlRegister(0xFF000, u_thresh);

        usleep(50000);
        uint32_t qval1 = 0x08000000 | (0x90 << 16) | ( ( ((VidereData*)(cd.otherData))->mode & 0x7) << 16);
        uint32_t qval2 = 0x08000000 | (0x9C << 16);

        cd.cam->setControlRegister(0xFF000, qval1);
        cd.cam->setControlRegister(0xFF000, qval2);
      }

      cams.push_back(cd);
    }

    next_time = ros::Time::now();
    count = 0;
  }


  
        

  ~Dc1394Node()
  {
    for (list<CamData>::iterator i = cams.begin(); i != cams.end(); i++)
      i->cleanup();

    dc1394_cam::fini();  
  }

  bool getAndSend()
  {

    dc1394_cam::waitForData(1000000);

    for (list<CamData>::iterator i = cams.begin(); i != cams.end(); i++)
    {

      dc1394video_frame_t *in_frame = i->cam->getFrame(DC1394_CAPTURE_POLICY_POLL);
      dc1394video_frame_t *frame;

      if (in_frame != NULL)
      {

        if (i->colorize)
        {
          frame = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
          in_frame->color_filter = i->bayer;

          if (dc1394_debayer_frames(in_frame, frame, DC1394_BAYER_METHOD_BILINEAR) != DC1394_SUCCESS)
            printf("Debayering failed!\n");
        } else if (i->camType == VIDERE) {
          frame = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));

          dc1394_deinterlace_stereo_frames(in_frame, frame, DC1394_STEREO_METHOD_INTERLACED);
        } else {
          frame = in_frame;
        } 

        uint8_t *buf      = frame->image;
        uint32_t width    = frame->size[0];
        uint32_t height   = frame->size[1];
        uint32_t buf_size = width*height;
        
        i->img.width = width;
        i->img.height = height;
        i->img.compression = "raw";

        if (frame->color_coding == DC1394_COLOR_CODING_RGB8)
        {
          i->img.colorspace = "rgb24";
          buf_size *= 3;
        } else {
          i->img.colorspace = "mono8";
        }

        i->img.set_data_size(buf_size);
        memcpy(i->img.data, buf, buf_size);

        publish(i->name + string("/image"), i->img);

        if (i->colorize || i->camType == VIDERE) {
          free(frame->image);
          free(frame);
        }

        i->cam->releaseFrame(in_frame);

        count++;

      }
    }

    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << " imgs/sec at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }

    return true;

  }
  
  /*
  bool get_and_send_jpeg() 
  {
    const uint8_t *buf;
    uint32_t buf_size;
    flea2.get_jpeg(&buf, &buf_size);

    img.width = flea2.cam.frame_width;
    img.height = flea2.cam.frame_height;
    img.compression = "jpeg";
    img.colorspace = "mono8";
    img.set_data_size(buf_size);
    memcpy(img.data, buf, buf_size);


    count++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << " imgs/sec at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }

    publish("image", img);
    return true;
  }
  */

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  
  Dc1394Node dc;

  while (dc.ok() && dc.cams.size() > 0) {
    dc.getAndSend();
  }

  ros::fini();
  return 0;
}

