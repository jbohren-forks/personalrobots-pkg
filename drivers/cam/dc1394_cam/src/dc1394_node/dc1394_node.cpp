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

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>

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
  std_msgs::String cal_params;
  int32_t mode;
  int32_t textureThresh;
  int32_t uniqueThresh;
  bool    companding;
  bool    HDR;
  double  Cx;
  double  Cy;
  double  Tx;
  double  f;
  int w;
  int h;
  int corrs;
  int logs;
  int offx;
  int dleft;
  int dwidth;
  int dtop;
  int dheight;
  std_msgs::ImageArray imgs;
  NEWMAT::Matrix lproj;
  NEWMAT::Matrix rproj;
  std_msgs::PointCloudFloat32 cloud;
  VidereData() : lproj(3,4), rproj(3,4) {}
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
  bool colorize;
  dc1394color_filter_t bayer;
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
        VidereData* v = new VidereData;
        cd.otherData = v;

        string strVidereMode;
        param(cd.name + string("/videreParam/mode"), strVidereMode, string("none"));
        if (strVidereMode == string("none"))
          v->mode = 1;
        else if (strVidereMode == string("rectified"))
          v->mode = 3;
        else if (strVidereMode == string("disparity"))
          v->mode = 4;
        else if (strVidereMode == string("disparity_raw"))
          v->mode = 5;
        else
          v->mode = 4;

        int textureThresh;
        param(cd.name + string("/videreParam/textureThresh"), textureThresh, 12);
        if (textureThresh < 0)
          textureThresh = 0;
        if (textureThresh > 63)
          textureThresh = 63;
        v->textureThresh = textureThresh;

        int uniqueThresh;
        param(cd.name + string("/videreParam/uniqueThresh"), uniqueThresh, 12);
        if (uniqueThresh < 0)
          uniqueThresh = 0;
        if (uniqueThresh > 63)
          uniqueThresh = 63;
        v->uniqueThresh = uniqueThresh;

        int companding;
        param(cd.name + string("/videreParam/companding"), companding, 0);
        if (companding)
          v->companding = true;

        int HDR;
        param(cd.name + string("/videreParam/HDR"), HDR, 0);
        if (HDR)
          v->HDR = true;

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
        VidereData* v = ((VidereData*)(cd.otherData));

        if (cd.cam->dcCam->vendor_id != 0x5505)
        {
          printf("Not a videre camera!\n");
          cd.cleanup();
          continue;
        }

        // Extract the Videre calibration file:
        // check for calibration
        uint32_t qval = cd.cam->getControlRegister(0xF0800);
        if (qval == 0xffffffff)
        {
          printf("No calibration parameters found for Videre");
          cd.cleanup();
          continue;
        }

        char buf[4096*4];
        int n = 4096*4;
        char* bb = buf;

        // read in each byte
        int pos = 0;
        uint32_t quad;
        quad = cd.cam->getControlRegister(0xF0800+pos);

        while (quad != 0x0 && quad != 0xffffffff && n > 3)
        {
          int val;
          pos += 4;
          n -= 4;
          val = (quad >> 24) & 0xff;
          *bb++ = val;
          val = (quad >> 16) & 0xff;
          *bb++ = val;
          val = (quad >> 8) & 0xff;
          *bb++ = val;
          val = quad & 0xff;
          *bb++ = val;
          quad = cd.cam->getControlRegister(0xF0800+pos);
        }
        *bb = 0;                      // just in case we missed the last zero

        v->cal_params.data = buf;

        istringstream iss( v->cal_params.data.substr( v->cal_params.data.find("proj", v->cal_params.data.find("[left camera]") ) + strlen("proj")) );
        
        for (int i = 1; i <= 3; i++)
          for (int j = 1; j <= 4; j++)
            iss >> v->lproj(i,j);

        iss.str( v->cal_params.data.substr( v->cal_params.data.find("proj", v->cal_params.data.find("[right camera]") ) + strlen("proj")) );
        
        for (int i = 1; i <= 3; i++)
          for (int j = 1; j <= 4; j++)
            iss >> v->rproj(i,j);

        v->Cx = v->lproj(1,3);
        v->Cy = v->lproj(2,3);
        v->Tx = -v->rproj(1,4)/v->lproj(1,1) / 1000.0;
        v->f = v->lproj(1,1);


        iss.str( v->cal_params.data.substr( v->cal_params.data.find("width", v->cal_params.data.find("[image]") ) + strlen("width")) );
        iss >> v->w;

        iss.str( v->cal_params.data.substr( v->cal_params.data.find("height", v->cal_params.data.find("[image]") ) + strlen("height")) );
        iss >> v->h;

        iss.str( v->cal_params.data.substr( v->cal_params.data.find("corrxsize", v->cal_params.data.find("[stereo]") ) + strlen("corrxsize")) );
        iss >> v->corrs;

        v->logs = 9;

        iss.str( v->cal_params.data.substr( v->cal_params.data.find("offx", v->cal_params.data.find("[stereo]") ) + strlen("offx")) );
        iss >> v->offx;

        v->dleft   = (v->logs + v->corrs - 2)/2 - 1 + v->offx;
        v->dwidth  = v->w - (v->logs + v->corrs + v->offx - 2);
        v->dtop    = (v->logs + v->corrs - 2)/2;
        v->dheight = v->h - (v->logs + v->corrs);

        /*
        printf("Read in file: %s\n", params.c_str());
        std::cout << v->lproj << std::endl << std::endl;
        std::cout << v->rproj << std::endl << std::endl;
        
        std::cout << "Disparity shift params: "
                  << v->w << ", "
                  << v->h << ", "
                  << v->corrs << ", "
                  << v->logs << ", "
                  << v->offx << ", "
                  << v->dleft << ", "
                  << v->dwidth << ", "
                  << v->dtop << ", "
                  << v->dheight << std::endl;
        */

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
        VidereData* v = ((VidereData*)(cd.otherData));

        usleep(50000);
        uint32_t t_thresh = 0x08000000 | (0x40 << 16) | ( v->textureThresh << 16);
        cd.cam->setControlRegister(0xFF000, t_thresh);

        usleep(50000);
        uint32_t u_thresh = 0x08000000 | (0x00 << 16) | ( v->uniqueThresh << 16);
        cd.cam->setControlRegister(0xFF000, u_thresh);

        usleep(50000);
        uint32_t qval1 = 0x08000000 | (0x90 << 16) | ( ( v->mode & 0x7) << 16);
        uint32_t qval2 = 0x08000000 | (0x9C << 16);

        cd.cam->setControlRegister(0xFF000, qval1);
        cd.cam->setControlRegister(0xFF000, qval2);

        usleep(50000);
        if (v->companding)
          cd.cam->setControlRegister(0xFF000, 0x041C0003);
        else
          cd.cam->setControlRegister(0xFF000, 0x041C0002);

        usleep(50000);
        if (v->HDR)
          cd.cam->setControlRegister(0xFF000, 0x040F0051);
        else
          cd.cam->setControlRegister(0xFF000, 0x040F0011);
          
        
      }


      if (cd.camType == VIDERE)
      {
        VidereData* v = ((VidereData*)(cd.otherData));

        advertise<std_msgs::String>(cd.name + string("/cal_params"));
        advertise<std_msgs::ImageArray>(cd.name + string("/images"));
        v->imgs.set_images_size(2);

        switch (v->mode)
        {
        case 3:
          v->imgs.images[0].label = string("right_rectified");
          v->imgs.images[1].label = string("left_rectified");
          break;
        case 4:
          advertise<std_msgs::PointCloudFloat32>(cd.name + string("/cloud"));
          advertise<std_msgs::Empty>(cd.name + string("/shutter"));
          
          v->imgs.set_images_size(2);
          v->imgs.images[0].label = string("left_disparity");
          v->imgs.images[1].label = string("left_rectified");

          break;
       case 5:
          v->imgs.images[0].label = string("left_disparity");
          v->imgs.images[1].label = string("left_rectified");
          break;
        default:
          v->imgs.images[0].label = string("right");
          v->imgs.images[1].label = string("left");
        }
      } else {
        advertise<std_msgs::Image>(cd.name + string("/image"));
      }

      cams.push_back(cd);
    }

    next_time = ros::Time::now();
    count = 0;
  }


  
        

  ~Dc1394Node()
  {
    for (list<CamData>::iterator c = cams.begin(); c != cams.end(); c++)
      c->cleanup();

    dc1394_cam::fini();  
  }

  bool getAndSend()
  {

    dc1394_cam::waitForData(1000000);

    for (list<CamData>::iterator c = cams.begin(); c != cams.end(); c++)
    {

      dc1394video_frame_t *in_frame = c->cam->getFrame(DC1394_CAPTURE_POLICY_POLL);
      dc1394video_frame_t *frame;

      if (in_frame != NULL)
      {

        if (c->colorize)
        {
          frame = c->cam->debayerFrame(in_frame, c->bayer);
        } else if (c->camType == VIDERE) {  // THIS IS HACK TO avoid frame being set equal to in_frame again...
          frame = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));

          dc1394_deinterlace_stereo_frames(in_frame, frame, DC1394_STEREO_METHOD_INTERLACED);
        } else {
          frame = in_frame;
        } 

        if (c->camType == VIDERE)
        {
          VidereData* v = (VidereData*)(c->otherData);

          uint8_t *buf      = frame->image;
          uint32_t width    = frame->size[0];
          uint32_t height   = frame->size[1]/2;
          uint32_t buf_size = width*height;

          // Fix disparity window location

          if (v->mode == 4)
          {
            for (int i = 0; i < v->dheight; i++)
            {
              memcpy(buf + (v->dtop + i)*v->w + v->dleft - 6, buf + (v->h - v->dheight + i)*v->w + v->w - v->dwidth, v->dwidth);
              memset(buf + (v->dtop + i)*v->w + v->dleft - 6 + v->dwidth, 0, v->w - v->dwidth - v->dleft + 6);
            }
            for (int i = v->dheight + v->dtop; i < v->h; i++)
            {
              memset(buf + i*v->w, 0, v->w);
            }
          }
        
          v->imgs.images[0].width = width;
          v->imgs.images[0].height = height;
          v->imgs.images[0].colorspace = "mono8";
          v->imgs.images[0].compression = "raw";

          v->imgs.images[1].width = width;
          v->imgs.images[1].height = height;
          v->imgs.images[1].colorspace = "mono8";
          v->imgs.images[1].compression = "raw";

          v->imgs.images[0].set_data_size(buf_size);
          memcpy(v->imgs.images[0].data, buf, buf_size);

          v->imgs.images[1].set_data_size(buf_size);
          memcpy(v->imgs.images[1].data, buf + buf_size, buf_size);

          if (count < cams.size())
            publish(c->name + string("/cal_params"), v->cal_params);

          switch (v->mode)
          {
          case 4:
            {
            int goodPixCount = 0;
            for (uint32_t i = 0; i < buf_size; i++)
              if (buf[i] != 0)
                goodPixCount++;

            v->cloud.set_pts_size(goodPixCount);
            v->cloud.set_chan_size(1);
            v->cloud.chan[0].name = "intensities";
            v->cloud.chan[0].set_vals_size(goodPixCount);

            int j = 0;
            for (uint32_t i = 0; i < buf_size; i++)
              if (buf[i] != 0)
              {
                double X = v->Tx * ( (double)(i % width) - v->Cx ) / ((double)(buf[i]) / 4.0);
                double Y = v->Tx * ( (double)(i / width) - v->Cy ) / ((double)(buf[i]) / 4.0);
                double Z = v->Tx * ( v->f ) / ((double)(buf[i]) / 4.0);

                v->cloud.pts[j].y = - X;
                v->cloud.pts[j].z = - Y;
                v->cloud.pts[j].x = Z;

                v->cloud.chan[0].vals[j] = buf[buf_size + i];
                j++;
              }

            std_msgs::Empty e;
            publish(c->name + string("/cloud"), v->cloud);
            publish(c->name + string("/shutter"), e);

            publish(c->name + string("/images"), v->imgs);
            }
            break;
          default:
            publish(c->name + string("/images"), v->imgs);
            break;
          }
        } else {

          uint8_t *buf      = frame->image;
          uint32_t width    = frame->size[0];
          uint32_t height   = frame->size[1];
          uint32_t buf_size = width*height;
        
          c->img.width = width;
          c->img.height = height;
          c->img.compression = "raw";

          if (frame->color_coding == DC1394_COLOR_CODING_RGB8)
          {
            c->img.colorspace = "rgb24";
            buf_size *= 3;
          } else {
            c->img.colorspace = "mono8";
          }

          c->img.set_data_size(buf_size);
          memcpy(c->img.data, buf, buf_size);

          publish(c->name + string("/image"), c->img);

        }

        if (c->camType == VIDERE) {
          free(frame->image);
          free(frame);
        }

        c->cam->releaseFrame(in_frame);

        if (c->colorize)
          c->cam->freeFrame(frame);

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
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  //Keep things from dying poorly
  signal(SIGHUP, ros::basic_sigint_handler);
  signal(SIGPIPE, ros::basic_sigint_handler);

  
  Dc1394Node dc;

  while (dc.ok() && dc.cams.size() > 0) {
    dc.getAndSend();
  }

  ros::fini();
  return 0;
}

