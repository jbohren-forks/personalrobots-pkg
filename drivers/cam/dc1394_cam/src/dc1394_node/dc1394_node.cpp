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
#include <sstream>

using namespace std;

struct camData
{
  dc1394_cam::Cam* cam;
  std_msgs::Image img;
  bool colorize;
  dc1394color_filter_t bayer;
  int index;
};

class Dc1394Node : public ros::node
{
public:
  ros::Time next_time;

  vector<camData> cams;
  
  int count;

  Dc1394Node() : ros::node("dc1394_node")
  {

    dc1394_cam::init();

    int numCams;
    param("numCams", numCams, 1);

    for (int i = 0; i < numCams; i++)
    {
      camData cd;
      cd.index = i;

      ostringstream oss;
      oss << "image" << i;

      advertise<std_msgs::Image>(oss.str());

      uint64_t guid;

      oss.str("");
      oss << "guid" << i;
      if (has_param(oss.str()))
      {
        string guidStr;
        get_param(oss.str(), guidStr);

        guid = strtoll(guidStr.c_str(), NULL, 16);
      } else {
        guid = dc1394_cam::getGuid(i);
      }

      oss.str("");
      oss << "bayer" << i;

      cd.colorize = false;
      if (has_param(oss.str()))
      {
        cd.colorize = true;
        string bayer;
        get_param(oss.str(), bayer);
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

      printf("Opening camera with guid: %llx\n", guid);
      
      cd.cam = new dc1394_cam::Cam(guid);

      /*
        set_shutter(0.8);
        set_gamma(0.24);
        set_gain(0);
      */      

      cd.cam->start();

      cams.push_back(cd);
    }

    next_time = ros::Time::now();
    count = 0;
  }

  ~Dc1394Node()
  {
    for (vector<camData>::iterator i = cams.begin(); i != cams.end(); i++)
    {
      delete i->cam;
    }

    dc1394_cam::fini();  
  }

  bool getAndSend()
  {

    for (vector<camData>::iterator i = cams.begin(); i != cams.end(); i++)
    {

      dc1394video_frame_t *in_frame = i->cam->getFrame();
      dc1394video_frame_t *frame;

      if (i->colorize)
      {
        frame = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
        in_frame->color_filter = i->bayer;

        if (dc1394_debayer_frames(in_frame, frame, DC1394_BAYER_METHOD_BILINEAR) != DC1394_SUCCESS)
          printf("Debayering failed!\n");
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

      ostringstream oss;
      oss << "image" << i->index;

      publish(oss.str(), i->img);

      if (i->colorize) {
        free(frame->image);
        free(frame);
      }

      i->cam->releaseFrame(in_frame);

      count++;
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

  while (dc.ok()) {
    dc.getAndSend();
  }

  ros::fini();
  return 0;
}

