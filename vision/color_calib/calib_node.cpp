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
#include <vector>
#include <map>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/ImageArray.h"
#include "image_utils/cv_bridge.h"

#include <sys/stat.h>

using namespace std;

struct imgData
{
  string label;
  IplImage *cv_image;
  CvBridge<std_msgs::Image> *bridge;
};

class ColorCalib : public ros::node
{
public:
  std_msgs::ImageArray image_msg;

  ros::thread::mutex cv_mutex;

  char dir_name[256];
  int img_cnt;
  bool made_dir;

  map<string, imgData> images;

  ColorCalib() : node("color_calib", ros::node::ANONYMOUS_NAME), 
             img_cnt(0), made_dir(false)
  { 
    subscribe("images", image_msg, &ColorCalib::image_cb, 1);
  }

  ~ColorCalib()
  {
    for (map<string, imgData>::iterator i = images.begin(); i != images.end(); i++)
    {
      if (i->second.cv_image)
        cvReleaseImage(&i->second.cv_image);
    }
  }

  void image_cb()
  {
    unsubscribe("images");

    cv_mutex.lock();

    for (uint32_t i = 0; i < image_msg.get_images_size(); i++)
    {
      string l = image_msg.images[i].label;
      map<string, imgData>::iterator j = images.find(l);

      if (j == images.end())
      {
        images[l].label = image_msg.images[i].label;
        images[l].bridge = new CvBridge<std_msgs::Image>(&image_msg.images[i], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
        cvNamedWindow(l.c_str(), CV_WINDOW_AUTOSIZE);
        images[l].cv_image = 0;
      } else {

        if (j->second.cv_image)
          cvReleaseImage(&j->second.cv_image);

        if (j->second.bridge->to_cv(&j->second.cv_image))
        {
          cvShowImage(j->second.label.c_str(), j->second.cv_image);
        }
      }
      
    }

    cv_mutex.unlock();
  }

  void check_keys() 
  {
    cv_mutex.lock();
    if (cvWaitKey(3) == 10)
    { }
      //      save_image();
    cv_mutex.unlock();
  }

  /*
  void save_image() 
  {
    if (!made_dir) 
    {
      if (mkdir(dir_name, 0755)) 
      {
        printf("Failed to make directory: %s\n", dir_name);
        return;
      } 
      else 
        made_dir = true;
    }
    std::ostringstream oss;
    oss << dir_name << "/Img" << img_cnt++ << ".png";
    cvSaveImage(oss.str().c_str(), cv_image);
  }
  */
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ColorCalib view;
  while (view.ok()) 
  {
    usleep(10000);
    view.check_keys();
  }
  ros::fini();
  return 0;
}

