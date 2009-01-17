/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Jimmy Sastra
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

#include <iostream>
#include <fstream>
#include "elphel_cam/elphel_cam.h"
#include "ros/node.h"

// OpenCV libraries
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <ctype.h>
#include "std_msgs/Image.h"
#include "image_utils/cv_bridge.h"


#include "image_utils/image_codec.h"

using namespace std;


class Elphel_Node : public ros::Node
{
public:
  std_msgs::Image image_msg;
  CvBridge<std_msgs::Image> cv_bridge;
  ImageCodec<std_msgs::Image> codec;

  uint8_t* jpeg;
  uint32_t jpeg_size;


  Elphel_Cam *e; //("10.12.0.103");
  IplImage* img;

  Elphel_Node() : ros::Node("elphel"), cv_bridge(&image_msg), codec(&image_msg)
  {
      advertise<std_msgs::Image>("elphel_bus");
      e = new Elphel_Cam("10.12.0.103");
//      e->init(10, 4, 4);
      e->init(1, 4, 4);
      e->start();
  }


  bool getFrame() {
    static int i = 0;
    i++;

    if (!e->next_jpeg(&jpeg, &jpeg_size))
    {
      ROS_ERROR("Elphel_Cam::next_jpeg returned an error");
      return false;
    }

    // Sometimes things break for no great reason.  When this happens
    // the http server returns a 1x1 GIF.
    // Skip failures and re-init cam if we get too many of them
    int failcount = 0;
    while (jpeg[0] == 0x47 && jpeg[1] == 0x49) { //THIS IS A GIF
      if (failcount++ > 10) {
      	printf("Received too many failures... restarting cam\n");
	      e->init(10, 4, 4);
	      e->start();
	      failcount = 0;
      }
      if (!e->next_jpeg(&jpeg, &jpeg_size))
      {
	      ROS_ERROR("Elphel_Cam::next_jpeg returned an error");
	      return false;
      }
    }




    if(e->next_jpeg(&jpeg, &jpeg_size)) {
      assert(jpeg != NULL);
      image_msg.set_data_size(jpeg_size);
      memcpy(&image_msg.data[0], jpeg, jpeg_size);
      image_msg.compression = "jpeg";
      image_msg.colorspace = "rgb24";
      codec.inflate_header();
      publish("elphel_bus", image_msg);
      printf("releasing frames %i\n", i);
    }
    return true;
  }

  virtual ~Elphel_Node() {
    if(e) {
      e->stop();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Elphel_Node n;
  while(n.ok()) {
    if(!n.getFrame())
    {
      printf("Elphel camera failed.\n");
      ROS_ERROR("Elphel camera failed.");
      break;
    }
  }

  ros::fini();
  return 0;
}
