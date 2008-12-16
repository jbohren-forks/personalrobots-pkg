/*********************************************************************
* This node takes OpenCV images and tracks blobs 
*
*
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
#include <cstdlib>
#include <fstream>
//#include "elphel_cam/elphel_cam.h"
#include "ros/node.h"

// OpenCV libraries
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <ctype.h>
#include "std_msgs/Image.h"
#include "image_utils/cv_bridge.h"


#include "blob_tracker/blob_tracker.h"
#include <string.h>


using namespace std;

static int i;

class Blob_Tracker_Node : public ros::node
{
public:
  std_msgs::Image image_msg;
  CvBridge<std_msgs::Image> cv_bridge;

  IplImage* img;
  IplImage *cv_image;
  IplImage *frame;
  uint8_t* jpeg;
  uint32_t jpeg_size;
  Blob_Tracker b;
  
  
  Blob_Tracker_Node() : ros::node("blobtracker"), cv_bridge(&image_msg, CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U)
  {
      cvNamedWindow("blobtracker", CV_WINDOW_AUTOSIZE);
      subscribe("elphel_bus",image_msg, &Blob_Tracker_Node::processFrame); 
      b.init();     
  }

  virtual ~Blob_Tracker_Node() {

  }

  void processFrame() 
  {
    if (cv_bridge.to_cv(&cv_image))
    {
/* to save frames as png's:      
        std::stringstream ss;
        ss << "test" << i++ << ".png";
        b.saveFrame(cv_image);
*/
        b.processFrame(&cv_image);
    }
  } 
};




int main(int argc, char **argv) 
{

  ros::init(argc, argv);
  Blob_Tracker_Node n;
  while(n.ok()) {
    usleep(1000);      
  }
  ros::fini();
  return 0;
}
