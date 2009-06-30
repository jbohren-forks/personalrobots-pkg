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

#include <stdio.h>

#include <stdexcept>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <opencv_latest/CvBridge.h>

#include <neven/neven.h>

sensor_msgs::CvBridge g_img_bridge;

neven::FaceDetector* fd;

void imageCB(const sensor_msgs::ImageConstPtr& image)
{
  if (g_img_bridge.fromImage(*image, "mono"))
  {
    CvSize size = cvGetSize(g_img_bridge.toIpl());

    uint32_t width = size.width;
    uint32_t height = size.height;

    char* bwbuffer = g_img_bridge.toIpl()->imageData;


    std::vector<neven::Face> faces = fd->findFaces(bwbuffer, width, height);

    printf("Found %d faces!\n", faces.size());

    for (std::vector<neven::Face>::iterator i = faces.begin();
         i != faces.end();
         i++)
    {
      cvCircle(g_img_bridge.toIpl(), cvPoint(i->midpointx, i->midpointy), i->eyedist, cvScalar(255, 0, 0, 0), 1, 8, 0);
    }

      
    cvShowImage("image", g_img_bridge.toIpl());

    cvWaitKey(3);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "neven_face_detector");
  ros::NodeHandle nh;


  fd = new neven::FaceDetector(640,480,5,5,100,neven::CONFIG_SPEED);

  ros::Subscriber image_sub = nh.subscribe("image", 1, imageCB);

  cvNamedWindow("image", CV_WINDOW_AUTOSIZE);

  ros::spin();

  delete fd;

  return 0;
}
