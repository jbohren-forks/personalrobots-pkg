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

#include <image_msgs/Image.h>
#include <opencv_latest/CvBridge.h>
#include <ros/ros.h>

#include "sensor_msgs/CompressedImage.h"
#include "opencv/cv.h"

#include "jpeg/SetQuality.h"

#include "jpeg.h"

image_msgs::CvBridge g_img_bridge;

ros::Publisher jpegPub;

unsigned char *jpegBuffer = NULL;
int jpegBufferSize = 0;
int jpegQuality = 80;

std::string pubTopicName = "jpeg_image";

sensor_msgs::CompressedImage compressedImageMessage;

////////////////////////////////////////////////////////////////////////////////
// Image callback
void imageCB(const image_msgs::ImageConstPtr &image)
{
  CvSize size;
  uint32_t width;
  uint32_t height;
  int compressedSize;
  IplImage *ipl;
  unsigned char *rawBuffer = NULL;
  int rawBufferSize = 0;
  int depth;

  g_img_bridge.fromImage(*image, image->encoding);

  if (image->encoding == "mono")
  {
    depth = 1;
  }
  else
  {
    depth = 3;
  }

  // Get the raw image attributes
  ipl = g_img_bridge.toIpl();
  size = cvGetSize(ipl);

  width = size.width;
  height = size.height;

  rawBuffer = (unsigned char*)(ipl->imageData);
  rawBufferSize = ipl->imageSize;

  // Make sure our buffers can handle the data
  if (rawBufferSize > jpegBufferSize)
  {
    if (jpegBuffer)
      delete [] jpegBuffer;

    jpegBufferSize = rawBufferSize;
    jpegBuffer = new unsigned char[jpegBufferSize];
  } 

  // Compress the raw image
  compressedSize = jpeg_compress(jpegBuffer, rawBuffer, width, height, depth, 
                                 jpegBufferSize, jpegQuality );

  compressedImageMessage.encoding = image->encoding;

  // Create the output message
  compressedImageMessage.uint8_data.layout.dim.resize(2);
  compressedImageMessage.uint8_data.layout.dim[0].label = "height";
  compressedImageMessage.uint8_data.layout.dim[0].size = height;
  compressedImageMessage.uint8_data.layout.dim[0].stride = 0;

  compressedImageMessage.uint8_data.layout.dim[1].label = "width";
  compressedImageMessage.uint8_data.layout.dim[1].size = width;
  compressedImageMessage.uint8_data.layout.dim[1].stride = 0;

  compressedImageMessage.uint8_data.data.resize(compressedSize);
  memcpy((char*)(&compressedImageMessage.uint8_data.data[0]), 
         (char*)jpegBuffer, compressedSize);

  // Send the message
  jpegPub.publish(compressedImageMessage);

}

////////////////////////////////////////////////////////////////////////////////
// Set the quality of the jpeg compression output
bool setQuality( jpeg::SetQuality::Request &req, 
                 jpeg::SetQuality::Response &resp)
{
  if (req.quality <100)
    jpegQuality = req.quality;
  else
    jpegQuality = 100;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "encoder");
  ros::ServiceServer service;
  ros::Subscriber rawSub;
  ros::NodeHandle nodeHandle;

  // Subscribe to the raw image topic
  rawSub = nodeHandle.subscribe("raw_image",1, imageCB);

  // Advertise a service that can set the jpeg quality
  service = nodeHandle.advertiseService("set_jpeg_quality", &setQuality);

  // Create the publisher to output jpeg images
  jpegPub = nodeHandle.advertise<sensor_msgs::CompressedImage>(pubTopicName,1);

  // Setup some basic stuff for the compressed image message
  compressedImageMessage.label = "jpeg image";
  compressedImageMessage.format = "jpeg";

  // Run it baby
  ros::spin();

  // Cleanup
  if (jpegBuffer)
    delete [] jpegBuffer;

  jpegBuffer = NULL;

  rawSub.shutdown();
  jpegPub.shutdown();
  nodeHandle.shutdown();

  return 0;
}
