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
#include <image_msgs/FillImage.h>
#include <image_msgs/CompressedImage.h>
#include <ros/ros.h>

#include "opencv/cv.h"
#include "jpeg.h"


ros::Publisher rawPub;

unsigned char *rawBuffer = NULL;
unsigned int rawBufferSize = 0;

std::string pubTopicName = "decompressed_image";
image_msgs::Image decompressedImageMessage;

int frameCount = 0;

////////////////////////////////////////////////////////////////////////////////
// Image callback
void imageCB(const image_msgs::CompressedImageConstPtr &image)
{
  int depth = 3;
  unsigned char *jpegBuffer = NULL;
  int jpegBufferSize = 0;

  uint32_t width;
  uint32_t height;

  // Make sure that the message has the right format
  if (image->format != "jpeg")
  {
    ROS_ERROR("Jpeg format expected, but got: %s\n", image->format.c_str());
    return;
  }

  height = image->uint8_data.layout.dim[0].size;
  width = image->uint8_data.layout.dim[1].size;

  // Create the raw buffer
  if (rawBufferSize != width * height * depth)
  {
    if (!rawBuffer)
      delete [] rawBuffer;

    rawBufferSize = width * height * depth;
    rawBuffer = new unsigned char[rawBufferSize];
  }

  jpegBuffer = const_cast<unsigned char*>(&(image->uint8_data.data[0]));
  jpegBufferSize = image->uint8_data.data.size();

  // Decompress the jpeg image
  jpeg_decompress( rawBuffer, rawBufferSize, jpegBuffer, jpegBufferSize );

  decompressedImageMessage.header.frame_id = frameCount++;

  // Copy raw image data into the message
  fillImage(decompressedImageMessage, "decompressed_image",
            height, width, depth, "mono", "uint8", rawBuffer);

  // Publish the decompressed image
  rawPub.publish(decompressedImageMessage);

  // Debug: Will save the raw buffer to a pgm file
  char filename[256];
  sprintf(filename, "test%d.pgm", frameCount);

  // Output test file
  FILE *file = fopen(filename, "w");
  fprintf(file, "P5\n# My File\n%d %d\n255\n",width, height);
  fwrite(rawBuffer, rawBufferSize, 1, file);
  fclose(file);
}

////////////////////////////////////////////////////////////////////////////////
// Main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "decoder");
  ros::Subscriber rawSub;
  ros::NodeHandle nodeHandle;

  // Subscribe to the encoded jpeg image topic
  rawSub = nodeHandle.subscribe("jpeg_image", 1, imageCB);

  // Create the raw jpeg publisher to output raw images
  rawPub = nodeHandle.advertise<image_msgs::Image>(pubTopicName,1);

  // Run it baby
  ros::spin();

  // Cleanup
  if (rawBuffer)
    delete [] rawBuffer;
  rawBuffer = NULL;

  nodeHandle.shutdown();

  return 0;
}
