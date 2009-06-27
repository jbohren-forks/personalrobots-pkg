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

#include <ros/ros.h>
#include <image_msgs/Image.h>
#include <diagnostic_updater/publisher.h>

#define BASESIZE 1000000
int oldwidth = 0;
int oldheight = 0;
uint8_t base[BASESIZE];

#define NUMSKIP 400

inline void insert(int *max, int val)
{
  if (val < max[0])
    return;

  int i = 1;
  while (val > max[i] && i < NUMSKIP)
  {
    max[i - 1] = max[i];
    i++;
  }

  max[i - 1] = val;
}

void callback(const image_msgs::ImageConstPtr& msg)
{
  int width = msg->uint8_data.layout.dim[1].size;
  int height = msg->uint8_data.layout.dim[0].size;

  int max[2][NUMSKIP];
  int maxv[NUMSKIP];
  bzero(max, sizeof(max));
  bzero(maxv, sizeof(maxv));

  if (oldwidth != width || oldheight != height)
  {
    oldwidth = width;
    oldheight = height;
    for (int i = 0; i < width * height; i++)
      base[i] = 0;
    fprintf(stderr, "Resetting base. Show a black screen!\n");
  }

  /*assert(width * height < BASESIZE);

  for (int i = 0; i < width * height; i++)
    if (base[i] > msg->uint8_data.data[i])
      base[i] = msg->uint8_data.data[i];
  */
  for (int x = 1; x < width - 1; x++)
  {
    for (int y = 0; y < height - 1; y++)
    {                
      int a = (x + y) & 1;

      //if (a == 0)
      //  continue;
        
      int i = y * width + x;
      int i1 = i + width - 1;
      int i2 = i1 + 2;
      int p = msg->uint8_data.data[i] - base[i];
      int p1 = msg->uint8_data.data[i1] - base[i1];
      int p2 = msg->uint8_data.data[i2] - base[i2];

      int d1 = abs(p - p1);
      int d2 = abs(p - p2);

      insert(max[a], d1);
      insert(max[a], d2);
//        insert(maxv, p);
    }
  }

//  for (int i = 0; i < NUMSKIP; i++)
//    printf("%i ", max[i]);
//  printf(": %i %i %i %i\n", width, height, *max, *maxv);
  printf("%i %i\n", max[0][0], max[1][0]);
  fflush(stdout);
}

void callback2(const image_msgs::ImageConstPtr& msg)
{
  int width = msg->uint8_data.layout.dim[1].size;
  int height = msg->uint8_data.layout.dim[0].size;

  if (oldwidth != width || oldheight != height)
  {
    oldwidth = width;
    oldheight = height;
    for (int i = 0; i < width * height; i++)
      base[i] = 0;
    fprintf(stderr, "Resetting base. Show a black screen!\n");
  }

  /*assert(width * height < BASESIZE);

  for (int i = 0; i < width * height; i++)
    if (base[i] > msg->uint8_data.data[i])
      base[i] = msg->uint8_data.data[i];
  */
  printf("set yrange [0:255]\n");
  printf("set terminal x11\n");
  printf("plot \"-\" using 0:1 with lines\n");
  for (int y = 0; y < height - 1; y++)
  {
    int max = 0;

    for (int x = 1; x < width - 1; x++)
    {                
      int a = (x + y) & 1;

      if (a == 0)
        continue;
        
      int i = y * width + x;
      int i1 = i + width - 1;
      int i2 = i1 + 2;
      int p = msg->uint8_data.data[i] - base[i];
      int p1 = msg->uint8_data.data[i1] - base[i1];
      int p2 = msg->uint8_data.data[i2] - base[i2];

      int d1 = abs(p - p1);
      int d2 = abs(p - p2);

      if (d1 > max) max = d1;
      if (d2 > max) max = d2;
    }
    printf("%i\n", max);
  }
  printf("e\n\n");
  fflush(stdout);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forearm_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("image", 1, callback2);
  ros::spin();
  return 0;
}
