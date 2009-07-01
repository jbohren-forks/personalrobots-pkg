///////////////////////////////////////////////////////////////////////////////
// The logsetta package provides log translators for various other robotics
// software frameworks
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include "ros/node.h"
#include "sensor_msgs/LaserScan.h"
#include "robot_msgs/PoseStamped.h"
#include <vector>
#include <string>
#include "rosrecord/Player.h"
using std::vector;
using std::string;

FILE *clog = NULL;
double odom_x = 0, odom_y = 0, odom_th = 0;

void odom_callback(string name, robot_msgs::PoseStamped * odom, ros::Time t, ros::Time t_no_use, void* n)
{
  //printf("odom cb\n");
  odom_th = 2 * asin(odom->pose.orientation.z);
  odom_x = odom->pose.position.x;
  odom_y = odom->pose.position.y;
  /*
  fprintf(clog, "Odometry %f %f %f\n",
          odom->pose.position.x, odom->pose.position.y, th);
  */
}

void scan_callback(string name, sensor_msgs::LaserScan* scan, ros::Time t, ros::Time t_no_use, void* n)
{
  //printf("scan cb\n");
  // if we want to use this with a non-SICK laser, we need this code to
  // be smarter and subsample / upsample
  // also, the STAIR1A laser is upside down, so I'm inverting it here...
  fprintf(clog, "Odometry %f %f %f\n", odom_x, odom_y, odom_th);
  fprintf(clog, "Laser 181 ");
  for (uint32_t i = 0; i < scan->get_ranges_size(); i++)
    fprintf(clog, "%.3f ", scan->ranges[scan->get_ranges_size() - i - 1]);//scan->ranges[scan->get_ranges_size() - i - 1]);
  fprintf(clog, "\n");
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: genlog_dpslam FILE1\n"
           "where FILE1 is a bag file containing at least /odom and /scan messages.\n");
    return 1;
  }
  ros::record::MultiPlayer player;
  vector<string> files;
  for (int i = 1; i < argc; i++)
    files.push_back(argv[i]);
  player.open(files, ros::Time());
  player.addHandler<robot_msgs::PoseStamped>(string("odom"), &odom_callback, NULL);
  player.addHandler<sensor_msgs::LaserScan>(string("scan"), &scan_callback, NULL);
  clog = fopen("dpslam.log", "w");
  while(player.nextMsg()) { }
  fclose(clog);
  return 0;
}
