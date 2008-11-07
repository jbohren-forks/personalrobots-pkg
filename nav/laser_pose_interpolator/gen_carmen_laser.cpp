///////////////////////////////////////////////////////////////////////////////
// Translates laser pose logs to carmen. Warning: this program does not care
// about velocities.
//
// Copyright (C) 2008, Morgan Quigley and Tony Pratkanis
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
#include "laser_pose_interpolator/PoseLaserScan.h"
#include <vector>
#include <string>
#include "logging/LogPlayer.h"
using std::vector;
using std::string;

FILE *clog = NULL;
FILE *test_log = NULL;

const float LASER_POSITION = 0.05;

void scan_callback(string name, laser_pose_interpolator::PoseLaserScan* scan, ros::Time t, void* n)
{
  double rel_time = t.to_double();

  //const double fov = fabs(scan->scan.angle_max - scan->scan.angle_min);
  // only make an exception for the SICK LMS2xx running in centimeter mode
  //const double acc = (scan->scan.range_max >= 81 ? 0.05 : 0.005); 
  /*fprintf(clog, "ROBOTLASER1 0 %f %f %f %f %f 0 %d ",
          scan->scan.angle_min, fov, fov / scan->scan.angle_increment,
          scan->scan.range_max, acc, scan->scan.get_ranges_size());*/
  fprintf(clog, "FLASER 181 ");
  double bearing = scan->scan.angle_min;
  unsigned int downsample = 0;
  int count=0;
  for (unsigned int i = 0; i < scan->scan.get_ranges_size(); i++) {
    if (downsample == 0) {
      fprintf(clog, "%.3f ", scan->scan.ranges[i]);
      downsample = 4;
      count++;
    }
    downsample--;
    bearing += scan->scan.angle_increment;
  }
  assert(count == 181);
  
  fprintf(clog, "%f %f %f %f %f %f %f rosdump %f\n", scan->pose.x, scan->pose.y, scan->pose.th, 
	  scan->pose.x + LASER_POSITION * cos(scan->pose.th), scan->pose.y + LASER_POSITION * sin(scan->pose.th), scan->pose.th, rel_time, rel_time);

  fprintf(clog, "ODOM %f %f %f 0 0 0 %f rosdump %f\n", scan->pose.x, scan->pose.y, scan->pose.th, rel_time, rel_time);

  /*fprintf(clog, " 0 %f %f %f %f %f %f %f %f 0.3 0.3 1000000 %f logsetta %f\n",
          scan->pose.x, scan->pose.y, scan->pose.th,
          scan->pose.x, scan->pose.y, scan->pose.th,
          0.0, 0.0, rel_time, rel_time);*/
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: genlog_carmen LOG\n"
           "  Where log is a .bag file from megamaid that contains the pose_scan topic.\n");
    return 1;
  }

  MultiLogPlayer player;

  vector<string> files;

  for (int i = 1; i < argc; i++)
    files.push_back(argv[i]);

  player.open(files, ros::Time(0.0));

  player.addHandler<laser_pose_interpolator::PoseLaserScan>(string("pose_scan"), &scan_callback, NULL);

  clog = fopen("carmen.txt", "w");

  while(player.nextMsg())  {}

  fclose(clog);

  return 0;
}
