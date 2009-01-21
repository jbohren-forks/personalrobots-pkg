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
#include "std_msgs/LaserScan.h"
#include "std_msgs/RobotBase2DOdom.h"
#include <vector>
#include <string>
#include "rosrecord/Player.h"
using std::vector;
using std::string;

FILE *clog = NULL;
FILE *test_log = NULL;
double prev_x = 0, prev_y = 0, prev_th = 0, dumb_rv = 0, dumb_tv = 0, prev_time;

void odom_callback(string name, std_msgs::RobotBase2DOdom* odom, ros::Time t, ros::Time t_no_use, void* n)
{
  double rel_time = t.toSec();

  static bool vel_init = false;
  static double yaw_offset = 0;

  if (!vel_init)
  {
    vel_init = true;
    dumb_rv = dumb_tv = 0;
    prev_time = rel_time;
    test_log = fopen("test.txt", "w");
  }
  else
  {
    double next_th = odom->pos.th + yaw_offset;
    if (fabs(next_th - prev_th) > M_PI)
    {
      if (next_th > prev_th)
        yaw_offset -= 2 * M_PI;
      else
        yaw_offset += 2 * M_PI;
    }
    odom->pos.th += yaw_offset;
    double dt = rel_time - prev_time;
    double dx = odom->pos.x - prev_x;
    double dy = odom->pos.y - prev_y;
    dumb_rv = (odom->pos.th - prev_th) / dt;
    dumb_tv = sqrt(dx*dx + dy*dy) / dt;
    fprintf(test_log, "%f %f %f %f %f %f\n", 
            odom->pos.x, odom->pos.y, odom->pos.th, dumb_tv, dumb_rv, dt);
  }
  prev_x  = odom->pos.x;
  prev_y  = odom->pos.y;
  prev_th = odom->pos.th;
  prev_time = rel_time;
  fprintf(clog, "ODOM %f %f %f %f %f 0 %f logsetta %f\n", 
          odom->pos.x, odom->pos.y, odom->pos.th,
          dumb_tv, dumb_rv, rel_time, rel_time);
}

void scan_callback(string name, std_msgs::LaserScan* scan, ros::Time t, ros::Time t_no_use, void* n)
{

  double rel_time = t.toSec();

  const double fov = fabs(scan->angle_max - scan->angle_min);
  // only make an exception for the SICK LMS2xx running in centimeter mode
  const double acc = (scan->range_max >= 81 ? 0.05 : 0.005); 
  fprintf(clog, "ROBOTLASER1 0 %f %f %f %f %f 0 %d ",
          scan->angle_min, fov, fov / scan->angle_increment,
          scan->range_max, acc, scan->get_ranges_size());
  for (uint32_t i = 0; i < scan->get_ranges_size(); i++)
    fprintf(clog, "%.3f ", scan->ranges[scan->get_ranges_size() - i - 1]);
  double laser_x = prev_x + 0.30 * cos(prev_th); // in the robot frame
  double laser_y = prev_y + 0.30 * sin(prev_th);
  double laser_th = prev_th;
  double laser_rv = dumb_rv; // not really
  double laser_tv = dumb_tv; // not really
  fprintf(clog, " 0 %f %f %f %f %f %f %f %f 0.3 0.3 1000000 %f logsetta %f\n",
          laser_x, laser_y, laser_th,
          prev_x, prev_y, prev_th,
          laser_tv, laser_rv, rel_time,
          rel_time);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: genlog_carmen FILE1 [FILE2, ...]\n"
           "  The logs must have been vacuumed up with megamaid's vacuum or central_vac and contain /odom and /scan messages.\n");
    return 1;
  }

  ros::record::MultiPlayer player;

  vector<string> files;

  for (int i = 1; i < argc; i++)
    files.push_back(argv[i]);

  player.open(files, ros::Time());

  player.addHandler<std_msgs::RobotBase2DOdom>(string("*"), &odom_callback, NULL);
  player.addHandler<std_msgs::LaserScan>(string("*"), &scan_callback, NULL);

  clog = fopen("carmen.txt", "w");

  while(player.nextMsg())  {}

  fclose(clog);

  return 0;
}
