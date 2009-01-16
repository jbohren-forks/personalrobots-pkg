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

#include "ros/node.h"
#include "std_msgs/LaserScan.h"
#include "std_msgs/RobotBase2DOdom.h"

class CarmenLogger : public ros::Node
{
public:
  std_msgs::LaserScan laserMsg;
  std_msgs::RobotBase2DOdom odomMsg;
  double robot_x, robot_y, robot_th, robot_tv, robot_rv;
  double start_time;
  FILE *f;
  ros::thread::mutex log_mutex;
  CarmenLogger() : ros::Node("carmenLogger"), 
                   robot_x(0), robot_y(0), robot_th(0), start_time(0)
  {
    f = fopen("carmen.log", "w");
    if (!f)
      assert(0);
    subscribe("scan", laserMsg, &CarmenLogger::scanCB);
    subscribe("odom", odomMsg,  &CarmenLogger::odomCB);
  }
  virtual ~CarmenLogger()
  {
    fclose(f);
  }
  void scanCB()
  {
    const double fov = fabs(laserMsg.angle_max - laserMsg.angle_min);
    // only make an exception for the SICK LMS2xx running in centimeter mode
    const double acc = (laserMsg.range_max >= 81 ? 0.01 : 0.001); 
    log_mutex.lock();
    fprintf(f, "ROBOTLASER1 0 %f %f %f %f %f 0 %d ",
            laserMsg.angle_min, fov, fov / laserMsg.angle_increment,
            laserMsg.range_max, acc, laserMsg.get_ranges_size());
    for (int i = 0; i < laserMsg.get_ranges_size(); i++)
      fprintf(f, "%f ", laserMsg.ranges[i]);
    const double laser_x = 0.30; // in the robot frame
    const double laser_y = 0;
    const double laser_th = 0;
    const double laser_rv = robot_rv;
    const double laser_tv = robot_tv;
    if (start_time == 0)
      start_time = laserMsg.header.stamp.to_double();
    fprintf(f, " %f %f %f %f %f %f %f %f 0.3 0.3 1000000 %f rosetta %f\n",
            laser_x, laser_y, laser_th,
            robot_x, robot_y, robot_th,
            laser_tv, laser_rv, laserMsg.header.stamp.to_double(), 
            laserMsg.header.stamp.to_double() - start_time);
    log_mutex.unlock();
  }
  void odomCB()
  {
    if (start_time == 0)
      start_time = odomMsg.header.stamp.to_double();
    log_mutex.lock();
    fprintf(f, "ODOM %f %f %f %f %f 0 %f rosetta %f\n",
            robot_x, robot_y, robot_th, robot_rv, robot_tv,
            odomMsg.header.stamp.to_double(),
            odomMsg.header.stamp.to_double() - start_time);
    log_mutex.unlock();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  CarmenLogger l;
  l.spin();
  ros::fini();
  return 0;
}
