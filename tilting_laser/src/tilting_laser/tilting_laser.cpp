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

#include "ros/ros_slave.h"
#include "common_flows/FlowEmpty.h"
#include "common_flows/FlowLaserScan.h"
#include "common_flows/FlowPointCloudFloat32.h"
#include "unstable_flows/FlowActuator.h"
#include "libTF/libTF.h"
#include "math.h"

class Tilting_Laser : public ROS_Slave
{
public:
  FlowLaserScan *scans;
  FlowActuator *encoder;
  FlowActuator *cmd;
  FlowPointCloudFloat32 *cloud;
  FlowEmpty *shutter;

  struct timeval starttime;

  double next_shutter;
  double period;

  double min_ang;
  double max_ang;

  int scancount;

  TransformReference TR;

  Tilting_Laser() : ROS_Slave(), next_shutter(0.0), scancount(0)
  {
    register_source(cloud = new FlowPointCloudFloat32("cloud"));
    register_source(shutter = new FlowEmpty("shutter"));
    register_sink(scans = new FlowLaserScan("scans"), ROS_CALLBACK(Tilting_Laser, scans_callback));
    register_sink(encoder = new FlowActuator("encoder"), ROS_CALLBACK(Tilting_Laser, encoder_callback));
    register_source(cmd = new FlowActuator("cmd"));
    register_with_master();
    printf("package path is [%s]\n", get_my_package_path().c_str());

    if (!get_double_param(".period", &period))
      period = 5.0;

    if (!get_double_param(".min_ang", &min_ang))
      min_ang = -1.3;

    if (!get_double_param(".max_ang", &max_ang))
      max_ang = 0.6;

    unsigned long long time = Quaternion3D::Qgettime();
    TR.setWithEulers(3, 2,
		     0, 0, 0,
		     0, 0, 0,
		     time);

    TR.setWithEulers(2, 1,
		     0, 0, 0,
		     0, 0, 0,
		     time);

    gettimeofday(&starttime,NULL);    
  }

  virtual ~Tilting_Laser()
  { 

  }

  void scans_callback() {
    encoder->lock_atom();
    cloud->set_x_size(scans->get_ranges_size());
    cloud->set_y_size(scans->get_ranges_size());
    cloud->set_z_size(scans->get_ranges_size());

    /*
    for (int i = 0; i < scans->get_ranges_size(); i++) {
      cloud->x[i] = cos(encoder->val)*cos(scans->angle_min + i*scans->angle_increment) * scans->ranges[i];
      cloud->y[i] = sin(scans->angle_min + i*scans->angle_increment) * scans->ranges[i];
      cloud->z[i] = sin(encoder->val)*cos(scans->angle_min + i*scans->angle_increment) * scans->ranges[i];
    }
    */

    NEWMAT::Matrix points(4,scans->get_ranges_size());
    for (int i = 0; i < scans->get_ranges_size(); i++) {
      if (scans->ranges[i] < 20.0) {
	points(1,i+1) = cos(scans->angle_min + i*scans->angle_increment) * scans->ranges[i];
	points(2,i+1) = sin(scans->angle_min + i*scans->angle_increment) * scans->ranges[i];
	points(3,i+1) = 0.0;
	points(4,i+1) = 1.0;
      } else {
	points(1,i+1) = 0.0;
	points(2,i+1) = 0.0;
	points(3,i+1) = 0.0;
	points(4,i+1) = 1.0;
      }
    }
    
    unsigned long long time = (unsigned long long)scans->get_stamp_secs()*1000000 + (unsigned long long)scans->get_stamp_nsecs()/1000 - 20000;
    NEWMAT::Matrix rot_points = TR.getMatrix(1,3,time) * points;
    
    for (int i = 0; i < scans->get_ranges_size(); i++) {
      cloud->x[i] = rot_points(1,i+1);
      cloud->y[i] = rot_points(2,i+1);
      cloud->z[i] = rot_points(3,i+1);
    }

    scancount++;

    encoder->unlock_atom();
    cloud->publish();
  }

  void encoder_callback() {
    unsigned long long time = (unsigned long long)encoder->get_stamp_secs()*1000000 + (unsigned long long)encoder->get_stamp_nsecs()/1000;
    TR.setWithEulers(3, 2,
		     .02, 0, .02,
		     0.0, 0.0, 0.0,
		     time);
    TR.setWithEulers(2, 1,
		     0, 0, 0,
		     0, -encoder->val, 0,
		     time);

    motor_control(); // Control on encoder reads sounds reasonable
    //    printf("I got some encoder values: %g!\n", encoder->val);
  }

  double timeval_diff (struct timeval x, 
		       struct timeval y)  {
    double dsec = x.tv_sec - y.tv_sec;
    double dusec = x.tv_usec - y.tv_usec;
    return dsec + dusec/1000000.0;
  }

  void motor_control() {

    struct timeval nowtime;
    gettimeofday(&nowtime,NULL);    
    
    double elapsed_cycles = timeval_diff(nowtime, starttime) / (period);
    double index = fabs(fmod(elapsed_cycles, 1) * 2.0 - 1.0);

    cmd->val = index * (max_ang - min_ang) + min_ang;
    cmd->rel = false;
    cmd->valid = true;

    if (elapsed_cycles > next_shutter) {
      next_shutter += 0.5;
      shutter->publish();
    }
    cmd->publish();
  }

};

int main(int argc, char **argv)
{
  Tilting_Laser tl;
  while (tl.happy()) {
    usleep(10000);
  }
  return 0;
}

