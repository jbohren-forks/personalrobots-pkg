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

#include "ros/node.h"
#include "std_msgs/MsgEmpty.h"
#include "std_msgs/MsgLaserScan.h"
#include "std_msgs/MsgPointCloudFloat32.h"
#include "std_msgs/MsgImage.h"
#include "std_msgs/MsgActuator.h"
#include "libTF/libTF.h"
#include "math.h"

class Tilting_Laser : public ros::node
{
public:

  MsgPointCloudFloat32 cloud;
  MsgEmpty shutter;
  MsgActuator cmd;

  MsgLaserScan scans;
  MsgActuator encoder;
  MsgImage image;

  int num_scans;

  double min_ang;
  double max_ang;

  libTF::TransformReference TR;

  bool img_ready;
  int img_ind;
  int img_dir;

  double last_ang;
  double rate_err_up;
  double rate_err_down;
  double accum_angle;

  ros::Time last_motor_time;

  bool scan_received;

  Tilting_Laser() : ros::node("tilting_laser"), img_ready(false), img_ind(-1), last_ang(1000000), img_dir(1), rate_err_down(0.0), rate_err_up(0.0), accum_angle(0), scan_received(false)
  {
    
    advertise<MsgPointCloudFloat32>("cloud");
    advertise<MsgEmpty>("shutter");
    advertise<MsgActuator>("mot_cmd");
    advertise<MsgImage>("image");

    subscribe("scan", scans, &Tilting_Laser::scans_callback,10);
    subscribe("mot",  encoder, &Tilting_Laser::encoder_callback,10);

    param("tilting_laser/num_scans", num_scans, 400);
    param("tilting_laser/min_ang", min_ang, -1.3);
    param("tilting_laser/max_ang", max_ang, 0.6);

    unsigned long long time = clock.ulltime();

    TR.setWithEulers(3, 2,
		     0, 0, 0,
		     0, 0, 0,
		     time);

    TR.setWithEulers(2, 1,
		     0, 0, 0,
		     0, 0, 0,
		     time);

  }

  virtual ~Tilting_Laser()
  { 

  }

  void scans_callback() {

    scan_received = true;

    //TODO: Add check for hokuyo changing width!
    if (!img_ready) {
      image.height = num_scans;
      image.width = scans.get_ranges_size();
      image.set_data_size(2*image.height*image.width);
      
      image.colorspace = "mono16";
      image.compression = "raw";

      img_ready = true;
    }

    cloud.set_pts_size(scans.get_ranges_size());

    cloud.set_chan_size(1);
    cloud.chan[0].name = "intensities";
    cloud.chan[0].set_vals_size(scans.get_ranges_size());

    NEWMAT::Matrix points(4,scans.get_ranges_size());
    for (uint32_t i = 0; i < scans.get_ranges_size(); i++) {
      if (scans.ranges[i] < 20.0) {
	points(1,i+1) = cos(scans.angle_min + i*scans.angle_increment) * scans.ranges[i];
	points(2,i+1) = sin(scans.angle_min + i*scans.angle_increment) * scans.ranges[i];
	points(3,i+1) = 0.0;
	points(4,i+1) = 1.0;
      } else {
	points(1,i+1) = 0.0;
	points(2,i+1) = 0.0;
	points(3,i+1) = 0.0;
	points(4,i+1) = 1.0;
      }
    }

    unsigned long long t = scans.header.stamp.to_ull() - 30000000;
    
    NEWMAT::Matrix rot_points = TR.getMatrix(1,3,t) * points;

    libTF::TFVector u;
    u.x = 1;
    u.y = 0;
    u.z = 0;
    u.time = t;
    u.frame = 3;

    libTF::TFVector v = TR.transformVector(1,u);
    

    double ang = atan2(v.z,v.x);

    if (ang < max_ang && last_ang > max_ang)
    {
      img_dir = 1;
      img_ind = 0;
    }

    if (ang > min_ang && last_ang < min_ang)
    {
      img_dir = -1;
      img_ind = 0;
    }

    last_ang = ang;

    for (uint32_t i = 0; i < scans.get_ranges_size(); i++) {
      cloud.pts[i].x = rot_points(1,i+1);
      cloud.pts[i].y = rot_points(2,i+1);
      cloud.pts[i].z = rot_points(3,i+1);
      cloud.chan[0].vals[i] = scans.intensities[i];

      if (img_ind >= 0)
      {
	if (img_dir == 1)
	  *(unsigned short*)(&(image.data[img_ind * 2 * image.width + image.width * 2 - 2 - 2*i])) = scans.intensities[i];
	else
	  *(unsigned short*)(&(image.data[(num_scans - img_ind - 1) * 2 * image.width + image.width * 2 - 2 - 2*i])) = scans.intensities[i];
      }
    }

    if (img_ind >= 0)
      img_ind++;

    publish("cloud", cloud);

    if (img_ind == num_scans) {
      printf("Sending image and shutter\n");

      double ang_err;

      if (img_dir == 1)
      {
	ang_err = min_ang - ang;
	rate_err_down -= ang_err / (scans.scan_time * num_scans);
      }
      else
      {
	ang_err = max_ang - ang;
	rate_err_up += ang_err / (scans.scan_time * num_scans);
      }

      printf("Final scan off by: %g\n", ang_err);

      publish("image", image);
      publish("shutter",shutter);
      img_ind = -1;
    }
  }

  void encoder_callback() {

    TR.setWithEulers(3, 2,
		     .02, 0, .02,
		     0.0, 0.0, 0.0,
		     encoder.header.stamp.to_ull());
    TR.setWithEulers(2, 1,
		     0, 0, 0,
		     0, -encoder.val, 0,
		     encoder.header.stamp.to_ull());

    motor_control(); // Control on encoder reads sounds reasonable
  }


  void motor_control() {

    if (!scan_received)
      return;

    cmd.rel = false;
    cmd.valid = true;    
    
    ros::Time t = ros::Time::now();

    double rate = (max_ang - min_ang) / (scans.scan_time * num_scans);
    if (img_dir == 1)
      rate += rate_err_down;
    else
      rate += rate_err_up;
    
    double ext_max_ang = max_ang + .025;
    double ext_min_ang = min_ang - .025;

    accum_angle += (t - last_motor_time).to_double() * rate;

    double index = fabs(fmod( (accum_angle) / (2.0 * (ext_max_ang - ext_min_ang)), 1) * 2.0 - 1.0);

    cmd.val = index * (ext_max_ang - ext_min_ang) + ext_min_ang;
    publish("mot_cmd",cmd);

    last_motor_time = t;

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Tilting_Laser tl;
  tl.spin();
  ros::fini();
  return 0;
}

