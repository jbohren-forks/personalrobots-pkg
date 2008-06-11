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


/**

@mainpage

@htmlinclude manifest.html

@b tilting_laser is a driver for generating 3D Point clouds from a a
laser range-finder mounted on a tilting stage

<hr>

@section usage Usage

The tilting_laser has runtime dependencies on hokuyourg_player and
etherdrive nodes to generate laser scans, and control the tilting
stage respectively.  These must be started in addition to
tilting_laser node.  A bashscript launch_tl is used to start these
nodes conveniently.

@verbatim
$ ./launch_tl [optional path to ros master]
@endverbatim

@par Example

@verbatim
$ ./launch_tl http://vnc:11311
@endverbatim

Note in the bash script that the topic name for the motor input and
output will often have to be remapped.

@verbatim
$ tilting_laser mot:=/mot0 mot_cmd:=/mot0_cmd&
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../std_msgs/html/classstd__msgs_1_1LaserScan.html">LaserScan</a> : laser scan data
- @b "mot"/<a href="../../std_msgs/html/classstd__msgs_1_1Actuator.html">Actuator</a> : encoder data from the tilting stage

Publishes to (name / type):
- @b "cloud"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloudFloat32.html">PointCloudFloat32</a> : Incremental cloud data.  Each scan from the laser is converted into a PointCloud.
- @b "shutter"/<a href="../../std_msgs/html/classstd__msgs_1_1Empty.html">Empty</a> : An empty message is sent to indicate a full sweep has occured
- @b "full_cloud"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloudFloat32.html">PointCloudFloat32</a> : A full point cloud containing all of the points between the last two shutters
- @b "laser_image"/<a href="../../std_msgs/html/classstd__msgs_1_1LaserImage.html">LaserImage</a> : A representation of the full point cloud as a pair of pseudo-images
- @b "image"/<a href="../../std_msgs/html/classstd__msgs_1_1Image.html">Image</a> : The intensity image from the LaserImage
- @b "mot_cmd"/<a href="../../std_msgs/html/classstd__msgs_1_1Actuator.html">Actuator</a> : The commanded position of the tilting stage

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "tilting_laser/num_scans" : @b [int] the number of scans to take between the min and max angles (Default: 400)
- @b "tilting_laser/min_ang" : @b [double] the minimum angle of the scan in degrees (Default: -65.0)
- @b "tilting_laser/max_ang" : @b [double] the maximum angle of the scan in degrees (Default: 35.0)

 **/


#include "ros/node.h"
#include "std_msgs/Empty.h"
#include "std_msgs/LaserScan.h"
#include "std_msgs/PointCloudFloat32.h"
#include "std_msgs/LaserImage.h"
#include "std_msgs/Actuator.h"
#include "libTF/libTF.h"
#include "math.h"

using namespace std_msgs;

class Tilting_Laser : public ros::node
{
public:

  PointCloudFloat32 cloud;
  Empty shutter;
  Actuator cmd;

  LaserScan scans;
  Actuator encoder;
  LaserImage image;

  PointCloudFloat32 full_cloud;
  int full_cloud_cnt;

  int num_scans;

  double min_ang;
  double max_ang;

  libTF::TransformReference TR;

  bool sizes_ready;
  int img_ind;
  int img_dir;

  double last_ang;
  double rate_err_down;
  double rate_err_up;
  double accum_angle;

  ros::Time last_motor_time;

  bool scan_received;

  Tilting_Laser() : ros::node("tilting_laser"), full_cloud_cnt(0), sizes_ready(false), img_ind(-1), img_dir(1),last_ang(1000000), rate_err_down(0.0), rate_err_up(0.0), accum_angle(0.0), scan_received(false)
  {
    
    advertise<PointCloudFloat32>("cloud");
    advertise<Empty>("shutter");
    advertise<PointCloudFloat32>("full_cloud");
    advertise<LaserImage>("laser_image");
    advertise<Image>("image");
    advertise<Actuator>("mot_cmd");

    subscribe("scan", scans, &Tilting_Laser::scans_callback,40);
    subscribe("mot",  encoder, &Tilting_Laser::encoder_callback,40);

    param("tilting_laser/num_scans", num_scans, 400);
    param("tilting_laser/min_ang", min_ang, -65.0);
    min_ang *= M_PI/180.0;
    param("tilting_laser/max_ang", max_ang, 35.0);
    max_ang *= M_PI/180.0;

    last_motor_time = ros::Time::now();
  }

  virtual ~Tilting_Laser()
  { 

  }

  void scans_callback() {
    scan_received = true;

    //TODO: Add check for hokuyo changing width!
    if (!sizes_ready)
    {
      image.range_img.height = num_scans;
      image.range_img.width = scans.get_ranges_size();
      image.range_img.set_data_size(2*image.range_img.height*image.range_img.width);
      image.range_img.colorspace = "mono16";
      image.range_img.compression = "raw";

      image.intensity_img.height = num_scans;
      image.intensity_img.width = scans.get_ranges_size();
      image.intensity_img.set_data_size(2*image.intensity_img.height*image.intensity_img.width);
      image.intensity_img.colorspace = "mono16";
      image.intensity_img.compression = "raw";

      image.set_vert_angles_size(num_scans);
      image.set_horiz_angles_size(scans.get_ranges_size());

      full_cloud.set_pts_size(scans.get_ranges_size()*num_scans);
      full_cloud.set_chan_size(1);
      full_cloud.chan[0].name = "intensities";
      full_cloud.chan[0].set_vals_size(scans.get_ranges_size()*num_scans);

      sizes_ready = true;
    }

    cloud.set_pts_size(scans.get_ranges_size());
    cloud.set_chan_size(1);
    cloud.chan[0].name = "intensities";
    cloud.chan[0].set_vals_size(scans.get_ranges_size());


    libTF::TFVector u;
    u.x = 1;
    u.y = 0;
    u.z = 0;
    u.time = scans.header.stamp.to_ull();
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
    
    int ind;
    
    if (img_dir == 1)
      ind = img_ind;
    else
      ind = (num_scans - img_ind - 1);

    if (img_ind >= 0)
    {

      NEWMAT::Matrix point(4,1);
      NEWMAT::Matrix rot_point(4,1);

      int cnt = 0;

      for (uint32_t i = 0; i < scans.get_ranges_size(); i++) {
        bool valid = false;
        if (scans.ranges[i] < scans.range_max && scans.ranges[i] > scans.range_min)
          valid = true;
        
        if (valid) {
          point(1,1) = cos(scans.angle_min + i*scans.angle_increment) * scans.ranges[i];
          point(2,1) = sin(scans.angle_min + i*scans.angle_increment) * scans.ranges[i];
          point(3,1) = 0.0;
          point(4,1) = 1.0;
        } else {
          point(1,1) = 0.0;
          point(2,1) = 0.0;
          point(3,1) = 0.0;
          point(4,1) = 1.0;
        }
      
        long long inc = scans.time_increment * i * 1000000000;
        unsigned long long t = scans.header.stamp.to_ull() + inc;
        rot_point = TR.getMatrix(1,3,t) * point;

        if (valid)
        {
          cloud.pts[i].x = rot_point(1,1);
          cloud.pts[i].y = rot_point(2,1);
          cloud.pts[i].z = rot_point(3,1);
          cloud.chan[0].vals[i] = scans.intensities[i];
          cnt++;

          full_cloud.pts[img_ind * scans.get_ranges_size() + i].x = rot_point(1,1);
          full_cloud.pts[img_ind * scans.get_ranges_size() + i].y = rot_point(2,1);
          full_cloud.pts[img_ind * scans.get_ranges_size() + i].z = rot_point(3,1);
          full_cloud.chan[0].vals[img_ind * scans.get_ranges_size() + i] = scans.intensities[i];
          full_cloud_cnt++;
        }
        
	*(unsigned short*)(&(image.intensity_img.data[ind * 2 * image.intensity_img.width + image.intensity_img.width * 2 - 2 - 2*i])) = scans.intensities[i];
	*(unsigned short*)(&(image.range_img.data[ind * 2 * image.range_img.width + image.range_img.width * 2 - 2 - 2*i])) = scans.ranges[i] * 1000.0;
	image.horiz_angles[i] = scans.angle_min + i*scans.angle_increment;
      }

      image.vert_angles[ind] = ang;
      cloud.set_pts_size(cnt);
      cloud.chan[0].set_vals_size(cnt);
      publish("cloud", cloud);
      img_ind++;

      if (img_ind == num_scans) {
        printf("Scan completed.\n");
        
        if (img_dir == 1)
          rate_err_down -= (min_ang - ang) / (scans.scan_time * num_scans);
        else
          rate_err_up += (max_ang - ang) / (scans.scan_time * num_scans);


        publish("laser_image", image);
        publish("image", image.intensity_img);
        full_cloud.set_pts_size(full_cloud_cnt);
        full_cloud.chan[0].set_vals_size(full_cloud_cnt);
        publish("full_cloud", full_cloud);
        full_cloud.set_pts_size(scans.get_ranges_size()*num_scans);
        full_cloud.chan[0].set_vals_size(scans.get_ranges_size()*num_scans);
        publish("shutter",shutter);
        full_cloud_cnt = 0;

        img_ind = -1;
        sizes_ready = false;
      }
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
    {
      last_motor_time = ros::Time::now();
      return;
    }

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

    double index = fabs(fmod( (accum_angle) / (2.0 * (ext_max_ang - ext_min_ang + .1)), 1) * 2.0 - 1.0);

    cmd.val = index * (ext_max_ang - ext_min_ang + .1) + ext_min_ang -.05;

    if (cmd.val > ext_max_ang)
      cmd.val = ext_max_ang;

    if (cmd.val < ext_min_ang)
      cmd.val = ext_min_ang;

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

