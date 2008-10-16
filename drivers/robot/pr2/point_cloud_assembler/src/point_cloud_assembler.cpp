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

@b point_cloud_assembler accumulates scan lines from a laser range finder mounted on a tilting stage and generates 3D point clouds.

<hr>

@section usage Usage

To use this node, the tilting stage and laser have to be started. A convenient method for starting them is on the way, perhaps as a script.

@par Example

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../std_msgs/html/classstd__msgs_1_1LaserScan.html">LaserScan</a> : laser scan data

Publishes to (name / type):
- @b "cloud"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloudFloat32.html">PointCloudFloat32</a> : Incremental cloud data.  Each scan from the laser is converted into a PointCloud.
- @b "full_cloud"/<a href="../../std_msgs/html/classstd__msgs_1_1PointCloudFloat32.html">PointCloudFloat32</a> : A full point cloud containing all of the points between the last two shutters

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "tilting_laser/num_scans" : @b [int] the number of scans to take between the min and max angles (Default: 400)

 **/


#include "ros/node.h"
#include "rosTF/rosTF.h"
#include "std_msgs/LaserScan.h"
#include "std_msgs/PointCloudFloat32.h"
#include "pr2_mechanism_controllers/LaserScannerSignal.h"

//Laser projection
#include "laser_scan_utils/laser_scan.h"

#include "math.h"

using namespace std_msgs;

class PointCloudAssembler : public ros::node
{
public:

  rosTFClient tf_;
  laser_scan::LaserProjection projector_;

  PointCloudFloat32 cur_scan_cloud_;

  LaserScan scans_;
  pr2_mechanism_controllers::LaserScannerSignal scanner_signal_;

  PointCloudFloat32 full_cloud_;
  int full_cloud_cnt_;

  int num_scans_;

  /* Reinstate to let user control the angles.
  double min_ang;
  double max_ang; */

  //ros::Time last_motor_time;
  int count_; // This just counts the number of scans per second for output. It's unnecessary.
  ros::Time next_time_;

  bool start_scan_;
  bool stop_scan_;
  bool scanning_;

  PointCloudAssembler() : ros::node("point_cloud_assembler"), tf_(*this), full_cloud_cnt_(0), count_(0), start_scan_(false), stop_scan_(false), scanning_(false)
  {
    
    advertise<PointCloudFloat32>("cloud", 1);
    advertise<PointCloudFloat32>("full_cloud", 1);

    subscribe("scan", scans_, &PointCloudAssembler::scans_callback, 40);
    subscribe("laser_scanner_signal", scanner_signal_, &PointCloudAssembler::scanner_signal_callback, 40);

    param("point_cloud_assembler/num_scans", num_scans_, 400);

    //last_motor_time = ros::Time::now();
    next_time_ = ros::Time::now();

  }

  virtual ~PointCloudAssembler()
  { 
    unadvertise("cloud") ;
    unadvertise("full_cloud") ;
    //unsubscribe("scan") ;
    //unsubscribe("laser_scanner_signal") ;
  }

  void scans_callback()
  {
    //printf("start %d stop %d scan %d\n", start_scan_, stop_scan_, scanning_);

    count_++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time_)
    {
      //std::cout << count_ << " scans/sec at " << now_time << std::endl;
      count_ = 0;
      next_time_ = next_time_ + ros::Duration(1,0);
    }

    // If told to stop scanning and you have some points in your full cloud, publish the cloud.
    if (stop_scan_)
    {
      if (scanning_)
      {
	// What happens if the point_cloud_cnt is 0? Should I still publish?
	full_cloud_.set_pts_size(full_cloud_cnt_);
	full_cloud_.chan[0].set_vals_size(full_cloud_cnt_);
	publish("full_cloud",full_cloud_);
	printf("PointCloudAssembler::Publishing scan of size %d.\n", full_cloud_cnt_);
      }
      full_cloud_cnt_ = 0;
      scanning_ = false;
      stop_scan_ = false;
    }


    // If you need to start scanning, allocate the cloud size.
    if (start_scan_)
    {
      full_cloud_.set_pts_size(scans_.get_ranges_size()*num_scans_);
      full_cloud_.set_chan_size(1);
      full_cloud_.chan[0].name = "intensities";
      full_cloud_.chan[0].set_vals_size(scans_.get_ranges_size()*num_scans_);
      full_cloud_cnt_ = 0;
      scanning_ = true;
      start_scan_=false;
    }

    // Mid-scan
    if (scanning_) 
    {
      // CRP: Clouds are now stl vectors.
      // This will be the cloud storing points from the current single scan.
      cur_scan_cloud_.set_pts_size(scans_.get_ranges_size());
      cur_scan_cloud_.set_chan_size(1);
      cur_scan_cloud_.chan[0].name = "intensities";
      cur_scan_cloud_.chan[0].set_vals_size(scans_.get_ranges_size());

      //printf("size(cur_scan_cloud_) =  %d\n", cur_scan_cloud_.pts.size()) ;

      
      // Define cloud to store the current single scan after being projected into the laser frame by the projector
      std_msgs::PointCloudFloat32 projector_cloud ;
      // CRP: Will be a transform in TF that takes scan time into account. High-precision, recalc for every point, from scan to point cloud. More expensive. Wait for new library to do this.
      projector_.projectLaser(scans_, projector_cloud); // CRP: This takes care of converting the scan msg to the right format, so you don't really need to know what the format is. 

      //printf("size(projector_cloud_) = %d\n", projector_cloud.pts.size()) ;	

      cur_scan_cloud_ = tf_.transformPointCloud("base", projector_cloud); // CRP: We'll make people do this themselves for one scan line, we'll just tansform and publish aggregate clouds.

      //printf("size(cur_scan_cloud_ %d\n", cur_scan_cloud_.pts.size());

      publish("cloud", cur_scan_cloud_);

      full_cloud_.header = cur_scan_cloud_.header; //find a better place to do this/way to do this
      full_cloud_.header.stamp = ros::Time::now(); //HACK

      // CRP: Might make this more efficient by using stl vector functions instead.
      // CRP: Try this with full_cloud.pts.push_back(cloud.pts[i]);, and use an iterator to go through the elements. Actually, that's inefficient, you should index them. But perhaps I can just copy them wholesale?
      //Populate full_cloud from the cloud
      for(unsigned int i = 0; i < cur_scan_cloud_.get_pts_size(); i ++)
      {
        full_cloud_.pts[full_cloud_cnt_].x = cur_scan_cloud_.pts[i].x;  
        full_cloud_.pts[full_cloud_cnt_].y = cur_scan_cloud_.pts[i].y;  
        full_cloud_.pts[full_cloud_cnt_].z = cur_scan_cloud_.pts[i].z;  
        full_cloud_.chan[0].vals[full_cloud_cnt_] = cur_scan_cloud_.chan[0].vals[i];
        full_cloud_cnt_++;
      }
    }
  }

  // Msg callback for top/bottom msg.
  void scanner_signal_callback() 
  {
    // If the scanner signal is 0, the scanner is at the bottom of it's sine wave, and at the top for 1. 
    // I'm going to listen to both and produce one scan on the way down and one on the way up, so I don't actually care what the signal's value is.
    stop_scan_ = true;
    start_scan_ = true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  PointCloudAssembler pc_assembler;
  pc_assembler.spin();
  ros::fini();
  return 0;
}

