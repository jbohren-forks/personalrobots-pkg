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
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_SCAN_FOOTPRINT_FILTER_H
#define LASER_SCAN_FOOTPRINT_FILTER_H
/**
\author Tully Foote
@b ScanFootprintFilter takes input scans and corrects for footprint angle assuming a flat target.  
This is useful for ground plane extraction

**/


#include "filters/filter_base.h"
#include "tf/transform_listener.h"
#include "robot_msgs/PointCloud.h"
#include "ros/ros.h"

namespace laser_scan
{

template <typename T>
class PointCloudFootprintFilter : public filters::FilterBase<robot_msgs::PointCloud>
{
public:
  PointCloudFootprintFilter() {}

  bool configure()
  {
    getDoubleParam("inscribed_radius", inscribed_radius_, 0.325);
    return true;
  }

  virtual ~PointCloudFootprintFilter()
  { 

  }

  bool update(const std::vector<robot_msgs::PointCloud>& data_in, std::vector<robot_msgs::PointCloud>& data_out)
  {
    if(&data_in == &data_out){
      ROS_ERROR("This filter does not currently support in place copying");
      return false;
    }
    if (data_in.size() != 1 || data_out.size() != 1)
    {
      ROS_ERROR("PointCloudFootprintFilter is not vectorized");
      return false;
    }
    
    const robot_msgs::PointCloud& input_scan = data_in[0];
    robot_msgs::PointCloud& filtered_scan = data_out[0];

    robot_msgs::PointCloud laser_cloud;

    try{
      tf_.transformPointCloud("base_link", input_scan, laser_cloud);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Transform unavailable %s", ex.what());
      return false;
    }

    filtered_scan.header = input_scan.header;
    filtered_scan.pts.resize (input_scan.pts.size());
    filtered_scan.chan.resize (input_scan.chan.size());
    for (unsigned int d = 0; d < input_scan.get_chan_size (); d++){
      filtered_scan.chan[d].vals.resize  (input_scan.pts.size());
      filtered_scan.chan[d].name = input_scan.chan[d].name;
    }

    int num_pts = 0;
    for (unsigned int i=0; i < laser_cloud.pts.size(); i++)  
    {
      if (!inFootprint(laser_cloud.pts[i])){
        filtered_scan.pts[num_pts] = input_scan.pts[i];
        for (unsigned int d = 0; d < filtered_scan.get_chan_size (); d++)
          filtered_scan.chan[d].vals[num_pts] = input_scan.chan[d].vals[i];
        num_pts++;
      }
    }

    // Resize output vectors
    filtered_scan.pts.resize (num_pts);
    for (unsigned int d = 0; d < filtered_scan.get_chan_size (); d++)
      filtered_scan.chan[d].vals.resize (num_pts);

    return true;
  }


  bool inFootprint(const robot_msgs::Point32& scan_pt){
    if(scan_pt.x < -1.0 * inscribed_radius_ || scan_pt.x > inscribed_radius_ || scan_pt.y < -1.0 * inscribed_radius_ || scan_pt.y > inscribed_radius_)
      return false;
    return true;
  }

private:
  tf::TransformListener tf_;
  LaserProjection projector_;
  double inscribed_radius_;
} ;

typedef robot_msgs::PointCloud RM_POINT_CLOUD;
FILTERS_REGISTER_FILTER(PointCloudFootprintFilter, RM_POINT_CLOUD);
}

#endif // LASER_SCAN_FOOTPRINT_FILTER_H
