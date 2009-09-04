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

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

class PointCloudWriter
{

public:

  PointCloudWriter()
    : pc_sub_(nh_, "point_cloud", 1)
  {
    ros::NodeHandle local_nh("~");
    local_nh.param("out_directory",out_directory_,std::string("/tmp/"));
    frame_ = "base_link";
    tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pc_sub_, tf_, frame_,1);
    tf_filter_->registerCallback(boost::bind(&PointCloudWriter::pointCloudCB, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.03));
  }

  ~PointCloudWriter()
  {
    delete tf_filter_;
  }

  void pointCloudCB(const sensor_msgs::PointCloudConstPtr& pc)
  { 

    // Convert the point cloud to the desired frame.
    sensor_msgs::PointCloud pc_transformed; 
    tf_.transformPointCloud(frame_, *pc, pc_transformed);

    std::stringstream ss;
    ss << out_directory_ << "/" << pc->header.stamp << "." << frame_ << ".xyz";
    std::fstream file(ss.str().c_str(), std::fstream::out);
    
    ROS_INFO_STREAM("Writing file "<<ss.str());
    
    // Write out the point cloud including all of the channels.
    ROS_INFO_STREAM("Num points " << pc_transformed.points.size());
    for (uint i=0; i<pc_transformed.points.size(); ++i) 
    {
      file << pc_transformed.points[i].x << " " << pc_transformed.points[i].y << " " << pc_transformed.points[i].z;
          
      for (uint j=0; j<pc_transformed.channels.size(); ++j) 
	file << " " << pc_transformed.channels[j].values[i];
	      
      file << std::endl;
    }

    file.close();

    ROS_INFO_STREAM("File closed");
  }
  
protected:
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::PointCloud> pc_sub_;
  std::string frame_;
  tf::MessageFilter<sensor_msgs::PointCloud> *tf_filter_;
  std::string out_directory_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_to_file");
  PointCloudWriter pc;

  ros::spin();

  return 0;
}
