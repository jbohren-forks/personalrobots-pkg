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

/** \author Ioan Sucan */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

class DrawRays
{
public:

    DrawRays(void) : cloudNotifier_(tf_, boost::bind(&DrawRays::cloudCallback, this, _1), "cloud_in", "laser_tilt_mount_link", 1)
    {
	id_ = 1;
	vmPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
	nodeHandle_.param<std::string>("~sensor_frame", sensor_frame_, "laser_tilt_mount_link");
    }
    
    ~DrawRays(void)
    {
    }

    void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud)
    {
	// compute the origin of the sensor in the frame of the cloud
	btVector3 sensor_pos;
	try
	{
	    tf::Stamped<btTransform> transf;
	    tf_.lookupTransform(cloud->header.frame_id, sensor_frame_, cloud->header.stamp, transf);
	    sensor_pos = transf.getOrigin();
	}
	catch(...)
	{
	    sensor_pos.setValue(0, 0, 0);
	    ROS_ERROR("Unable to lookup transform from %s to %s", sensor_frame_.c_str(), cloud->header.frame_id.c_str());
	}
	for (unsigned int i = 0 ; i < cloud->pts.size() ; ++i)
	    sendLine(cloud->header, cloud->pts[i].x, cloud->pts[i].y, cloud->pts[i].z, sensor_pos.x(), sensor_pos.y(), sensor_pos.z());
	id_ = 1;
    }
    
    void sendLine(const roslib::Header &header, double x1, double y1, double z1, double x2, double y2, double z2)
    {
	visualization_msgs::Marker mk;

	mk.header = header;

	mk.ns = "draw_rays";
	mk.id = id_++;
	mk.type = visualization_msgs::Marker::ARROW;
	mk.action = visualization_msgs::Marker::ADD;
	mk.pose.position.x = 0;
	mk.pose.position.y = 0;
	mk.pose.position.z = 0;
	mk.pose.orientation.x = 0;
	mk.pose.orientation.y = 0;
	mk.pose.orientation.z = 0;
	mk.pose.orientation.w = 1.0;

	mk.scale.x = mk.scale.y = mk.scale.z = 0.01;

	mk.color.a = 1.0;
	mk.color.r = 0.7;
	mk.color.g = 0.4;
	mk.color.b = 0.4;
	
	mk.points.resize(2);
	mk.points[0].x = x1;
	mk.points[0].y = y1;
	mk.points[0].z = z1;
	
	mk.points[1].x = x2;
	mk.points[1].y = y2;
	mk.points[1].z = z2;

	mk.lifetime = ros::Duration(2);
	
	vmPub_.publish(mk);
    }

protected:
    
  
    ros::NodeHandle                             nodeHandle_;        
    tf::TransformListener                       tf_;
    tf::MessageNotifier<sensor_msgs::PointCloud> cloudNotifier_;
    std::string                                 sensor_frame_;
    
    ros::Publisher                              vmPub_;
    int                                         id_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_rays");

    DrawRays t;
    ros::spin();
    
    return 0;
}

    
    
