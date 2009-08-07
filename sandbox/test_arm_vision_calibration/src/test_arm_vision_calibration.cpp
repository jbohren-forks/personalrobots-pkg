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
#include "robot_self_filter/self_mask.h"
#include <tf/message_notifier.h>
#include <geometry_msgs/Point.h>

class TestArmVisionCallibration
{
public:

    TestArmVisionCallibration(void) : mn_(tf_, boost::bind(&TestArmVisionCallibration::cloudCallback, this, _1), "cloud_in", "", 1)
    {
	nh_.param<double>("~error", error_, 0.02);
	sfMin_ = new robot_self_filter::SelfMask(tf_, 1.0, -error_);
	sfMax_ = new robot_self_filter::SelfMask(tf_, 1.0,  error_);
	d_.x = 1;
	d_.y = 1;
	d_.z = 0.5;
	
	std::vector<std::string> frames;
	sfMax_->getLinkNames(frames);
	mn_.setTargetFrame(frames);

	pointCloudPublisherMax_ = nh_.advertise<sensor_msgs::PointCloud>("should_be_empty", 1);
	pointCloudPublisherMin_ = nh_.advertise<sensor_msgs::PointCloud>("should_be_full",  1);
    }

    ~TestArmVisionCallibration(void)
    {
	delete sfMin_;
	delete sfMax_;
    }
    
    void run(void)
    {
	ros::spin();
    }

private:

    void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud)
    {
	ROS_INFO("Received %f second old pointcloud with %d points", (ros::Time::now() - cloud->header.stamp).toSec(), cloud->points.size());
	
	sensor_msgs::PointCloud transformed;
	tf_.transformPointCloud("torso_lift_link", *cloud, transformed);

	sensor_msgs::PointCloud close;
	keepClose(transformed, close);

	sensor_msgs::PointCloud outMax;
	sensor_msgs::PointCloud outMin;
	std::vector<int> maskMax;
	std::vector<int> maskMin;
	sfMax_->maskContainment(close, maskMax);
	sfMin_->maskContainment(close, maskMin);
	fillResult(close, maskMax, outMax);
	fillResult(close, maskMin, outMin);
	pointCloudPublisherMax_.publish(outMax);
	pointCloudPublisherMin_.publish(outMin);
	
	if (outMax.points.size() > 0.1 * close.points.size())
	    ROS_ERROR("%f%% of points are seen closer to the sensor (as compared to where they should be)", 100.0 * (double)outMax.points.size() / (double)close.points.size());
	
	if (outMin.points.size() < 0.9 * close.points.size())
	    ROS_ERROR("%f%% of points are seen further to the sensor (as compared to where they should be)", 100.0 - 100.0 * (double)outMin.points.size() / (double)close.points.size());
    }
    
    void fillResult(const sensor_msgs::PointCloud& data_in, const std::vector<int> &keep, sensor_msgs::PointCloud& data_out)
    {
	const unsigned int np = data_in.points.size();
	
	// fill in output data with points that are NOT on the robot
	data_out.header = data_in.header;	  
	
	data_out.points.resize(0);
	data_out.points.reserve(np);
	
	data_out.channels.resize(data_in.channels.size());
	for (unsigned int i = 0 ; i < data_out.channels.size() ; ++i)
	{
	    ROS_ASSERT(data_in.channels[i].values.size() == data_in.points.size());
	    data_out.channels[i].name = data_in.channels[i].name;
	    data_out.channels[i].values.reserve(data_in.channels[i].values.size());
	}
	
	for (unsigned int i = 0 ; i < np ; ++i)
	{
	    if (keep[i] == robot_self_filter::OUTSIDE)
	    {
		data_out.points.push_back(data_in.points[i]);
		for (unsigned int j = 0 ; j < data_out.channels.size() ; ++j)
		    data_out.channels[j].values.push_back(data_in.channels[j].values[i]);
	    }
	}
    }

    // keep points in a bounding box around robot
    void keepClose(const sensor_msgs::PointCloud& data_in, sensor_msgs::PointCloud& data_out)
    {	
	const unsigned int np = data_in.points.size();
	
	// fill in output data with points that are NOT on the robot
	data_out.header = data_in.header;	  
	
	data_out.points.resize(0);
	data_out.points.reserve(np);
	
	data_out.channels.resize(data_in.channels.size());
	for (unsigned int i = 0 ; i < data_out.channels.size() ; ++i)
	{
	    ROS_ASSERT(data_in.channels[i].values.size() == data_in.points.size());
	    data_out.channels[i].name = data_in.channels[i].name;
	    data_out.channels[i].values.reserve(data_in.channels[i].values.size());
	}
	
	for (unsigned int i = 0 ; i < np ; ++i)
	{
	    if (fabs(data_in.points[i].x) < d_.x && fabs(data_in.points[i].y) < d_.y && fabs(data_in.points[i].z) < d_.z)
	    {
		data_out.points.push_back(data_in.points[i]);
		for (unsigned int j = 0 ; j < data_out.channels.size() ; ++j)
		    data_out.channels[j].values.push_back(data_in.channels[j].values[i]);
	    }
	}
    }
    
    
    ros::NodeHandle              nh_;
    tf::TransformListener        tf_;
    robot_self_filter::SelfMask *sfMin_;
    robot_self_filter::SelfMask *sfMax_;
    double                       error_;
    ros::Publisher               pointCloudPublisherMin_;
    ros::Publisher               pointCloudPublisherMax_;   

    geometry_msgs::Point            d_;
    
    tf::MessageNotifier<sensor_msgs::PointCloud> mn_;
};
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_arm_vision_callibration");

    TestArmVisionCallibration t;
    t.run();
    
    return 0;
}
