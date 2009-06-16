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
#include "robot_self_filter/self_see_filter.h"

class SelfFilter
{
public:

    SelfFilter(void)
    {
	sf_.configure();
	pointCloudSubscriber_ = nh_.subscribe("full_cloud", 1, &SelfFilter::cloudCallback, this);
	pointCloudPublisher_ = nh_.advertise<robot_msgs::PointCloud>("full_cloud_filtered", 1);	
    }
    
    void cloudCallback(const robot_msgs::PointCloudConstPtr &cloud)
    {
	robot_msgs::PointCloud out;
	ros::WallTime tm = ros::WallTime::now();
	sf_.update(*cloud, out);
	double sec = (ros::WallTime::now() - tm).toSec();
	ROS_INFO("Self filter: reduced %d points to %d points in %f seconds", (int)cloud->pts.size(), (int)out.pts.size(), sec);	
	pointCloudPublisher_.publish(out);
    }
    
private:
    
    ros::Publisher                              pointCloudPublisher_;
    ros::Subscriber                             pointCloudSubscriber_;
    ros::NodeHandle                             nh_;        
    filters::SelfFilter<robot_msgs::PointCloud> sf_;
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "self_filter");

    SelfFilter s;
    ros::spin();
    
    return 0;
}

    
    
