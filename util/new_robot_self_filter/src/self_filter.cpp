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

class SelfFilter
{
public:

    SelfFilter(void) : sf_(tf_), mn_(tf_, boost::bind(&SelfFilter::cloudCallback, this, _1), "cloud_in", "", 1)
    {
	sf_.configure(false);
	std::vector<std::string> frames;
	sf_.getLinkFrames(frames);
	mn_.setTargetFrame(frames);
	pointCloudPublisher_ = nh_.advertise<robot_msgs::PointCloud>("cloud_out", 1);
    }
    
private:
    
    void cloudCallback(const robot_msgs::PointCloudConstPtr &cloud)
    {
	robot_msgs::PointCloud out;
	std::vector<bool> mask;
	ros::WallTime tm = ros::WallTime::now();
	sf_.mask(*cloud, mask);
	double sec = (ros::WallTime::now() - tm).toSec();
	ROS_DEBUG("Self filter: reduced %d points to %d points in %f seconds", (int)cloud->pts.size(), (int)out.pts.size(), sec);
	fillResult(*cloud, mask, out);
	pointCloudPublisher_.publish(out);
    }

    void fillResult(const robot_msgs::PointCloud& data_in, const std::vector<bool> &keep, robot_msgs::PointCloud& data_out)
    {
	const unsigned int np = data_in.pts.size();
	
	// fill in output data 
	data_out.header = data_in.header;	  
	
	data_out.pts.resize(0);
	data_out.pts.reserve(np);
	
	data_out.chan.resize(data_in.chan.size());
	for (unsigned int i = 0 ; i < data_out.chan.size() ; ++i)
	{
	    ROS_ASSERT(data_in.chan[i].vals.size() == data_in.pts.size());
	    data_out.chan[i].name = data_in.chan[i].name;
	    data_out.chan[i].vals.reserve(data_in.chan[i].vals.size());
	}
	
	for (unsigned int i = 0 ; i < np ; ++i)
	    if (keep[i])
	    {
		data_out.pts.push_back(data_in.pts[i]);
		for (unsigned int j = 0 ; j < data_out.chan.size() ; ++j)
		    data_out.chan[j].vals.push_back(data_in.chan[j].vals[i]);
	    }
    }
    
    tf::TransformListener                       tf_;
    robot_self_filter::SelfMask                 sf_;
    tf::MessageNotifier<robot_msgs::PointCloud> mn_;
    ros::Publisher                              pointCloudPublisher_;
    ros::NodeHandle                             nh_;
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "self_filter", ros::init_options::AnonymousName);

    SelfFilter s;
    ros::spin();
    
    return 0;
}
