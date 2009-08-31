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
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <geometric_shapes/bodies.h>
#include <visualization_msgs/Marker.h>

class FindTrash
{
public:

    FindTrash(void) 
    {
	subscr_ = nh_.subscribe("base_scan_marking_throttled", 1, &FindTrash::cloudCallback, this);
	
	// hack
	trashPub2_ = nh_.advertise<sensor_msgs::PointCloud>("trash_cloud", 1);
	trashPub_ = nh_.advertise<geometry_msgs::PointStamped>("trash_can", 1);
	vmPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
    }
    
private:
    
    void fillResult(const sensor_msgs::PointCloud& data_in, const bodies::Body *body, sensor_msgs::PointCloud& data_out)
    {
	const unsigned int np = data_in.pts.size();

	// fill in output data with points that are NOT on the robot
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
	    if (body->containsPoint(data_in.pts[i].x, data_in.pts[i].y, data_in.pts[i].z))
	    {
		data_out.pts.push_back(data_in.pts[i]);
		for (unsigned int j = 0 ; j < data_out.chan.size() ; ++j)
		    data_out.chan[j].vals.push_back(data_in.chan[j].vals[i]);
		
	    }
	
    }

    unsigned int countPoints(const sensor_msgs::PointCloud &data_in, bodies::Body *sep)
    {
	const unsigned int np = data_in.pts.size();
	unsigned int k = 0;
	for (unsigned int i = 0 ; i < np ; ++i)
	    if (sep->containsPoint(data_in.pts[i].x, data_in.pts[i].y, data_in.pts[i].z))
		k++;
	return k;
    }

    void sendPoint(int id, double x, double y, double z)
    {
	visualization_msgs::Marker mk;
	mk.header.frame_id = "base_laser";
	mk.header.stamp = ros::Time::now();
	
	mk.ns = "ana_are_mere_trash";
	mk.id = id;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;

	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;

	mk.scale.x = mk.scale.y = mk.scale.z = 0.06;

	mk.color.a = 1.0;
	mk.color.r = 0.9;
	mk.color.g = 0.2;
	mk.color.b = 0.1;
	mk.lifetime = ros::Duration(10.0);
	
	vmPub_.publish(mk);
    }

    void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud)
    {
	// large box
	shapes::Box *box_s = new shapes::Box(2, 3, 1); // z does not matter
	bodies::Body *box = bodies::createBodyFromShape(box_s);
	btTransform pose;
	pose.setIdentity();
	pose.setOrigin(btVector3(0.1 + box_s->size[0] / 2, 0, 0));
	box->setPose(pose);
	delete box_s;
	
	sensor_msgs::PointCloud pcloud;
	fillResult(*cloud, box, pcloud);
	delete box;
	pcloud.header.stamp = ros::Time::now();
	
	if (pcloud.pts.size() <= 20)
	    return;
	
	double sx = 0;
	for (unsigned int i = 0 ; i < pcloud.pts.size() ; ++i)
	    sx += pcloud.pts[i].x;
	sx /= (double)pcloud.pts.size();



	// large box 2
	shapes::Box *box_s2 = new shapes::Box(0.1, 3, 1); // z does not matter
	bodies::Body *box2 = bodies::createBodyFromShape(box_s2);
	pose.setIdentity();
	pose.setOrigin(btVector3(sx, 0, 0));
	box2->setPose(pose);
	delete box_s2;
	
	sensor_msgs::PointCloud pcloud2;
	fillResult(pcloud, box2, pcloud2);
	delete box2;
	
	if (pcloud2.pts.size() <= 20)
	    return;
	
	pcloud = pcloud2;
		
	trashPub2_.publish(pcloud);
	
	sx = 0;
	for (unsigned int i = 0 ; i < pcloud.pts.size() ; ++i)
	    sx += pcloud.pts[i].x;
	sx /= (double)pcloud.pts.size();

	

	btTransform fitT;
	fitT.setIdentity();
	fitT.setOrigin(btVector3(sx, 0, 0));
	

	// small box
	shapes::Box *box_sml = new shapes::Box(0.1, 0.1, 0.1); 
	bodies::Body *bound = bodies::createBodyFromShape(box_sml);
	delete box_sml;
	
	double y = -2;
	while (y < 2)
	{
	    y += 0.1;
	    fitT.setOrigin(btVector3(sx, y, 0));
	    bound->setPose(fitT);
	    if (countPoints(pcloud, bound) > 10)
	    {
		fitT.setOrigin(btVector3(sx, y + 0.1, 0));
		unsigned int c1 = countPoints(pcloud, bound);
		fitT.setOrigin(btVector3(sx, y + 0.2, 0));
		unsigned int c2 = countPoints(pcloud, bound);
		fitT.setOrigin(btVector3(sx, y + 0.3, 0));
		unsigned int c3 = countPoints(pcloud, bound);
		if (c1 > 10 && c2 > 10 && c3 > 10)
		{
		    geometry_msgs::PointStamped ps;
		    ps.header = pcloud.header;
		    ps.point.x = sx;
		    ps.point.y = y;
		    ps.point.z = 0;
		    trashPub_.publish(ps);
		    sendPoint(0, sx, y, 0);
		    break;
		}
	    }
	}
	
	delete bound;
    }

    tf::TransformListener                       tf_;
    ros::Subscriber subscr_;
    
    ros::Publisher                              trashPub_;
    ros::Publisher                              trashPub2_;
    ros::Publisher                              vmPub_;
    ros::NodeHandle                             nh_;
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_trash", ros::init_options::AnonymousName);

    FindTrash s;
    ros::spin();
    
    return 0;
}
