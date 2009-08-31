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
#include <find_bottles/FindBottles.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <geometric_shapes/bodies.h>
#include <visualization_msgs/Marker.h>

class FindBottle
{
public:

    FindBottle(void) : mn_(tf_, boost::bind(&FindBottle::cloudCallback, this, _1), "cloud_in", "base_link", 1)
    {
	srv_ = nh_.advertiseService("find_bottles", &FindBottle::findBottle, this);
	cloudPub_ = nh_.advertise<sensor_msgs::PointCloud>("bottles", 1);
	vmPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
    }
    
private:
    
    void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud)
    {
	inp = *cloud;
	/*
	find_bottles::FindBottles::Request req;
	find_bottles::FindBottles::Response res;
	req.z = 0.85;   // bar under TV
	// req.z = 0.76;         // rectangular table
	//	req.z = 0.75;         // round table
	findBottle(req, res);
	*/
    }

    unsigned int countPoints(const sensor_msgs::PointCloud &data_in, bodies::Body *sep)
    {
	const unsigned int np = data_in.points.size();
	unsigned int k = 0;
	for (unsigned int i = 0 ; i < np ; ++i)
	    if (sep->containsPoint(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z))
		k++;
	return k;
    }

    void sendPoint(int id, double x, double y, double z)
    {
	visualization_msgs::Marker mk;
	mk.header.frame_id = "base_link";
	mk.header.stamp = ros::Time::now();
	
	mk.ns = "ana_are_mere";
	mk.id = id;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;

	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;

	mk.scale.x = mk.scale.y = mk.scale.z = 0.04;

	mk.color.a = 1.0;
	mk.color.r = 0.1;
	mk.color.g = 0.9;
	mk.color.b = 0.1;
	mk.lifetime = ros::Duration(10.0);
	
	vmPub_.publish(mk);
    }

    void setObject(const shapes::Shape *obj, visualization_msgs::Marker &mk)
    {
	switch (obj->type)
	{
	case shapes::SPHERE:
	    mk.type = visualization_msgs::Marker::SPHERE;
	    mk.scale.x = mk.scale.y = mk.scale.z = static_cast<const shapes::Sphere*>(obj)->radius * 2.0;
	    break;
	    
	case shapes::BOX:
	    mk.type = visualization_msgs::Marker::CUBE;
	    {
		const double *size = static_cast<const shapes::Box*>(obj)->size;
		mk.scale.x = size[0];
		mk.scale.y = size[1];
		mk.scale.z = size[2];
	    }
	    break;

	case shapes::CYLINDER:
	    mk.type = visualization_msgs::Marker::CYLINDER;
	    mk.scale.x = static_cast<const shapes::Cylinder*>(obj)->radius * 2.0;
	    mk.scale.y = mk.scale.x;
	    mk.scale.z = static_cast<const shapes::Cylinder*>(obj)->length;
	    break;

	case shapes::MESH:
	    mk.type = visualization_msgs::Marker::LINE_LIST;
	    mk.scale.x = mk.scale.y = mk.scale.z = 0.02;
	    {	   
		const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(obj);
		unsigned int nt = mesh->triangleCount / 3;
		for (unsigned int i = 0 ; i < nt ; ++i)
		{
		    unsigned int v = mesh->triangles[3*i];
		    geometry_msgs::Point pt1;
		    pt1.x = mesh->vertices[v];
		    pt1.y = mesh->vertices[v+1];
		    pt1.z = mesh->vertices[v+2];
		    mk.points.push_back(pt1);

		    v = mesh->triangles[3*i + 1];
		    geometry_msgs::Point pt2;
		    pt2.x = mesh->vertices[v];
		    pt2.y = mesh->vertices[v+1];
		    pt2.z = mesh->vertices[v+2];
		    mk.points.push_back(pt2);

		    mk.points.push_back(pt1);

		    v = mesh->triangles[3*i + 2];
		    geometry_msgs::Point pt3;
		    pt3.x = mesh->vertices[v];
		    pt3.y = mesh->vertices[v+1];
		    pt3.z = mesh->vertices[v+2];
		    mk.points.push_back(pt3);

		    mk.points.push_back(pt2);
		    mk.points.push_back(pt3);
		}
	    }
	    
	    break;
	    
	default:
	    ROS_ERROR("Unknown object type: %d", (int)obj->type);
	}
    }
    
    bool findBottle(find_bottles::FindBottles::Request &req, find_bottles::FindBottles::Response &res)
    {
      if (inp.points.size() == 0)
	{
	  ROS_WARN("No data yet...");
	  return true;
	}

	ros::WallTime tm = ros::WallTime::now();
	
	sensor_msgs::PointCloud cloud;
	tf_.transformPointCloud("base_link", inp, cloud);
	
	// large box
	shapes::Box *box_s = new shapes::Box(0.4, 3, 0.25);
	bodies::Body *box = bodies::createBodyFromShape(box_s);
	btTransform pose;
	pose.setIdentity();
	pose.setOrigin(btVector3(0.5 + box_s->size[0] / 2, 0, req.z + box_s->size[2]/2));
	box->setPose(pose);
	
	
	sensor_msgs::PointCloud c_out;
	fillResult(cloud, box, c_out);
	

	// separator box
	shapes::Box *box_bottle = new shapes::Box(box_s->size[0], 0.1, 0.2);
	bodies::Body *bottle = bodies::createBodyFromShape(box_bottle);

	// separator box
	shapes::Box *box_sep = new shapes::Box(box_s->size[0], 0.03, 2);
	bodies::Body *sep = bodies::createBodyFromShape(box_sep);
	
	double y = - box_s->size[1] / 2 - box_sep->size[1] / 2;
	bool clear = false;
	int id = 0;
	while (y < box_s->size[1] / 2)
	{
	    y += box_sep->size[1];
	    
	    btTransform pose;
	    pose.setIdentity();
	    pose.setOrigin(btVector3(0.5 + box_s->size[0] / 2, y, req.z));
	    sep->setPose(pose);

	    /*
	    visualization_msgs::Marker mk;
	    mk.ns = ros::this_node::getName();
	    mk.id = id++;
	    mk.header.frame_id = "base_link";
	    mk.header.stamp = ros::Time::now();
	    mk.action = visualization_msgs::Marker::ADD;
	    setObject(box_sep, mk);
	    tf::poseTFToMsg(pose, mk.pose);
	    mk.lifetime = ros::Duration(1.0);
	    
	    mk.color.a = 0.5;
	    mk.color.r = 0.6;
	    mk.color.g = 0.4;
	    mk.color.b = 0.3;
	    vmPub_.publish(mk);
	    */

	    unsigned int cp = countPoints(c_out, sep);

	    // candidate bottle
	    if (clear && cp > 2)
	    {
		btTransform poseb;
		poseb.setIdentity();
		poseb.setOrigin(btVector3(0.5 + box_s->size[0] / 2, y - box_sep->size[1], req.z + box_s->size[2]/2));
		bottle->setPose(poseb);
		unsigned int pp = countPoints(c_out, bottle);
		if (pp > 20)
		{
		    clear = true;
		    y += 0.1;
		    sensor_msgs::PointCloud c_out2;
		    fillResult(c_out, bottle, c_out2);
		    std::vector<double> xv;
		    double sy, sz;
		    sy = sz = 0.0;

		    for (unsigned int i = 0 ; i < c_out2.points.size() ; ++i)
			xv.push_back(c_out2.points[i].x);
		    std::sort(xv.begin(), xv.end());
		    double ppx = xv[xv.size()/3];
		    unsigned int nn = 0;
		    for (unsigned int i = 0 ; i < c_out2.points.size() ; ++i)
		    {
			if (c_out2.points[i].x - ppx < 0.04)
			{
			    sy += c_out2.points[i].y;
			    sz += c_out2.points[i].z;
			    nn++;
			}
		    }
		    
		    if (nn > 10)
		    {
			btVector3 pz(ppx, sy/(double)nn, sz/(double)nn);
			sendPoint(id++, pz.x(), pz.y(), pz.z());
			geometry_msgs::PointStamped ps;
			ps.header.frame_id = "base_link";
			ps.header.stamp = ros::Time::now();
			ps.point.x = pz.x();
			ps.point.y = pz.y();
			ps.point.z = pz.z();
			res.pts.push_back(ps);
		    }
		}
	    }
	    
	    if (cp <= 2)
		clear = true;
	    
	    //	    ROS_INFO("%d", cp);
	}
	
	
	cloudPub_.publish(c_out);
	

	delete box_s;
	delete box_sep;
	delete box;
	delete sep;
	delete box_bottle;
	delete bottle;
	
	ROS_INFO("Spend %f seconds finding bottles", (ros::WallTime::now() - tm).toSec());
	
	return true;
    }
    
    void fillResult(const sensor_msgs::PointCloud& data_in, const bodies::Body *body, sensor_msgs::PointCloud& data_out)
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
	    if (body->containsPoint(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z))
	    {
		data_out.points.push_back(data_in.points[i]);
		for (unsigned int j = 0 ; j < data_out.channels.size() ; ++j)
		    data_out.channels[j].values.push_back(data_in.channels[j].values[i]);
	    }
    }
    
    tf::TransformListener                       tf_;
    sensor_msgs::PointCloud                     inp;
    tf::MessageNotifier<sensor_msgs::PointCloud> mn_;
    ros::ServiceServer                          srv_;
    ros::Publisher                              cloudPub_;
    ros::Publisher                              vmPub_;
    ros::NodeHandle                             nh_;
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_bottles", ros::init_options::AnonymousName);

    FindBottle s;
    ros::spin();
    
    return 0;
}
