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
#include <robot_msgs/PointCloud.h>
#include <mapping_msgs/CollisionMap.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <robot_self_filter/self_mask.h>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <algorithm>
#include <set>
#include <iterator>
#include <cstdlib>

class CollisionMapperOcc
{
public:
    
    CollisionMapperOcc(void) : sf_(tf_)			       
    { 
	// read ROS params
	loadParams();
	
	// advertise our topics: full map and updates
	cmapPublisher_ = nh_.advertise<mapping_msgs::CollisionMap>("collision_map_occ", 1);
	cmapUpdPublisher_ = nh_.advertise<mapping_msgs::CollisionMap>("collision_map_occ_update", 1);
	if (publishOcclusion_)
  	    occPublisher_ = nh_.advertise<mapping_msgs::CollisionMap>("collision_map_occ_occlusion", 1);

	// create a message notifier (and enable subscription) for both the full map and for the updates
	mnCloud_ = new tf::MessageNotifier<robot_msgs::PointCloud>(tf_, boost::bind(&CollisionMapperOcc::cloudCallback, this, _1), "cloud_in", "", 1);
	mnCloudIncremental_ = new tf::MessageNotifier<robot_msgs::PointCloud>(tf_, boost::bind(&CollisionMapperOcc::cloudIncrementalCallback, this, _1), "cloud_incremental_in", "", 1);

	// configure the self mask and the message notifier
	std::vector<std::string> frames;
	sf_.getLinkNames(frames);
	if (std::find(frames.begin(), frames.end(), robotFrame_) == frames.end())
	    frames.push_back(robotFrame_);
	mnCloud_->setTargetFrame(frames);
	mnCloudIncremental_->setTargetFrame(frames);
    }
    
    ~CollisionMapperOcc(void)
    {
	delete mnCloud_;
	delete mnCloudIncremental_;
    }

    void run(void)
    {
	if (bi_.sensor_frame.empty())
	    ROS_ERROR("No sensor frame specified. Cannot perform raytracing");
	else
	    ros::spin();
    }
        
private:
    
    struct CollisionPoint
    {
	CollisionPoint(void) {}
	CollisionPoint(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
	
	int x, y, z;
    };

    // define an order on points
    struct CollisionPointOrder
    {
	bool operator()(const CollisionPoint &a, const CollisionPoint &b) const
	{
	    if (a.x < b.x)
		return true;
	    if (a.x > b.x)
		return false;
	    if (a.y < b.y)
		return true;
	    if (a.y > b.y)
		return false;
	    return a.z < b.z;
	}
    };
    
    typedef std::set<CollisionPoint, CollisionPointOrder> CMap;

    // parameters & precomputed values for the box that represents the collision map
    // around the robot
    struct BoxInfo
    {    
	double dimensionX, dimensionY, dimensionZ;
	double originX, originY, originZ;
	std::string sensor_frame;
	double resolution;
	double real_minX, real_minY, real_minZ;
	double real_maxX, real_maxY, real_maxZ;
    };
    
    void loadParams(void)
    {
	// a frame that does not move with the robot
	nh_.param<std::string>("~fixed_frame", fixedFrame_, "odom");

	// a frame that moves with the robot
	nh_.param<std::string>("~robot_frame", robotFrame_, "base_link");

	// bounds of collision map in robot frame
	nh_.param<double>("~dimension_x", bi_.dimensionX, 1.0);
	nh_.param<double>("~dimension_y", bi_.dimensionY, 1.5);
	nh_.param<double>("~dimension_z", bi_.dimensionZ, 2.0);

	// origin of collision map in the robot frame
	nh_.param<double>("~origin_x", bi_.originX, 1.1);
	nh_.param<double>("~origin_y", bi_.originY, 0.0);
	nh_.param<double>("~origin_z", bi_.originZ, 0.0);

	// sensor frame
	nh_.param<std::string>("~sensor_frame", bi_.sensor_frame, std::string());
	
	// resolution
	nh_.param<double>("~resolution", bi_.resolution, 0.015);
	
	ROS_INFO("Maintaining occlusion map in frame '%s', with origin at (%f, %f, %f) and dimension (%f, %f, %f), resolution of %f; "
		 "sensor is in frame '%s', fixed fame is '%s'.",
		 robotFrame_.c_str(), bi_.dimensionX, bi_.dimensionY, bi_.dimensionZ, bi_.originX, bi_.originY, bi_.originZ,
		 bi_.resolution, bi_.sensor_frame.c_str(), fixedFrame_.c_str());

	nh_.param<bool>("~publish_occlusion", publishOcclusion_, false);

	// compute some useful values
	bi_.real_minX = -bi_.dimensionX + bi_.originX;
	bi_.real_maxX =  bi_.dimensionX + bi_.originX;
	bi_.real_minY = -bi_.dimensionY + bi_.originY;
	bi_.real_maxY =  bi_.dimensionY + bi_.originY;
	bi_.real_minZ = -bi_.dimensionZ + bi_.originZ;
	bi_.real_maxZ =  bi_.dimensionZ + bi_.originZ;	
    }
    
    void cloudIncrementalCallback(const robot_msgs::PointCloudConstPtr &cloud)
    {
	if (!mapProcessing_.try_lock())
	    return;

	ROS_DEBUG("Got pointcloud update that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
	
	robot_msgs::PointCloud out;
	ros::WallTime tm = ros::WallTime::now();

	// transform the pointcloud to the robot frame
	// since we need the points in this frame (around the robot)
	// to compute the collision map
	tf_.transformPointCloud(robotFrame_, *cloud, out);
	
	CMap obstacles;
	constructCollisionMap(out, obstacles);

	CMap diff;
	set_difference(obstacles.begin(), obstacles.end(), currentMap_.begin(), currentMap_.end(),
		       std::inserter(diff, diff.begin()), CollisionPointOrder());
	mapProcessing_.unlock();
	
	if (!diff.empty())
	    publishCollisionMap(diff, out.header, cmapUpdPublisher_);
    }
    
    void cloudCallback(const robot_msgs::PointCloudConstPtr &cloud)
    {
	ROS_DEBUG("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
	
	mapProcessing_.lock();
	
	robot_msgs::PointCloud out;
	ros::WallTime tm = ros::WallTime::now();
	
	CMap obstacles;
	
	// transform the pointcloud to the robot frame
	// since we need the points in this frame (around the robot)
	// to compute the collision map
	tf_.transformPointCloud(robotFrame_, *cloud, out);
	constructCollisionMap(out, obstacles);
	
	// try to transform the previous map (if it exists) to the new frame
	if (!currentMap_.empty())
	    if (!transformMap(currentMap_, header_, out.header))
		currentMap_.clear();
	header_ = out.header;
	
	// update map
	updateMap(obstacles);

	double sec = (ros::WallTime::now() - tm).toSec();
	ROS_INFO("Updated collision map with %d points at %f Hz", currentMap_.size(), 1.0/sec);
	
	publishCollisionMap(currentMap_, header_, cmapPublisher_);
	mapProcessing_.unlock();
    }

    void updateMap(CMap &obstacles)
    {
	if (currentMap_.empty())
	    currentMap_ = obstacles;
	else
	{
	    CMap diff;
	    
	    // find the points from the old map that are no longer visible
	    set_difference(currentMap_.begin(), currentMap_.end(), obstacles.begin(), obstacles.end(),
			   std::inserter(diff, diff.begin()), CollisionPointOrder());
	    
	    // the current map will at least contain the new info
	    currentMap_ = obstacles;

	    // find out which of these points are now occluded 
	    sf_.assumeFrame(header_, bi_.sensor_frame);
	    
	    // OpenMP need an int as the lookup variable, but for set,
	    // this is not possible, so we copy to a vector
	    int n = diff.size();
	    std::vector<CollisionPoint> pts(n);
	    std::copy(diff.begin(), diff.end(), pts.begin());

	    // add points occluded by self
	    if (publishOcclusion_)
	    {
		CMap keep;
		
#pragma omp parallel for
		for (int i = 0 ; i < n ; ++i)
		{
		    btVector3 p(((double)pts[i].x - 0.5) * bi_.resolution + bi_.originX,
				((double)pts[i].y - 0.5) * bi_.resolution + bi_.originY,
				((double)pts[i].z - 0.5) * bi_.resolution + bi_.originZ);
		    if (sf_.getMaskIntersection(p) == robot_self_filter::SHADOW)
		    {
#pragma omp critical
			{
			    keep.insert(pts[i]);
			    currentMap_.insert(pts[i]);
			}
		    }
		}
		
		publishCollisionMap(keep, header_, occPublisher_);
	    }
	    else
	    {
#pragma omp parallel for
		for (int i = 0 ; i < n ; ++i)
		{
		    btVector3 p(((double)pts[i].x - 0.5) * bi_.resolution + bi_.originX,
				((double)pts[i].y - 0.5) * bi_.resolution + bi_.originY,
				((double)pts[i].z - 0.5) * bi_.resolution + bi_.originZ);
		    if (sf_.getMaskIntersection(p) == robot_self_filter::SHADOW)
		    {
#pragma omp critical
			{
			    currentMap_.insert(pts[i]);
			}
		    }		
		}
		
	    }
	    
	}
    }
    
    bool transformMap(CMap &map, const roslib::Header &from, const roslib::Header &to)
    {
	tf::Stamped<btTransform> transf;
	try
	{
	    tf_.lookupTransform(to.frame_id, to.stamp, from.frame_id, from.stamp, fixedFrame_, transf);
	}
	catch(...)
	{
	    ROS_WARN("Unable to transform previous collision map into new frame");
	    return false;
	}
	
	// copy data to temporary location
	const int n = map.size();
	std::vector<CollisionPoint> pts(n);
	std::copy(map.begin(), map.end(), pts.begin());
	map.clear();
	
#pragma omp parallel for
	for (int i = 0 ; i < n ; ++i)
	{
	    btVector3 p(((double)pts[i].x - 0.5) * bi_.resolution + bi_.originX,
			((double)pts[i].y - 0.5) * bi_.resolution + bi_.originY,
			((double)pts[i].z - 0.5) * bi_.resolution + bi_.originZ);
	    p = transf * p;
	    if (p.x() > bi_.real_minX && p.x() < bi_.real_maxX && p.y() > bi_.real_minY && p.y() < bi_.real_maxY && p.z() > bi_.real_minZ && p.z() < bi_.real_maxZ)
	    {
		CollisionPoint c((int)(0.5 + (p.x() - bi_.originX) / bi_.resolution),
				 (int)(0.5 + (p.y() - bi_.originY) / bi_.resolution),
				 (int)(0.5 + (p.z() - bi_.originZ) / bi_.resolution));
#pragma omp critical
		{
		    map.insert(c);
		}
		
	    }
	}
	
	return true;

    }

    /** Construct an axis-aligned collision map from a point cloud assumed to be in the robot frame */
    void constructCollisionMap(const robot_msgs::PointCloud &cloud, CMap &map)
    {
	const unsigned int n = cloud.pts.size();
	CollisionPoint c;
	
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    const robot_msgs::Point32 &p = cloud.pts[i];
	    if (p.x > bi_.real_minX && p.x < bi_.real_maxX && p.y > bi_.real_minY && p.y < bi_.real_maxY && p.z > bi_.real_minZ && p.z < bi_.real_maxZ)
	    {
		c.x = (int)(0.5 + (p.x - bi_.originX) / bi_.resolution);
		c.y = (int)(0.5 + (p.y - bi_.originY) / bi_.resolution);
		c.z = (int)(0.5 + (p.z - bi_.originZ) / bi_.resolution);
		map.insert(c);
	    }
	}
    }
    
    void publishCollisionMap(const CMap &map, const roslib::Header &header, ros::Publisher &pub) const
    {
	mapping_msgs::CollisionMap cmap;
	cmap.header = header;
	const unsigned int ms = map.size();
	
	for (CMap::const_iterator it = map.begin() ; it != map.end() ; ++it)
	{
	    const CollisionPoint &cp = *it;
	    mapping_msgs::OrientedBoundingBox box;
	    box.extents.x = box.extents.y = box.extents.z = bi_.resolution;
	    box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
	    box.angle = 0.0;
	    box.center.x = cp.x * bi_.resolution + bi_.originX;
	    box.center.y = cp.y * bi_.resolution + bi_.originY;
	    box.center.z = cp.z * bi_.resolution + bi_.originZ;
	    cmap.boxes.push_back(box);
	}
	pub.publish(cmap);
	
	ROS_DEBUG("Published collision map with %u boxes", ms);
    }

    tf::TransformListener                        tf_;
    robot_self_filter::SelfMask                  sf_;
    tf::MessageNotifier<robot_msgs::PointCloud> *mnCloud_;
    tf::MessageNotifier<robot_msgs::PointCloud> *mnCloudIncremental_;
    ros::NodeHandle                              nh_;
    ros::Publisher                               cmapPublisher_;
    ros::Publisher                               cmapUpdPublisher_;
    ros::Publisher                               occPublisher_;
    roslib::Header                               header_;
    bool                                         publishOcclusion_;
    
    boost::mutex                                 mapProcessing_;
    CMap                                         currentMap_;
    
    BoxInfo                                      bi_;
    std::string                                  fixedFrame_;
    std::string                                  robotFrame_;
    
};  

int main (int argc, char** argv)
{
    ros::init(argc, argv, "collision_map_self_occ", ros::init_options::AnonymousName);

    CollisionMapperOcc cm;
    cm.run();
    
    return 0;
}
