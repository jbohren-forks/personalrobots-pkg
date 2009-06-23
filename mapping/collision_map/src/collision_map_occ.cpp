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
#include <robot_msgs/CollisionMap.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <robot_self_filter/self_mask.h>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <algorithm>
#include <set>
#include <iterator>

class CollisionMapperOcc
{
public:
    
    CollisionMapperOcc(void) : sf_(tf_),
			       mn_(tf_, boost::bind(&CollisionMapperOcc::cloudCallback, this, _1), "cloud_in", "", 1)
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
	
	// sensor origin in robot frame
	nh_.param<double>("~sensor_x", bi_.sensorX, 0.05);
	nh_.param<double>("~sensor_y", bi_.sensorY, 0.0);
	nh_.param<double>("~sensor_z", bi_.sensorZ, 1.0);
	
	// resolution
	nh_.param<double>("~resolution", bi_.resolution, 0.015);
	
	ROS_INFO("Maintaining occlusion map in frame '%s', with origin at (%f, %f, %f) and dimension (%f, %f, %f), resolution of %f; sensor is at (%f, %f, %f), fixed fame is '%s'.",
		 robotFrame_.c_str(), bi_.dimensionX, bi_.dimensionY, bi_.dimensionZ, bi_.originX, bi_.originY, bi_.originZ, bi_.resolution, bi_.sensorX, bi_.sensorY, bi_.sensorZ, fixedFrame_.c_str());
	
	// compute some useful values
	bi_.sx = (int)(0.5 + (bi_.sensorX - bi_.originX) / bi_.resolution);
	bi_.sy = (int)(0.5 + (bi_.sensorY - bi_.originY) / bi_.resolution);
	bi_.sz = (int)(0.5 + (bi_.sensorZ - bi_.originZ) / bi_.resolution);

	bi_.minX = (int)(0.5 + (-bi_.dimensionX - bi_.originX) / bi_.resolution);
	bi_.maxX = (int)(0.5 + (bi_.dimensionX - bi_.originX) / bi_.resolution);

	bi_.minY = (int)(0.5 + (-bi_.dimensionY - bi_.originY) / bi_.resolution);
	bi_.maxY = (int)(0.5 + (bi_.dimensionY - bi_.originY) / bi_.resolution);
	
	bi_.minZ = (int)(0.5 + (-bi_.dimensionZ - bi_.originZ) / bi_.resolution);
	bi_.maxZ = (int)(0.5 + (bi_.dimensionZ - bi_.originZ) / bi_.resolution);

	bi_.real_minX = -bi_.dimensionX + bi_.originX;
	bi_.real_maxX =  bi_.dimensionX + bi_.originX;
	bi_.real_minY = -bi_.dimensionY + bi_.originY;
	bi_.real_maxY =  bi_.dimensionY + bi_.originY;
	bi_.real_minZ = -bi_.dimensionZ + bi_.originZ;
	bi_.real_maxZ =  bi_.dimensionZ + bi_.originZ;


	// configure the self mask and the message notifier
	sf_.configure();
	std::vector<std::string> frames;
	sf_.getLinkFrames(frames);
	if (std::find(frames.begin(), frames.end(), robotFrame_) == frames.end())
	    frames.push_back(robotFrame_);
	mn_.setTargetFrame(frames);
	
	// advertise our topic
	cmapPublisher_ = nh_.advertise<robot_msgs::CollisionMap>("collision_map_occ", 1);
    }
    
private:
    
    struct CollisionPoint
    {
	int x, y, z;
    };
    
    struct CollisionPointOrder
    {
	bool operator()(const CollisionPoint &a, const CollisionPoint &b)
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
    
    struct BoxInfo
    {    
	double dimensionX, dimensionY, dimensionZ;
	double originX, originY, originZ;
	double sensorX, sensorY, sensorZ;
	double resolution;
	int sx, sy, sz;
	int minX, minY, minZ;
	int maxX, maxY, maxZ;
	double real_minX, real_minY, real_minZ;
	double real_maxX, real_maxY, real_maxZ;
    };
    
    typedef std::set<CollisionPoint, CollisionPointOrder> CMap;
    
    void cloudCallback(const robot_msgs::PointCloudConstPtr &cloud)
    {
	std::vector<bool> mask;
	robot_msgs::PointCloud out;
	ros::WallTime tm = ros::WallTime::now();

#pragma omp parallel sections
	{

#pragma omp section
	    {
		// transform the pointcloud to the robot frame
		// since we need the points in this frame (around the robot)
		// to compute the collision map
		tf_.transformPointCloud(robotFrame_, *cloud, out);
	    }
	    
#pragma omp section
	    {
		// separate the received points into ones on the robot and ones that are obstacles
		// the frame of the cloud does not matter here
		sf_.mask(*cloud, mask);	
	    }
	}
	
	CMap obstacles;
	CMap self;

	// compute collision maps for the points on the robot and points that define obstacles
#pragma omp parallel sections
	{
	    
#pragma omp section
	    {
		constructCollisionMap(out, mask, true,  obstacles);
	    }
#pragma omp section
	    {
		constructCollisionMap(out, mask, false, self);
	    }
	}
	
	// try to transform the previous map (if it exists) to the new frame
	if (!currentMap_.empty())
	    if (!transformMap(currentMap_, header_, out.header))
		currentMap_.clear();
	header_ = out.header;
	
	if (currentMap_.empty())
	{
	    // if we have no previous information, the map is simply what we see + we assume space hidden by self is obstructed
	    CMap occ_self;
	    findSelfOcclusion(self, occ_self);
	    std::set_union(obstacles.begin(), obstacles.end(), 
			   occ_self.begin(), occ_self.end(),
			   std::inserter(currentMap_, currentMap_.begin()),
			   CollisionPointOrder());
	}
	else
	{
	    CMap occ_self;
	    CMap diff;

#pragma omp parallel sections
	    {
#pragma omp section
		{
		    // find the set of points under the parts seen by the sensor
		    findSelfOcclusion(self, occ_self);
		}
#pragma omp section
		{
		    // find the new obstacles that could be occluding information
		    std::set_difference(obstacles.begin(), obstacles.end(), 
					currentMap_.begin(), currentMap_.end(),
					std::inserter(diff, diff.begin()),
					CollisionPointOrder());
		}
	    }
	    
	    // this is the set of points that could be occluding information
	    CMap occ;	    
	    std::set_union(diff.begin(), diff.end(), 
			   occ_self.begin(), occ_self.end(),
			   std::inserter(occ, occ.begin()),
			   CollisionPointOrder());
	    
	    // find the points in the previous map that are now occluded
	    CMap keep;
	    findOcclusionsInMap(currentMap_, occ, keep);
	    
	    // the new map is the new set of obstacles + occluded information
	    currentMap_.clear();
	    std::set_union(obstacles.begin(), obstacles.end(), 
			   keep.begin(), keep.end(),
			   std::inserter(currentMap_, currentMap_.begin()),
			   CollisionPointOrder()); 
	}
	
	double sec = (ros::WallTime::now() - tm).toSec();
	
	ROS_INFO("Updated collision map with %d points at %f Hz", currentMap_.size(), 1.0/sec);
	
	publishCollisionMap(currentMap_);
    }

    bool transformMap(CMap &map, const roslib::Header &from, const roslib::Header &to)
    {
	return true;
	
	if (tf_.canTransform(to.frame_id, to.stamp, from.frame_id, from.stamp, fixedFrame_))
	{
	    tf::Stamped<btTransform> transf;
	    tf_.lookupTransform(to.frame_id, to.stamp, from.frame_id, from.stamp, fixedFrame_, transf);

	    // copy data to temporary location
	    const int n = map.size();
	    std::vector<CollisionPoint> pts(n);
	    std::copy(map.begin(), map.end(), pts.begin());
	    map.clear();
	    
#pragma omp parallel for
	    for (int i = 0 ; i < n ; ++i)
	    {
		btVector3 p(pts[i].x * bi_.resolution + bi_.originX, pts[i].y * bi_.resolution + bi_.originY, pts[i].z * bi_.resolution + bi_.originZ);
		p = transf * p;
		if (p.x() > bi_.real_minX && p.x() < bi_.real_maxX && p.y() > bi_.real_minY && p.y() < bi_.real_maxY && p.z() > bi_.real_minZ && p.z() < bi_.real_maxZ)
		{
		    CollisionPoint c;
		    c.x = (int)(0.5 + (p.x() - bi_.originX) / bi_.resolution);
		    c.y = (int)(0.5 + (p.y() - bi_.originY) / bi_.resolution);
		    c.z = (int)(0.5 + (p.z() - bi_.originZ) / bi_.resolution);
		    map.insert(c);
		}
	    }
	    
	    return true;
	}
	else
	{
	    ROS_WARN("Unable to transform previous collision map into new frame");
	    return false;
	}
    }
    
    void findOcclusionsInMap(CMap &previous, const CMap &occ, CMap &keep)
    {
	// OpenMP need an int as the lookup variable, but for set,
	// this is not possible, so we copy to a vector
	const int n = occ.size();
	std::vector<CollisionPoint> pts(n);
	std::copy(occ.begin(), occ.end(), pts.begin());

	// we are doing OpenMP parallelism, but we use a mutex to synchronize
	boost::mutex lock;
	
#pragma omp parallel for schedule(dynamic)
	for (int i = 0 ; i < n ; ++i)
	    // run this function in a visitor pattern (it calls the callback at every cell it finds along the line)
	    bresenham_line_3D(bi_.sx, bi_.sy, bi_.sz, pts[i].x, pts[i].y, pts[i].z,
			      bi_.minX, bi_.minY, bi_.minZ, bi_.maxX, bi_.maxY, bi_.maxZ,
			      boost::bind(&CollisionMapperOcc::findOcclusionsInMapAux, this, &previous, &keep, &lock, _1, _2, _3));
    }
    
    bool findOcclusionsInMapAux(CMap *previous, CMap *keep, boost::mutex *lock, int x, int y, int z)
    {
	CollisionPoint p;
	p.x = x;
	p.y = y;
	p.z = z;

	// if we are occluding this point, remember it
	if (previous->find(p) != previous->end())
	{
	    lock->lock();
	    keep->insert(p);
	    lock->unlock();
	    // we can terminate
	    return true;
	}
	
	return true;
    }
    
    void findSelfOcclusion(const CMap &self, CMap &occ)
    {
	// tell the self filter the frame in which we call getMask()
	sf_.assumeFrame(header_);
	
	// we are doing OpenMP parallelism, but we use a mutex to synchronize
	boost::mutex lock;

	// OpenMP need an int as the lookup variable, but for set,
	// this is not possible, so we copy to a vector
	const int n = self.size();
	std::vector<CollisionPoint> pts(n);
	std::copy(self.begin(), self.end(), pts.begin());
	
#pragma omp parallel for schedule(dynamic)
	for (int i = 0 ; i < n ; ++i)
	{
	    int state = 0;
	    // run this function in a visitor pattern (it calls the callback at every cell it finds along the line)
	    bresenham_line_3D(bi_.sx, bi_.sy, bi_.sz, pts[i].x, pts[i].y, pts[i].z,
			      bi_.minX, bi_.minY, bi_.minZ, bi_.maxX, bi_.maxY, bi_.maxZ,
			      boost::bind(&CollisionMapperOcc::findSelfOcclusionAux, this, &occ, &state, &lock, _1, _2, _3));
	}
    }
    
    bool findSelfOcclusionAux(CMap *occ, int *state, boost::mutex *lock, int x, int y, int z)
    {
	// this is the point in the robotFrame_
	bool out = sf_.getMask(x * bi_.resolution + bi_.originX, y * bi_.resolution + bi_.originY, z * bi_.resolution + bi_.originZ);
	
	// if we are already inside the robot, we mark the fact we want to stop when we are outside
	if (out == false)
	{
	    *state = 8;
	    return false;
	}
	else
	{
	    // if we are now outside, but have seen the inside, we have a point we need to add to the collision map
	    if (*state == 8)
	    {
		CollisionPoint c;
		c.x = x;
		c.y = y;
		c.z = z;
		lock->lock();
		occ->insert(c);
		lock->unlock();
		return true;
	    }
	    else
		// if we have not seen the inside, but we are outside, it could be we are just above the obstacle,
		// so we check the next cell along the line as well
		if (*state == 0)
		{
		    *state = 1;
		    return false;		
		}
		else
		    // at this point, the ray is probably barely touching the padding of the arm, so it is safe to ignore
		    return true;
	}
    }
    
    /** Construct an axis-aligned collision map from a point cloud assumed to be in the robot frame */
    void constructCollisionMap(const robot_msgs::PointCloud &cloud, const std::vector<bool> &mask, bool keep, CMap &map)
    {
	const unsigned int n = cloud.pts.size();
	CollisionPoint c;
	
	for (unsigned int i = 0 ; i < n ; ++i)
	    if (mask[i] == keep)
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
    
    
    /* Based on http://www.cit.gu.edu.au/~anthony/info/graphics/bresenham.procs */
    /* Form the line (x1, y1, z1) -> (x2, y2, z2) and generate points on it starting at (x2, y2, z2)
     * until it reaches a boundary of the box (minX, minY, minZ) -> (maxX, maxY, maxZ).
     * If the callback returns true, the segment is stopped */
    void bresenham_line_3D(int x1, int y1, int z1, int x2, int y2, int z2,
			   int minX, int minY, int minZ, int maxX, int maxY, int maxZ,
			   const boost::function<bool(int, int, int)> &callback) const
    {
	int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
	int pixel[3];
	
	pixel[0] = x2;
	pixel[1] = y2;
	pixel[2] = z2;
	dx = x2 - x1;
	dy = y2 - y1;
	dz = z2 - z1;
	
	int c = -1;
	if (dx != 0)
	{
	    int cx = (maxX - x1) / dx;
	    if (cx <= 0)
		cx = (minX - x1) / dx;
	    if (cx < c || c < 0)
		c = cx;
	}

	if (dy != 0)
	{
	    int cy = (maxY - y1) / dy;
	    if (cy <= 0)
		cy = (minY - y1) / dy;
	    if (cy < c || c < 0)
		c = cy;
	}
	
	if (dz != 0)
	{
	    int cz = (maxZ - z1) / dz;
	    if (cz <= 0)
		cz = (minZ - z1) / dz;
	    if (cz < c || c < 0)
		c = cz;
	}

	if (c > 0)
	{
	    dx *= c;
	    dy *= c;
	    dz *= c;
	}
	
	x_inc = (dx < 0) ? -1 : 1;
	l = abs(dx);
	y_inc = (dy < 0) ? -1 : 1;
	m = abs(dy);
	z_inc = (dz < 0) ? -1 : 1;
	n = abs(dz);
	dx2 = l << 1;
	dy2 = m << 1;
	dz2 = n << 1;
	
	if ((l >= m) && (l >= n)) {
	    err_1 = dy2 - l;
	    err_2 = dz2 - l;
	    for (i = 0; i < l; i++) {
		if (callback(pixel[0], pixel[1], pixel[2]))
		    return;
		if (err_1 > 0) {
		    pixel[1] += y_inc;
		    err_1 -= dx2;
		}
		if (err_2 > 0) {
		    pixel[2] += z_inc;
		    err_2 -= dx2;
		}
		err_1 += dy2;
		err_2 += dz2;
		pixel[0] += x_inc;
	    }
	} else if ((m >= l) && (m >= n)) {
	    err_1 = dx2 - m;
	    err_2 = dz2 - m;
	    for (i = 0; i < m; i++) {
		if (callback(pixel[0], pixel[1], pixel[2]))
		    return;
		if (err_1 > 0) {
		    pixel[0] += x_inc;
		    err_1 -= dy2;
		}
		if (err_2 > 0) {
		    pixel[2] += z_inc;
		    err_2 -= dy2;
		}
		err_1 += dx2;
		err_2 += dz2;
		pixel[1] += y_inc;
	    }
	} else {
	    err_1 = dy2 - n;
	    err_2 = dx2 - n;
	    for (i = 0; i < n; i++) {
		if (callback(pixel[0], pixel[1], pixel[2]))
		    return;
		if (err_1 > 0) {
		    pixel[1] += y_inc;
		    err_1 -= dz2;
		}
		if (err_2 > 0) {
		    pixel[0] += x_inc;
		    err_2 -= dz2;
		}
		err_1 += dy2;
		err_2 += dx2;
		pixel[2] += z_inc;
	    }
	}
	callback(pixel[0], pixel[1], pixel[2]);
    }
    
    void publishCollisionMap(const CMap &map) const
    {
	robot_msgs::CollisionMap cmap;
	cmap.header = header_;
	const unsigned int ms = map.size();
	
	for (CMap::const_iterator it = map.begin() ; it != map.end() ; ++it)
	{
	    const CollisionPoint &cp = *it;
	    robot_msgs::OrientedBoundingBox box;
	    box.extents.x = box.extents.y = box.extents.z = bi_.resolution;
	    box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
	    box.angle = 0.0;
	    box.center.x = cp.x * bi_.resolution + bi_.originX;
	    box.center.y = cp.y * bi_.resolution + bi_.originY;
	    box.center.z = cp.z * bi_.resolution + bi_.originZ;
	    cmap.boxes.push_back(box);
	}
	cmapPublisher_.publish(cmap);
	
	ROS_DEBUG("Published collision map with %u boxes", ms);
    }

    tf::TransformListener                       tf_;
    robot_self_filter::SelfMask                 sf_;
    tf::MessageNotifier<robot_msgs::PointCloud> mn_;
    ros::NodeHandle                             nh_;
    ros::Publisher                              cmapPublisher_;
    roslib::Header                              header_;
    
    CMap                                        currentMap_;
    BoxInfo                                     bi_;
    std::string                                 fixedFrame_;
    std::string                                 robotFrame_;
    
};  

int main (int argc, char** argv)
{
    ros::init(argc, argv, "collision_map_occ");
    
    CollisionMapperOcc cm;

    ros::spin();
    
    return 0;
}


