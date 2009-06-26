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
    
    CollisionMapperOcc(void) : sf_(tf_)			       
    { 
	// read ROS params
	loadParams();
	
	// advertise our topic
	cmapPublisher_ = nh_.advertise<robot_msgs::CollisionMap>("collision_map_occ", 1);

	// create a message notifier (and enable subscription)
	mn_ = new tf::MessageNotifier<robot_msgs::PointCloud>(tf_, boost::bind(&CollisionMapperOcc::cloudCallback, this, _1), "cloud_in", "", 1);
	
	// configure the self mask and the message notifier
	std::vector<std::string> frames;
	sf_.getLinkFrames(frames);
	if (std::find(frames.begin(), frames.end(), robotFrame_) == frames.end())
	    frames.push_back(robotFrame_);
	mn_->setTargetFrame(frames);
    }
    
    ~CollisionMapperOcc(void)
    {
	delete mn_;
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

    int collisionPointDistance(const CollisionPoint &a, const CollisionPoint &b) const
    {
	return std::max(abs(a.x - b.x), std::max(abs(a.y - b.y), abs(a.z - b.z)));
    }
    
    typedef std::set<CollisionPoint, CollisionPointOrder> CMap;

    // parameters & precomputed values for the box that represents the collision map
    // around the robot
    struct BoxInfo
    {    
	double dimensionX, dimensionY, dimensionZ;
	double originX, originY, originZ;
	double sensorX, sensorY, sensorZ;
	double resolution;
	int radius;
	int sx, sy, sz;
	int minX, minY, minZ;
	int maxX, maxY, maxZ;
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
	
	// sensor origin in robot frame
	nh_.param<double>("~sensor_x", bi_.sensorX, 0.05);
	nh_.param<double>("~sensor_y", bi_.sensorY, 0.0);
	nh_.param<double>("~sensor_z", bi_.sensorZ, 1.0);
	
	// resolution
	nh_.param<double>("~resolution", bi_.resolution, 0.015);

	// when occluded obstacles are raytraced, keep boxes from occluded space within a given radius
	nh_.param<int>("~radius", bi_.radius, 1);
	
	ROS_INFO("Maintaining occlusion map in frame '%s', with origin at (%f, %f, %f) and dimension (%f, %f, %f), resolution of %f; "
		 "sensor is at (%f, %f, %f), fixed fame is '%s', radius for raytraced occlusions is %d.",
		 robotFrame_.c_str(), bi_.dimensionX, bi_.dimensionY, bi_.dimensionZ, bi_.originX, bi_.originY, bi_.originZ, bi_.resolution,
		 bi_.sensorX, bi_.sensorY, bi_.sensorZ, fixedFrame_.c_str(), bi_.radius);

	nh_.param<std::string>("~cloud_annotation", cloud_annotation_, std::string());

	// compute some useful values
	bi_.sx = (int)(0.5 + (bi_.sensorX - bi_.originX) / bi_.resolution);
	bi_.sy = (int)(0.5 + (bi_.sensorY - bi_.originY) / bi_.resolution);
	bi_.sz = (int)(0.5 + (bi_.sensorZ - bi_.originZ) / bi_.resolution);

	bi_.minX = (int)(0.5 + (-bi_.dimensionX - bi_.originX) / bi_.resolution) - 1;
	bi_.maxX = (int)(0.5 + (bi_.dimensionX - bi_.originX) / bi_.resolution) + 1;

	bi_.minY = (int)(0.5 + (-bi_.dimensionY - bi_.originY) / bi_.resolution) - 1;
	bi_.maxY = (int)(0.5 + (bi_.dimensionY - bi_.originY) / bi_.resolution) + 1;
	
	bi_.minZ = (int)(0.5 + (-bi_.dimensionZ - bi_.originZ) / bi_.resolution) - 1;
	bi_.maxZ = (int)(0.5 + (bi_.dimensionZ - bi_.originZ) / bi_.resolution) + 1;

	bi_.real_minX = -bi_.dimensionX + bi_.originX;
	bi_.real_maxX =  bi_.dimensionX + bi_.originX;
	bi_.real_minY = -bi_.dimensionY + bi_.originY;
	bi_.real_maxY =  bi_.dimensionY + bi_.originY;
	bi_.real_minZ = -bi_.dimensionZ + bi_.originZ;
	bi_.real_maxZ =  bi_.dimensionZ + bi_.originZ;	
    }
    
    void computeCloudMask(const robot_msgs::PointCloud &cloud, std::vector<bool> &mask)
    {
	if (cloud_annotation_.empty())
	    sf_.mask(cloud, mask);
	else
	{
	    int c = -1;
	    for (unsigned int i = 0 ; i < cloud.chan.size() ; ++i)
		if (cloud.chan[i].name == cloud_annotation_)
		{
		    c = i;
		    break;
		}
	    if (c < 0)
	    {
		ROS_WARN("Cloud annotation channel '%s' is missing", cloud_annotation_.c_str());
		sf_.mask(cloud, mask);
	    }
	    else
	    {
		ROS_ASSERT(cloud.chan[c].vals.size() == cloud.pts.size());
		mask.resize(cloud.pts.size());
		for (unsigned int i = 0 ; i < mask.size() ; ++i)
		    mask[i] = cloud.chan[c].vals[i] > 0.0;
	    }
	}
    }
    
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
		computeCloudMask(*cloud, mask);	
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
#pragma omp section
	    {
		// try to transform the previous map (if it exists) to the new frame
		if (!currentMap_.empty())
		    if (!transformMap(currentMap_, header_, out.header))
			currentMap_.clear();
		header_ = out.header;
	    }
	}
	
	// update map
	updateMap(obstacles, self);

	double sec = (ros::WallTime::now() - tm).toSec();
	ROS_INFO("Updated collision map with %d points at %f Hz", currentMap_.size(), 1.0/sec);
	
	publishCollisionMap(currentMap_, cmapPublisher_);
    }

    void updateMap(CMap &obstacles, CMap &self)
    {
	if (currentMap_.empty())
	{
	    // if we have no previous information, the map is simply what we see + we assume space hidden by self is obstructed
	    // we ignore points that were previously occluded by robot but are now free due to motion
	    CMap occ_self;
	    CMap dummy;
	    findSelfOcclusion(self, occ_self, dummy);
	    std::set_union(obstacles.begin(), obstacles.end(), 
			   occ_self.begin(), occ_self.end(),
			   std::inserter(currentMap_, currentMap_.begin()),
			   CollisionPointOrder());
	}
	else
	{
	    CMap occ_current;
	    CMap occ_moving;
	    CMap diff;

#pragma omp parallel sections
	    {
#pragma omp section
		{
		    // find the set of points under the parts seen by the sensor
		    findSelfOcclusion(self, occ_current, occ_moving);
		}
#pragma omp section
		{
		    // find the points from the old map that are no longer visible
		    set_difference(currentMap_.begin(), currentMap_.end(), obstacles.begin(), obstacles.end(),
				   std::inserter(diff, diff.begin()), CollisionPointOrder());
		}
	    }
	    
	    // take the union of points causing self occlusion
	    CMap occ_self;
	    std::set_union(occ_current.begin(), occ_current.end(), 
			   occ_moving.begin(), occ_moving.end(),
			   std::inserter(occ_self, occ_self.begin()),
			   CollisionPointOrder());
	    
	    // this is the set of points that could be occluding information
	    CMap occ;	    
	    std::set_union(obstacles.begin(), obstacles.end(), 
			   occ_self.begin(), occ_self.end(),
			   std::inserter(occ, occ.begin()),
			   CollisionPointOrder());

	    // find the points in the previous map that are now occluded (raytracing)
	    CMap keep;
	    findOcclusionsInMap(diff, occ, keep);
	    
	    // the new map is the new set of obstacles + occluded information
	    currentMap_.clear();
	    std::set_union(obstacles.begin(), obstacles.end(), 
			   keep.begin(), keep.end(),
			   std::inserter(currentMap_, currentMap_.begin()),
			   CollisionPointOrder()); 
	}
    }
    
    bool transformMap(CMap &map, const roslib::Header &from, const roslib::Header &to)
    {
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
		    CollisionPoint c((int)(0.5 + (p.x() - bi_.originX) / bi_.resolution),
				     (int)(0.5 + (p.y() - bi_.originY) / bi_.resolution),
				     (int)(0.5 + (p.z() - bi_.originZ) / bi_.resolution));
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
    
    void findOcclusionsInMap(const CMap &possiblyOccluded, const CMap &occluding, CMap &keep)
    {
	// OpenMP need an int as the lookup variable, but for set,
	// this is not possible, so we copy to a vector
	const int n = possiblyOccluded.size();
	std::vector<CollisionPoint> pts(n);
	std::copy(possiblyOccluded.begin(), possiblyOccluded.end(), pts.begin());
	
#pragma omp parallel for schedule(dynamic)
	for (int i = 0 ; i < n ; ++i)
	{
	    // run this function in a visitor pattern (it calls the callback at every cell it finds along the line)
	    // we start at a possibly occluded point and go towards the sensor
	    // if we hit a point in the occluding set, we have an occluded point. Otherwise, the point was part of a 
	    // moving obstacle so we ignore it
	    int result = 0;
	    bresenham_line_3D(pts[i].x, pts[i].y, pts[i].z, bi_.sx, bi_.sy, bi_.sz, 
			      boost::bind(&CollisionMapperOcc::findOcclusionsInMapAux, this, &occluding, &result, _1, _2, _3));
	    if (result == 1)
	    {
#pragma omp critical
		{
		    keep.insert(pts[i]);
		}
	    }
	}
    }
    
    bool findOcclusionsInMapAux(const CMap *occluding, int *result, int x, int y, int z)
    {
	CollisionPoint p(x, y, z);
	CMap::const_iterator it = occluding->lower_bound(p);
	
	if (it != occluding->end())
	{
	    if (collisionPointDistance(*it, p) <= bi_.radius)
	    {
		// we hit a point that is occluding, our start point is occluded
		*result = 1;
		return true;
	    }
	}
	// continue looking
	return false;
    }
    
    void findSelfOcclusion(const CMap &self, CMap &occ_current, CMap &occ_moving)
    {
	// tell the self filter the frame in which we call getMask()
	sf_.assumeFrame(header_);
	
	// we are doing OpenMP parallelism, but we use a mutex to synchronize
	boost::mutex lock_current;
	boost::mutex lock_moving;

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
			      boost::bind(&CollisionMapperOcc::findSelfOcclusionAux, this, &occ_current, &occ_moving, &state, &lock_current, &lock_moving, _1, _2, _3));
	}
    }
    
    bool findSelfOcclusionAux(CMap *occ_current, CMap *occ_moving, int *state, boost::mutex *lock_current, boost::mutex *lock_moving, int x, int y, int z)
    {
	if (*state == 16)
	{
	    // here we have the option of adding further points in the occluded space
	    // but for now, we stop

	    return true;
	}
	else
	{
	    // this is the point in the robotFrame_; check if it is currently inside the robot
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
		    CollisionPoint c(x, y, z);
		    lock_current->lock();
		    occ_current->insert(c);
		    lock_current->unlock();
		    
		    // mark that we are now outside the robot and have added a first point
		    *state = 16;

		    // continue looking on the ray
		    return false;
		}
		else
		    // if we have not seen the inside, but we are outside, it could be we are just above the obstacle,
		    // so we check the next cell along the line as well
		    if (*state == 0)
		    {
			*state = 1;

			// continue further on the ray
			return false;		
		    }
		    else
		    {
			// if we get to this point, it means the point was in self collision, but it no longer is
			// so at least a part of the robot has moved; we will need to raytrace from this point later on
			// and check if we are occluding anything
			CollisionPoint c(x, y, z);
			lock_moving->lock();
			occ_moving->insert(c);
			lock_moving->unlock();

			// and we stop looking on this ray
			return true;
		    }
	    }
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
			   const boost::function<bool(int, int, int)> &callback) const
    {
	int pixel[3];
	int dx, dy, dz;
	
	pixel[0] = x1;
	pixel[1] = y1;
	pixel[2] = z1;
	
	dx = x2 - x1;
	dy = y2 - y1;
	dz = z2 - z1;
	
	bresenham_line_3D(dx, dy, dz, pixel, callback);
    }

    void bresenham_line_3D(int x1, int y1, int z1, int x2, int y2, int z2,
			   int minX, int minY, int minZ, int maxX, int maxY, int maxZ,
			   const boost::function<bool(int, int, int)> &callback) const
    {
	int pixel[3];
	int dx, dy, dz;
	
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
	
	bresenham_line_3D(dx, dy, dz, pixel, callback);
    }
    
    void bresenham_line_3D(int dx, int dy, int dz, int pixel[3], const boost::function<bool(int, int, int)> &callback) const
    {
	int i, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
	
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
    
    void publishCollisionMap(const CMap &map, ros::Publisher &pub) const
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
	pub.publish(cmap);
	
	ROS_DEBUG("Published collision map with %u boxes", ms);
    }

    tf::TransformListener                        tf_;
    robot_self_filter::SelfMask                  sf_;
    tf::MessageNotifier<robot_msgs::PointCloud> *mn_;
    ros::NodeHandle                              nh_;
    ros::Publisher                               cmapPublisher_;
    roslib::Header                               header_;
    std::string                                  cloud_annotation_;
    
    CMap                                         currentMap_;
    
    BoxInfo                                      bi_;
    std::string                                  fixedFrame_;
    std::string                                  robotFrame_;
    
};  

int main (int argc, char** argv)
{
    ros::init(argc, argv, "collision_map_occ");
    
    CollisionMapperOcc cm;

    ros::spin();
    
    return 0;
}


