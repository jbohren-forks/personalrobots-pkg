/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_self_filter/self_mask.h"
#include <algorithm>
#include <climits>
#include <ros/console.h>

void robot_self_filter::SelfMask::freeMemory(void)
{
    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	if (bodies_[i].body)
	    delete bodies_[i].body;
    bodies_.clear();
}

bool robot_self_filter::SelfMask::configure(const std::vector<std::string> &links, double scale, double padd)
{
    // in case configure was called before, we free the memory
    freeMemory();
    sensor_pos_.setValue(0, 0, 0);
    
    // from the geometric model, find the shape of each link of interest
    // and create a body from it, one that knows about poses and can 
    // check for point inclusion
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	SeeLink sl;
	planning_models::KinematicModel::Link *link = rm_.getKinematicModel()->getLink(links[i]);
	sl.body = bodies::createBodyFromShape(link->shape);

	if (sl.body)
	{
	    sl.name = links[i];
	    
	    // collision models may have an offset, in addition to what TF gives
	    // so we keep it around
	    sl.constTransf = link->constGeomTrans;
	    sl.body->setScale(scale);
	    sl.body->setPadding(padd);
	    bodies_.push_back(sl);
	}
	else
	    ROS_WARN("Unable to create point inclusion body for link '%s'", links[i].c_str());
    }
    
    if (bodies_.empty())
	ROS_WARN("No robot links will be checked for self collision");
    
    // put larger volume bodies first -- higher chances of containing a point
    std::sort(bodies_.begin(), bodies_.end(), SortBodies());
    
    bspheres_.resize(bodies_.size());
    bspheresRadius2_.resize(bodies_.size());

    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	ROS_DEBUG("Self mask includes link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].body->computeVolume());
    
    ROS_INFO("Self filter using %f padding and %f scaling", padd, scale);

    return true; 
}

bool robot_self_filter::SelfMask::configure(double scale, double padd)
{
    return configure(rm_.getSelfSeeLinks(), scale, padd);
}

bool robot_self_filter::SelfMask::configure(void)
{
    return configure(rm_.getSelfSeeLinks(), rm_.getSelfSeeScale(), rm_.getSelfSeePadding());
}

void robot_self_filter::SelfMask::getLinkNames(std::vector<std::string> &frames) const
{
    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	frames.push_back(bodies_[i].name);
}

void robot_self_filter::SelfMask::maskContainment(const robot_msgs::PointCloud& data_in, std::vector<int> &mask)
{
    mask.resize(data_in.pts.size());
    if (bodies_.empty())
	std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    else
    {
	assumeFrame(data_in.header);
	maskAuxContainment(data_in, mask);
    }
}

void robot_self_filter::SelfMask::maskIntersection(const robot_msgs::PointCloud& data_in, const std::string &sensor_frame, std::vector<int> &mask)
{
    mask.resize(data_in.pts.size());
    if (bodies_.empty())
	std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    else
    {
	assumeFrame(data_in.header, sensor_frame);
	if (sensor_frame.empty())
	    maskAuxContainment(data_in, mask);
	else
	    maskAuxIntersection(data_in, mask);
    }
}

void robot_self_filter::SelfMask::maskIntersection(const robot_msgs::PointCloud& data_in, const btVector3 &sensor, std::vector<int> &mask)
{
    mask.resize(data_in.pts.size());
    if (bodies_.empty())
	std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    else
    {
	assumeFrame(data_in.header, sensor);
	maskAuxIntersection(data_in, mask);
    }
}

void robot_self_filter::SelfMask::computeBoundingSpheres(void)
{
    const unsigned int bs = bodies_.size();
    for (unsigned int i = 0 ; i < bs ; ++i)
    {
	bodies_[i].body->computeBoundingSphere(bspheres_[i]);
	bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
    }
}

void robot_self_filter::SelfMask::assumeFrame(const roslib::Header& header, const btVector3 &sensor)
{
    assumeFrame(header);
    sensor_pos_ = sensor;
}

void robot_self_filter::SelfMask::assumeFrame(const roslib::Header& header, const std::string &sensor_frame)
{
    const unsigned int bs = bodies_.size();
    
    // place the links in the assumed frame 
    for (unsigned int i = 0 ; i < bs ; ++i)
    {
	// find the transform between the link's frame and the pointcloud frame
	tf::Stamped<btTransform> transf;
	if (tf_.canTransform(header.frame_id, bodies_[i].name, header.stamp))
	    tf_.lookupTransform(header.frame_id, bodies_[i].name, header.stamp, transf);
	else
	{
	    transf.setIdentity();
	    ROS_ERROR("Unable to lookup transform from %s to %s", bodies_[i].name.c_str(), header.frame_id.c_str());
	}
	
	// set it for each body; we also include the offset specified in URDF
	bodies_[i].body->setPose(transf * bodies_[i].constTransf);
    }
    
    computeBoundingSpheres();

    // compute the origin of the sensor in the frame of the cloud
    if (!sensor_frame.empty())
    {
	if (tf_.canTransform(header.frame_id, sensor_frame, header.stamp))
	{
	    tf::Stamped<btTransform> transf;
	    tf_.lookupTransform(header.frame_id, sensor_frame, header.stamp, transf);
	    sensor_pos_ = transf.getOrigin();
	}
	else
	{
	    sensor_pos_.setValue(0, 0, 0);
	    ROS_ERROR("Unable to lookup transform from %s to %s", sensor_frame.c_str(), header.frame_id.c_str());
	}
    }
}

void robot_self_filter::SelfMask::maskAuxContainment(const robot_msgs::PointCloud& data_in, std::vector<int> &mask)
{
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.pts.size();
    
    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);	  
    btScalar radiusSquared = bound.radius * bound.radius;
    
    // we now decide which points we keep
#pragma omp parallel for schedule(dynamic) 
    for (int i = 0 ; i < (int)np ; ++i)
    {
	btVector3 pt = btVector3(data_in.pts[i].x, data_in.pts[i].y, data_in.pts[i].z);
	int out = OUTSIDE;
	if (bound.center.distance2(pt) < radiusSquared)
	    for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
		if (bodies_[j].body->containsPoint(pt))
		    out = INSIDE;
	
	mask[i] = out;
    }
}

void robot_self_filter::SelfMask::maskAuxIntersection(const robot_msgs::PointCloud& data_in, std::vector<int> &mask)
{
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.pts.size();
    
    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);	  
    btScalar radiusSquared = bound.radius * bound.radius;
    
    // we now decide which points we keep
#pragma omp parallel for schedule(dynamic) 
    for (int i = 0 ; i < (int)np ; ++i)
    {
	btVector3 pt = btVector3(data_in.pts[i].x, data_in.pts[i].y, data_in.pts[i].z);
	int out = OUTSIDE;
	if (bound.center.distance2(pt) < radiusSquared)
	    for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
		if (bodies_[j].body->containsPoint(pt))
		    out = INSIDE; 
	if (out == OUTSIDE)
	{
	    btVector3 dir(sensor_pos_ - pt);
	    dir.normalize();
	    for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
		if (bodies_[j].body->intersectsRay(pt, dir))
		    out = SHADOW;
	}
	
	mask[i] = out;
    }
}

int robot_self_filter::SelfMask::getMaskContainment(const btVector3 &pt) const
{
    const unsigned int bs = bodies_.size();
    int out = OUTSIDE;
    for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
	if (bodies_[j].body->containsPoint(pt))
	    out = INSIDE;
    return out;
}

int robot_self_filter::SelfMask::getMaskContainment(double x, double y, double z) const
{
    return getMaskContainment(btVector3(x, y, z));
}

int robot_self_filter::SelfMask::getMaskIntersection(const btVector3 &pt) const
{  
    int out = getMaskContainment(pt);
    if (out == OUTSIDE)
    {
	const unsigned int bs = bodies_.size();
	btVector3 dir(sensor_pos_ - pt);
	dir.normalize();
	for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
	    if (bodies_[j].body->intersectsRay(pt, dir))
		out = SHADOW;
    }
    return out;
}

int robot_self_filter::SelfMask::getMaskIntersection(double x, double y, double z) const
{
    return getMaskIntersection(btVector3(x, y, z));
}
