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

bool robot_self_filter::SelfMask::configure(void)
{
    std::vector<std::string> links = rm_.getSelfSeeLinks();
    double scale = rm_.getSelfSeeScale();
    double padd  = rm_.getSelfSeePadding();
    
    // from the geometric model, find the shape of each link of interest
    // and create a body from it, one that knows about poses and can 
    // check for point inclusion
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	SeeLink sl;
	sl.body = bodies::createBodyFromShape(rm_.getKinematicModel()->getLink(links[i])->shape);
	if (sl.body)
	{
	    sl.name = links[i];
	    
	    // collision models may have an offset, in addition to what TF gives
	    // so we keep it around
	    sl.constTransf = rm_.getKinematicModel()->getLink(links[i])->constGeomTrans;
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
	ROS_INFO("Self mask includes link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].body->computeVolume());
    
    return true;
}

void robot_self_filter::SelfMask::getLinkFrames(std::vector<std::string> &frames) const
{
    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	frames.push_back(bodies_[i].name);
}

void robot_self_filter::SelfMask::mask(const robot_msgs::PointCloud& data_in, std::vector<bool> &mask)
{
    mask.resize(data_in.pts.size());
    if (bodies_.empty())
	std::fill(mask.begin(), mask.end(), true);
    else
	maskSimple(data_in, mask);
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

void robot_self_filter::SelfMask::assumeFrame(const roslib::Header& header)
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
}

void robot_self_filter::SelfMask::maskSimple(const robot_msgs::PointCloud& data_in, std::vector<bool> &mask)
{
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.pts.size();
    
    assumeFrame(data_in.header);
    
    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);	  
    btScalar radiusSquared = bound.radius * bound.radius;
    
    // we now decide which points we keep
#pragma omp parallel for schedule(dynamic)
    for (int i = 0 ; i < (int)np ; ++i)
    {
	btVector3 pt = btVector3(btScalar(data_in.pts[i].x), btScalar(data_in.pts[i].y), btScalar(data_in.pts[i].z));
	bool out = true;
	if (bound.center.distance2(pt) < radiusSquared)
	    for (unsigned int j = 0 ; out && j < bs ; ++j)
		out = !bodies_[j].body->containsPoint(pt);
	
	mask[i] = out;
    }
}

bool robot_self_filter::SelfMask::getMask(double x, double y, double z) const
{
    btVector3 pt = btVector3(btScalar(x), btScalar(y), btScalar(z));
    const unsigned int bs = bodies_.size();
    bool out = true;
    for (unsigned int j = 0 ; out && j < bs ; ++j)
	out = !bodies_[j].body->containsPoint(pt);
    return out;
}
