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

bool robot_self_filter::SelfMask::configure(bool accurate)
{
    accurate_ = accurate;
    
    std::vector<std::string> links = rm_.getSelfSeeLinks();
    double scale = rm_.getSelfSeeScale();
    double padd  = rm_.getSelfSeePadding();
    
    // from the geometric model, find the shape of each link of interest
    // and create a body from it, one that knows about poses and can 
    // check for point inclusion
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	SeeLink sl;
	sl.body = collision_space::bodies::createBodyFromShape(rm_.getKinematicModel()->getLink(links[i])->shape);
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
    bodiesAtIdentity_ = false;
    
    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	ROS_INFO("Self mask includes link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].body->computeVolume());
    
    return true;
}


/** \brief Compute the mask for a given pointcloud. If a mask element is true, the point
    is outside the robot
*/
void robot_self_filter::SelfMask::mask(const robot_msgs::PointCloud& data_in, std::vector<bool> &mask)
{
    mask.resize(data_in.pts.size());
    if (bodies_.empty())
	std::fill(mask.begin(), mask.end(), true);
    else
    {
	if (accurate_)
	{
	    int chan = -1;
	    for (unsigned int i = 0 ; i < data_in.chan.size() ; ++i)
		if (data_in.chan[i].name == "stamps")
		{
		    chan = i;
		    break;
		}
	    if (chan < 0)
		maskSimple(data_in, mask);
	    else
	    {
		ROS_ASSERT(data_in.chan[chan].vals.size() == data_in.pts.size());
		maskAccurate(data_in, data_in.chan[chan], mask);
	    }
	}
	else
	    maskSimple(data_in, mask);
    }
}

void robot_self_filter::SelfMask::identityPoses(void)
{
    if (!bodiesAtIdentity_)
    {
	// put all links at origin & lookup the needed transforms
	const unsigned int bs = bodies_.size();
	for (unsigned int i = 0 ; i < bs ; ++i)
	    bodies_[i].body->setPose(bodies_[i].constTransf);
	computeBoundingSpheres();
	bodiesAtIdentity_ = true;
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

void robot_self_filter::SelfMask::maskAccurate(const robot_msgs::PointCloud& data_in, const robot_msgs::ChannelFloat32& times, std::vector<bool> &mask)
{
    float maxT = *std::max_element(times.vals.begin(), times.vals.end());
    if (maxT <= FLT_MIN)
    {
	ROS_WARN("'stamps' channel contains invalid data");
	maskSimple(data_in, mask);
	return;
    }
    
    // put all links at origin
    identityPoses();
    
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.pts.size();
    
    ros::Time timeStart = data_in.header.stamp;
    ros::Time timeEnd = timeStart + ros::Duration(maxT);
    std::vector< std::pair< btVector3, btVector3 > >       origs(bs);
    std::vector< std::pair< btQuaternion, btQuaternion > > quats(bs);
    tf::Stamped<btTransform> transf;
    
    // lookup the needed transforms
    for (unsigned int i = 0 ; i < bs ; ++i)
    {
	if (tf_.canTransform(bodies_[i].name, data_in.header.frame_id, timeStart))
	    tf_.lookupTransform(bodies_[i].name, data_in.header.frame_id, timeStart, transf);
	else
	{
	    transf.setIdentity();
	    ROS_ERROR("Unable to lookup transform from '%s' to '%s' at start time", data_in.header.frame_id.c_str(), bodies_[i].name.c_str());
	}
	quats[i].first = transf.getRotation();
	origs[i].first = transf.getOrigin();
	
	if (tf_.canTransform(bodies_[i].name, data_in.header.frame_id, timeEnd))
	    tf_.lookupTransform(bodies_[i].name, data_in.header.frame_id, timeEnd, transf);
	else
	{
	    transf.setIdentity();
	    ROS_ERROR("Unable to lookup transform from '%s' to '%s' at end time", data_in.header.frame_id.c_str(), bodies_[i].name.c_str());
	}
	quats[i].second = transf.getRotation();
	origs[i].second = transf.getOrigin();
    }
    
    // we now decide which points we keep
#pragma omp parallel for schedule(dynamic)
    for (int i = 0 ; i < (int)np ; ++i)
    {
	// this point is the cloud's frame
	btVector3 pt = btVector3(btScalar(data_in.pts[i].x), btScalar(data_in.pts[i].y), btScalar(data_in.pts[i].z));
	btScalar time01  = times.vals[i] / maxT;
	btScalar time01i = 1.0 - time01;
	
	bool out = true;
	for (unsigned int j = 0 ; out && j < bs ; ++j)
	{
	    // find the transform to bring the point to the link frame
	    btTransform t(quats[j].first.slerp(quats[j].second, time01i),
			  origs[j].first * time01i + origs[j].second * time01);
	    out = !bodies_[j].body->containsPoint(t * pt);
	}
	
	mask[i] = out;
    }
}

void robot_self_filter::SelfMask::maskSimple(const robot_msgs::PointCloud& data_in, std::vector<bool> &mask)
{
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.pts.size();
    
    // mark that links are no longer at origin
    bodiesAtIdentity_ = false;
    
    // place the links in the frame of the pointcloud
    for (unsigned int i = 0 ; i < bs ; ++i)
    {
	// find the transform between the link's frame and the pointcloud frame
	tf::Stamped<btTransform> transf;
	if (tf_.canTransform(data_in.header.frame_id, bodies_[i].name, data_in.header.stamp))
	    tf_.lookupTransform(data_in.header.frame_id, bodies_[i].name, data_in.header.stamp, transf);
	else
	{
	    transf.setIdentity();
	    ROS_ERROR("Unable to lookup transform from %s to %s", bodies_[i].name.c_str(), data_in.header.frame_id.c_str());
	}
	
	// set it for each body; we also include the offset specified in URDF
	bodies_[i].body->setPose(transf * bodies_[i].constTransf);
    }
    computeBoundingSpheres();
    
    // compute a sphere that bounds the entire robot
    collision_space::bodies::BoundingSphere bound;
    collision_space::bodies::mergeBoundingSpheres(bspheres_, bound);	  
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
