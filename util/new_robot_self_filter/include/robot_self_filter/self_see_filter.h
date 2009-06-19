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

#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <filters/filter_base.h>
#include <ros/console.h>
#include <robot_msgs/PointCloud.h>
#include <planning_environment/robot_models.h>
#include <collision_space/bodies.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <algorithm>
#include <climits>

namespace filters
{

/** \brief A filter to remove parts of the robot seen in a pointcloud
 *
 */

template <typename T>
class SelfFilter: public FilterBase <T>
{

protected:
    
    struct SeeLink
    {
	std::string                    name;
	collision_space::bodies::Body* body;
	btTransform                    constTransf;
    };
    
    struct SortBodies
    {
	bool operator()(const SeeLink &b1, const SeeLink &b2)
	{
	    return b1.body->computeVolume() > b2.body->computeVolume();
	}
    };
    
public:

    /** \brief Construct the filter */
    SelfFilter(void) : rm_("robot_description")
    {
	myTf_.reset(new tf::TransformListener());
	tf_ = myTf_.get();
    }
    
    SelfFilter(tf::TransformListener *tf) : rm_("robot_description")
    {	
	tf_ = tf;
    }
    
    /** \brief Destructor to clean up
     */
    ~SelfFilter(void)
    {
	for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
	    if (bodies_[i].body)
		delete bodies_[i].body;
    }
    
    virtual bool configure(void)
    {
	// keep only the points that are outside of the robot
	// for testing purposes this may be changed to true
	nh_.param("~invert", invert_, false);
	
	invert_ = false;
	
	if (invert_)
	    ROS_INFO("Inverting filter output");
	
	nh_.param("~accurate_timing", accurate_, true);
	
	accurate_ = false;
	
	if (accurate_)
	    ROS_INFO("Using accurate timing");
	else
	    ROS_INFO("Using simple timing");

	std::vector<std::string> links = rm_.getSelfSeeLinks();
	double scale = rm_.getSelfSeeScale();
	double padd  = rm_.getSelfSeePadding();
	
	scale = 1.0;
	padd = 0.0;
	
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
	    ROS_INFO("Self see link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].body->computeVolume());
	
	return true;
    }
    
    
    /** \brief Update the filter and return the data seperately
     * \param data_in T array with length width
     * \param data_out T array with length width
     */
    virtual bool update(const robot_msgs::PointCloud& data_in, robot_msgs::PointCloud& data_out)
    {
	if (bodies_.empty())
	    data_out = data_in;
	else
	{
	    std::vector<bool> keep(data_in.pts.size());
	    
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
		{
		    ROS_WARN("'stamps' channel not available; pruning with simple timing");
		    pruneSimple(data_in, keep);
		}
		else
		{
		    ROS_ASSERT(data_in.chan[chan].vals.size() == data_in.pts.size());
		    pruneAccurate(data_in, data_in.chan[chan], keep);
		}
	    }
	    else
		pruneSimple(data_in, keep);
	    fillResult(data_in, keep, data_out);
	}
	return true;
    }

    void identityPoses(void)
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
    
    void computeBoundingSpheres(void)
    {
	const unsigned int bs = bodies_.size();
	for (unsigned int i = 0 ; i < bs ; ++i)
	{
	    bodies_[i].body->computeBoundingSphere(bspheres_[i]);
	    bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
	}
    }
    
    void pruneAccurate(const robot_msgs::PointCloud& data_in, const robot_msgs::ChannelFloat32& times, std::vector<bool> &keep)
    {
	float maxT = *std::max_element(times.vals.begin(), times.vals.end());
	if (maxT <= FLT_MIN)
	{
	    ROS_WARN("'stamp' channel is bogus");
	    pruneSimple(data_in, keep);
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
	    if (tf_->canTransform(data_in.header.frame_id, bodies_[i].name, timeStart))
		tf_->lookupTransform(data_in.header.frame_id, bodies_[i].name, timeStart, transf);
	    else
	    {
		transf.setIdentity();
		ROS_ERROR("Unable to lookup transform from '%s' to '%s' at start time", data_in.header.frame_id.c_str(), bodies_[i].name.c_str());
	    }
	    quats[i].first = transf.getRotation();
	    origs[i].first = transf.getOrigin();
	    
	    if (tf_->canTransform(bodies_[i].name, data_in.header.frame_id, timeEnd))
		tf_->lookupTransform(bodies_[i].name, data_in.header.frame_id, timeEnd, transf);
	    else
	    {
		transf.setIdentity();
		ROS_ERROR("Unable to lookup transform from '%s' to '%s' at end time", data_in.header.frame_id.c_str(), bodies_[i].name.c_str());
	    }
	    quats[i].second = transf.getRotation();
	    origs[i].first = transf.getOrigin();
	}
	
	// we now decide which points we keep
#pragma omp parallel for schedule(dynamic)
	for (int i = 0 ; i < (int)np ; ++i)
	{
	    // this point is the cloud's frame
	    btVector3 pt = btVector3(btScalar(data_in.pts[i].x), btScalar(data_in.pts[i].y), btScalar(data_in.pts[i].z));
	    float time01  = times.vals[i] / maxT;
	    float time01i = 1.0f - time01;
	    
	    bool out = true;
	    for (unsigned int j = 0 ; out && j < bs ; ++j)
	    {
		// find the transform to bring the point to the link frame
		btTransform t(quats[j].first.slerp(quats[j].second, time01i),
			      origs[j].first * time01i + origs[j].second * time01);
		
		out = !bodies_[j].body->containsPoint(t * pt);
	    }
	    
	    keep[i] = invert_ ? !out : out;
	}
    }
    
    void pruneSimple(const robot_msgs::PointCloud& data_in, std::vector<bool> &keep)
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
	    if (tf_->canTransform(data_in.header.frame_id, bodies_[i].name, data_in.header.stamp))
		tf_->lookupTransform(data_in.header.frame_id, bodies_[i].name, data_in.header.stamp, transf);
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
		    if (bspheres_[j].center.distance2(pt) < bspheresRadius2_[j])
			out = !bodies_[j].body->containsPoint(pt);
	    
	    keep[i] = invert_ ? !out : out;
	}
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
    
    virtual bool update(const std::vector<robot_msgs::PointCloud> & data_in, std::vector<robot_msgs::PointCloud>& data_out)
    {
	bool result = true;
	data_out.resize(data_in.size());
	for (unsigned int i = 0 ; i < data_in.size() ; ++i)
	    if (!update(data_in[i], data_out[i]))
		result = false;
	return true;
    }
    
protected:
    
    planning_environment::RobotModels                    rm_;
    tf::TransformListener                               *tf_;
    ros::NodeHandle                                      nh_;
    
    std::vector<SeeLink>                                 bodies_;
    std::vector<double>                                  bspheresRadius2_;
    std::vector<collision_space::bodies::BoundingSphere> bspheres_;
    bool                                                 bodiesAtIdentity_;

    bool                                                 accurate_;
    bool                                                 invert_;
    
private:

    boost::shared_ptr<tf::TransformListener> myTf_;
    
};
    
    typedef robot_msgs::PointCloud robot_msgs_PointCloud;
    FILTERS_REGISTER_FILTER(SelfFilter, robot_msgs_PointCloud);
    
}

#endif //#ifndef FILTERS_SELF_SEE_H_
