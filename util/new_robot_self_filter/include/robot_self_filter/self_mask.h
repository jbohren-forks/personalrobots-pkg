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

#ifndef ROBOT_SELF_FILTER_SELF_MASK_
#define ROBOT_SELF_FILTER_SELF_MASK_

#include <robot_msgs/PointCloud.h>
#include <planning_environment/robot_models.h>
#include <collision_space/bodies.h>
#include <tf/transform_listener.h>
#include <string>

namespace robot_self_filter
{

/** \brief Computing a mask for a pointcloud that states which points are inside the robot
 *
 */

    class SelfMask
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
	SelfMask(tf::TransformListener &tf) : rm_("robot_description"), tf_(tf)
	{
	}
	
	/** \brief Destructor to clean up
	 */
	~SelfMask(void)
	{
	    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
		if (bodies_[i].body)
		    delete bodies_[i].body;
	}
	
	bool configure(bool accurate);
	
	/** \brief Compute the mask for a given pointcloud. If a mask element is true, the point
	    is outside the robot
	 */
	void mask(const robot_msgs::PointCloud& data_in, std::vector<bool> &mask);
	

    private:

	void identityPoses(void);
	void computeBoundingSpheres(void);
	
	void maskAccurate(const robot_msgs::PointCloud& data_in, const robot_msgs::ChannelFloat32& times, std::vector<bool> &mask);
	void maskSimple(const robot_msgs::PointCloud& data_in, std::vector<bool> &mask);
	
	planning_environment::RobotModels                    rm_;
	tf::TransformListener                               &tf_;
	ros::NodeHandle                                      nh_;
	
	std::vector<SeeLink>                                 bodies_;
	std::vector<double>                                  bspheresRadius2_;
	std::vector<collision_space::bodies::BoundingSphere> bspheres_;
	bool                                                 bodiesAtIdentity_;
	
	bool                                                 accurate_;
	
    };
    
}

#endif
