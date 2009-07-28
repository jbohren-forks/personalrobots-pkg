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
#include <planning_environment/models/robot_models.h>
#include <geometric_shapes/bodies.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>

namespace robot_self_filter
{

    /** \brief The possible values of a mask computed for a point */
    enum
    {
	INSIDE = 0,
	SHADOW = -1,
	OUTSIDE = 1
    };
    

    /** \brief Computing a mask for a pointcloud that states which points are inside the robot
     *
     */
    class SelfMask
    {	
    protected:
	
	struct SeeLink
	{
	    std::string   name;
	    bodies::Body *body;
	    btTransform   constTransf;
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
	    configure();
	}

	/** \brief Construct the filter */
	SelfMask(tf::TransformListener &tf, double scale, double padd) : rm_("robot_description"), tf_(tf)
	{
	    configure(scale, padd);
	}
	
	/** \brief Construct the filter */
	SelfMask(tf::TransformListener &tf, const std::vector<std::string> &links, double scale, double padd) : rm_("robot_description"), tf_(tf)
	{
	    configure(links, scale, padd);
	}
	
	/** \brief Destructor to clean up
	 */
	~SelfMask(void)
	{
	    freeMemory();
	}
	
	/** \brief Compute the mask for a given pointcloud. If a mask element is 1, the point
	    is outside the robot. The point is outside if the mask element is 0.
	 */
	void maskContainment(const robot_msgs::PointCloud& data_in, std::vector<int> &mask);

	/** \brief Compute the mask for a given pointcloud. If a mask
	    element is 1, the point is outside the robot. If it is -1,
	    the point is on a ray behind the robot and should not have
	    been seen. If the mask element is 0, the point is inside
	    the robot.
	 */
	void maskIntersection(const robot_msgs::PointCloud& data_in, const std::string &sensor_frame, std::vector<int> &mask);
	
	/** \brief Assume subsequent calls to getMask() will be in the frame passed to this function */
	void assumeFrame(const roslib::Header& header, const std::string &sensor_frame = std::string());
	
	/** \brief Get the mask value for an individual point. No
	    setup is performed, assumeFrame() should be called before use */
	int  getMaskContainment(double x, double y, double z) const;
	
	/** \brief Get the set of frames that correspond to the links */
	void getLinkNames(std::vector<std::string> &frames) const;
	
    private:

	/** \brief Free memory. */
	void freeMemory(void);
	
	/** \brief Configure the filter. */
	bool configure(void);

	/** \brief Configure the filter. */
	bool configure(double scale, double padd);

	/** \brief Configure the filter. */
	bool configure(const std::vector<std::string> &links, double scale, double padd);
	
	/** \brief Compute bounding spheres for the checked robot links. */
	void computeBoundingSpheres(void);
	
	/** \brief Perform the actual mask computation. */
	void maskAuxContainment(const robot_msgs::PointCloud& data_in, std::vector<int> &mask);

	/** \brief Perform the actual mask computation. */
	void maskAuxIntersection(const robot_msgs::PointCloud& data_in, const std::string &sensor_frame, std::vector<int> &mask);
	
	planning_environment::RobotModels   rm_;
	tf::TransformListener              &tf_;
	ros::NodeHandle                     nh_;
	
	btVector3                           sensor_pos_;
	
	std::vector<SeeLink>                bodies_;
	std::vector<double>                 bspheresRadius2_;
	std::vector<bodies::BoundingSphere> bspheres_;
	
    };
    
}

#endif
