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

#ifndef PLANNING_ENVIRONMENT_COLLISION_MODELS_
#define PLANNING_ENVIRONMENT_COLLISION_MODELS_

#include "planning_environment/robot_models.h"
#include <collision_space/environmentODE.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace planning_environment
{

    /** @htmlinclude ../../manifest.html

	@mainpage
	
	A class capable of loading a robot model from the parameter server */
    
    class CollisionModels : public RobotModels
    {
    public:
	
	CollisionModels(const std::string &description, double scale = 1.0, double padd = 0.0) : RobotModels(description), scale_(scale), padd_(padd)
	{
	    loadCollision();
	}
	
	virtual ~CollisionModels(void)
	{
	}

	/** Reload the robot description and recreate the model */	
	virtual void reload(void)
	{
	    RobotModels::reload();
	    ode_collision_model_.reset();
	    self_see_links_.clear();
	    collision_check_links_.clear();
	    self_collision_check_groups_.clear();
	    loadCollision();
	}
	
	/** Return the instance of the constructed ODE collision model */
	const boost::shared_ptr<collision_space::EnvironmentModel> &getODECollisionModel(void) const
	{
	    return ode_collision_model_;
	}

	const std::vector<std::string> &getCollisionCheckLinks(void) const
	{
	    return collision_check_links_;
	}
	
	const std::vector<std::string> &getSelfSeeLinks(void) const
	{
	    return self_see_links_;
	}

	const std::vector< std::vector<std::string> > &getSelfCollisionGroups(void) const
	{
	    return self_collision_check_groups_;
	}
	
    protected:
	
	void loadCollision(void);
	void getSelfSeeLinks(std::vector<std::string> &links);
	void getCollisionCheckLinks(std::vector<std::string> &links);
	void getSelfCollisionGroups(std::vector< std::vector<std::string> > &groups);
	
	std::vector<std::string>                             self_see_links_;
	std::vector<std::string>                             collision_check_links_;
	std::vector< std::vector<std::string> >              self_collision_check_groups_;
	
	boost::shared_ptr<collision_space::EnvironmentModel> ode_collision_model_;

	double                                               scale_;
	double                                               padd_;
    };
    
	
}

#endif

