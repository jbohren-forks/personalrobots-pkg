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

#ifndef PLANNING_ENVIRONMENT_ROBOT_MODELS_
#define PLANNING_ENVIRONMENT_ROBOT_MODELS_

#include "planning_models/kinematic.h"
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <vector>

namespace planning_environment
{
    
    /** @htmlinclude ../../manifest.html

	@mainpage
	
	A class capable of loading a robot model from the parameter server */
    
    class RobotModels
    {
    public:
	
	/** A class to define a planner configuration */
	class PlannerConfig
	{
	public:
	    PlannerConfig(const std::string &description, const std::string &config) : description_(description), config_(config)
	    {
	    }
	    
	    ~PlannerConfig(void)
	    {
	    }
	    
	    bool        hasParam(const std::string &param);
	    std::string getParam(const std::string &param);
	    
	private:
	    
	    std::string     description_;
	    std::string     config_;
	    ros::NodeHandle nh_;	  
	};
	
	
	RobotModels(const std::string &description) : description_(description)
	{
	    loadRobot();
	}
	
	virtual ~RobotModels(void)
	{
	}
	
	const std::string &getDescription(void) const
	{
	    return description_;
	}
	
	/** Return the instance of the constructed kinematic model */
	const boost::shared_ptr<planning_models::KinematicModel> &getKinematicModel(void) const
	{
	    return kmodel_;
	}

	/** Return the instance of the parsed robot description */
	const boost::shared_ptr<robot_desc::URDF> &getParsedDescription(void) const
	{
	    return urdf_;
	}
	
	const std::map< std::string, std::vector<std::string> > &getPlanningGroups(void) const
	{
	    return planning_groups_;
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
	
	double getSelfSeePadding(void);
	double getSelfSeeScale(void);
	
	std::vector< boost::shared_ptr<PlannerConfig> > getGroupPlannersConfig(const std::string &group);
	
	/** Reload the robot description and recreate the model */
	virtual void reload(void)
	{
	    kmodel_.reset();
	    urdf_.reset();
	    planning_groups_.clear();
	    self_see_links_.clear();
	    collision_check_links_.clear();
	    self_collision_check_groups_.clear();
	    loadRobot();
	}
	
    protected:
	
	void loadRobot(void);
	void getPlanningGroups(std::map< std::string, std::vector<std::string> > &groups);
	
	void getSelfSeeLinks(std::vector<std::string> &links);
	void getCollisionCheckLinks(std::vector<std::string> &links);
	void getSelfCollisionGroups(std::vector< std::vector<std::string> > &groups);
	

	ros::NodeHandle                                    nh_;
	  
	std::string                                        description_;
	
	boost::shared_ptr<planning_models::KinematicModel> kmodel_;
	boost::shared_ptr<robot_desc::URDF>                urdf_;
	
	std::map< std::string, std::vector<std::string> >  planning_groups_;
	std::vector<std::string>                           self_see_links_;
	std::vector<std::string>                           collision_check_links_;
	std::vector< std::vector<std::string> >            self_collision_check_groups_;

    };
    
	
}

#endif

