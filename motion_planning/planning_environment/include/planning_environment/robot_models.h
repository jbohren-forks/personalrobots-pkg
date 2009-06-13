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

#include <planning_models/kinematic.h>
#include <ros/ros.h>
#include <ros/node.h>
#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <vector>

namespace planning_environment
{
    
    /** \brief A class capable of loading a robot model from the parameter server */
    
    class RobotModels
    {
    public:
	
	/** \brief A class to define a planner configuration */
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
	    std::string getParamString(const std::string &param, const std::string &def = "");
	    double      getParamDouble(const std::string &param, double def);
	    
	private:
	    
	    std::string     description_;
	    std::string     config_;
	    ros::NodeHandle nh_;	  
	};
	
	
	RobotModels(const std::string &description)
	{
	    description_ = nh_.getNode()->mapName(description);
	    loaded_models_ = false;
	    loadRobot();
	}

	virtual ~RobotModels(void)
	{
	}
	
	/** \brief Return the name of the description */
	const std::string &getDescription(void) const
	{
	    return description_;
	}
	
	/** \brief Return the instance of the constructed kinematic model */
	const boost::shared_ptr<planning_models::KinematicModel> &getKinematicModel(void) const
	{
	    return kmodel_;
	}

	/** \brief Return the instance of the parsed robot description */
	const boost::shared_ptr<robot_desc::URDF> &getParsedDescription(void) const
	{
	    return urdf_;
	}
	
	/** \brief Return the map of the planning groups (arrays of link names) */
	const std::map< std::string, std::vector<std::string> > &getPlanningGroups(void) const
	{
	    return planning_groups_;
	}	
	
	/** \brief Return the names of the links that should be considered when performing collision checking */
	const std::vector<std::string> &getCollisionCheckLinks(void) const
	{
	    return collision_check_links_;
	}

	/** \brief Return the names of the links that should be considered when cleaning sensor data
	    of parts the robot can see from itself */
	const std::vector<std::string> &getSelfSeeLinks(void) const
	{
	    return self_see_links_;
	}

	/** \brief Return the groups of links that should be considered when testing for self collision. This is an
	    array of pairs. Both elements of the pair are groups of links. If any link in the first member of the pair
	    collides with some link in the second member of the pair, we have a collision */
	const std::vector< std::pair < std::vector<std::string>, std::vector<std::string> > > &getSelfCollisionGroups(void) const
	{
	    return self_collision_check_groups_;
	}
	
	/** \brief Return true if models have been loaded */
	bool loadedModels(void) const
	{
	    return loaded_models_;
	}
	
	/** \brief Get the amount of padding to be used for links when cleaning sensor data */
	double getSelfSeePadding(void);

	/** \brief Get the amount of scaling to be used for links when cleaning sensor data */
	double getSelfSeeScale(void);
	
	/** \breif Get the list of planner configurations available for a specific planning group */
	std::vector< boost::shared_ptr<PlannerConfig> > getGroupPlannersConfig(const std::string &group);
	
	/** \brief Reload the robot description and recreate the model */
	virtual void reload(void);
	
    protected:
	
	void loadRobot(void);
	void getPlanningGroups(std::map< std::string, std::vector<std::string> > &groups);
	
	void getSelfSeeLinks(std::vector<std::string> &links);
	void getCollisionCheckLinks(std::vector<std::string> &links);
	void getSelfCollisionGroups(std::vector< std::pair < std::vector<std::string>, std::vector<std::string> > > &groups);
	

	ros::NodeHandle                                    nh_;
	  
	std::string                                        description_;
	
	bool                                               loaded_models_;
	boost::shared_ptr<planning_models::KinematicModel> kmodel_;
	boost::shared_ptr<robot_desc::URDF>                urdf_;
	
	std::map< std::string, std::vector<std::string> >  planning_groups_;
	std::vector<std::string>                           self_see_links_;
	std::vector<std::string>                           collision_check_links_;
	std::vector< std::pair < std::vector<std::string>, std::vector<std::string> > >
	                                                   self_collision_check_groups_;

    };
    
}

#endif

