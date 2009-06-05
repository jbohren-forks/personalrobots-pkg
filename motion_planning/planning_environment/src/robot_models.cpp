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

#include "planning_environment/robot_models.h"
#include <ros/console.h>
#include <sstream>

void planning_environment::RobotModels::loadRobot(void)
{
    std::string content;
    if (nh_.getParam(description_, content))
    {
	urdf_ = boost::shared_ptr<robot_desc::URDF>(new robot_desc::URDF());
	if (urdf_->loadString(content.c_str()))
	{
	    getPlanningGroups(planning_groups_);
	    kmodel_ = boost::shared_ptr<planning_models::KinematicModel>(new planning_models::KinematicModel());
	    kmodel_->setVerbose(false);
	    kmodel_->build(*urdf_, planning_groups_);


	    // make sure the kinematic model is in its own frame
	    // (remove all transforms caused by planar or floating
	    // joints)
	    kmodel_->reduceToRobotFrame();
	    kmodel_->defaultState();

	    getCollisionCheckLinks(collision_check_links_);	
	    getSelfCollisionGroups(self_collision_check_groups_);
	    getSelfSeeLinks(self_see_links_);	    
	}
	else
	    urdf_.reset();
    }
    else
	ROS_ERROR("Robot model '%s' not found!", description_.c_str());
}


void planning_environment::RobotModels::getPlanningGroups(std::map< std::string, std::vector<std::string> > &groups) 
{
    std::string group_list;
    nh_.param(description_ + "_planning/group_list", group_list, std::string(""));
    std::stringstream group_list_stream(group_list);
    while (group_list_stream.good() && !group_list_stream.eof())
    {
	std::string name;
	std::string group_elems;
	group_list_stream >> name;
	if (name.size() == 0)
	    continue;
	nh_.param(description_ + "_planning/groups/" + name + "/links", group_elems, std::string(""));	
	std::stringstream group_elems_stream(group_elems);
	while (group_elems_stream.good() && !group_elems_stream.eof())
	{
	    std::string link_name;
	    group_elems_stream >> link_name;
	    if (link_name.size() == 0)
		continue;
	    if (urdf_->getLink(link_name))
		groups[name].push_back(link_name);
	    else
		ROS_ERROR("Unknown link: '%s'", link_name.c_str());
	}
    }
}

bool planning_environment::RobotModels::PlannerConfig::hasParam(const std::string &param)
{
    return nh_.hasParam(description_ + "_planning/planner_configs/" + config_ + "/" + param);
}

std::string planning_environment::RobotModels::PlannerConfig::getParam(const std::string &param)
{
    std::string value;
    nh_.param(description_ + "_planning/planner_configs/" + config_ + "/" + param, value, std::string(""));
    return value;
}

std::vector< boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> > planning_environment::RobotModels::getGroupPlannersConfig(const std::string &group)
{
    std::string configs_list;
    nh_.param(description_ + "_planning/groups/" + group + "/planner_configs", configs_list, std::string(""));
    
    std::vector< boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> > configs;
    
    std::stringstream configs_stream(configs_list);
    while (configs_stream.good() && !configs_stream.eof())
    {
	std::string cfg;
	configs_stream >> cfg;
	if (cfg.size() == 0)
	    continue;
	if (nh_.hasParam(description_ + "_planning/planner_configs/" + cfg + "/type"))
	    configs.push_back(boost::shared_ptr<PlannerConfig>(new PlannerConfig(description_, cfg)));
    }
    return configs;
}

void planning_environment::RobotModels::getSelfSeeLinks(std::vector<std::string> &links)
{
    std::string link_list;
    nh_.param(description_ + "_collision/self_see", link_list, std::string(""));
    std::stringstream link_list_stream(link_list);
    
    while (link_list_stream.good() && !link_list_stream.eof())
    {
	std::string name;
	link_list_stream >> name;
	if (name.size() == 0)
	    continue;
	if (urdf_->getLink(name))
	    links.push_back(name);
	else
	    ROS_ERROR("Unknown link: '%s'", name.c_str());
    }
}

void planning_environment::RobotModels::getCollisionCheckLinks(std::vector<std::string> &links)
{
    std::string link_list;
    nh_.param(description_ + "_collision/collision_links", link_list, std::string(""));
    std::stringstream link_list_stream(link_list);
    
    while (link_list_stream.good() && !link_list_stream.eof())
    {
	std::string name;
	link_list_stream >> name;
	if (name.size() == 0)
	    continue;
	if (urdf_->getLink(name))
	    links.push_back(name);
	else
	    ROS_ERROR("Unknown link: '%s'", name.c_str());
    }
}

void planning_environment::RobotModels::getSelfCollisionGroups(std::vector< std::vector<std::string> > &groups)
{
    std::string group_list;
    nh_.param(description_ + "_collision/self_collision_groups", group_list, std::string(""));
    std::stringstream group_list_stream(group_list);
    while (group_list_stream.good() && !group_list_stream.eof())
    {
	std::string name;
	std::string group_elems;
	group_list_stream >> name;
	if (name.size() == 0)
	    continue;
	nh_.param(description_ + "_collision/" + name, group_elems, std::string(""));	
	std::stringstream group_elems_stream(group_elems);
	std::vector<std::string> this_group;
	while (group_elems_stream.good() && !group_elems_stream.eof())
	{
	    std::string link_name;
	    group_elems_stream >> link_name;
	    if (link_name.size() == 0)
		continue;
	    if (urdf_->getLink(link_name))
		this_group.push_back(link_name);
	    else
		ROS_ERROR("Unknown link: '%s'", link_name.c_str());
	}
	if (this_group.size() > 0)
	    groups.push_back(this_group);
    }
}

double planning_environment::RobotModels::getSelfSeePadding(void)
{
    double value;
    nh_.param(description_ + "_collision/self_see_padd", value, 0.0);
    return value;
}

double planning_environment::RobotModels::getSelfSeeScale(void)
{
    double value;
    nh_.param(description_ + "_collision/self_see_scale", value, 1.0);
    return value;
}
