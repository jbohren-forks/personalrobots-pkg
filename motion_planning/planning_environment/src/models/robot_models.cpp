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

#include "planning_environment/models/robot_models.h"
#include <ros/console.h>

#include <boost/algorithm/string.hpp>
#include <sstream>

void planning_environment::RobotModels::reload(void)
{
    kmodel_.reset();
    urdf_.reset();
    planning_groups_.clear();
    collision_check_links_.clear();
    self_collision_check_groups_.clear();
    loadRobot();
}

void planning_environment::RobotModels::loadRobot(void)
{
    std::string content;
    if (nh_.getParam(description_, content))
    {
	urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
	if (urdf_->initString(content))
	{
	    loaded_models_ = true;
	    getPlanningGroups(planning_groups_);
	    kmodel_ = boost::shared_ptr<planning_models::KinematicModel>(new planning_models::KinematicModel(*urdf_, planning_groups_));
	    kmodel_->defaultState();
	    getCollisionCheckLinks(collision_check_links_);	
	    getSelfCollisionGroups(self_collision_check_groups_);
	}
	else
	{
	    urdf_.reset();
	    ROS_ERROR("Unable to parse URDF description!");
	}
    }
    else
	ROS_ERROR("Robot model '%s' not found! Did you remap 'robot_description'?", description_.c_str());
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
	nh_.param(description_ + "_planning/groups/" + name + "/joints", group_elems, std::string(""));	
	std::stringstream group_elems_stream(group_elems);
	while (group_elems_stream.good() && !group_elems_stream.eof())
	{
	    std::string joint_name;
	    group_elems_stream >> joint_name;
	    if (joint_name.size() == 0)
		continue;
	    if (urdf_->getJoint(joint_name))
		groups[name].push_back(joint_name);
	    else
		ROS_ERROR("Unknown joint: '%s'", joint_name.c_str());
	}
    }
}

const std::string& planning_environment::RobotModels::PlannerConfig::getName(void)
{
    return config_;
}

bool planning_environment::RobotModels::PlannerConfig::hasParam(const std::string &param)
{
    return nh_.hasParam(description_ + "_planning/planner_configs/" + config_ + "/" + param);
}

std::string planning_environment::RobotModels::PlannerConfig::getParamString(const std::string &param, const std::string& def)
{
    std::string value;
    nh_.param(description_ + "_planning/planner_configs/" + config_ + "/" + param, value, def);
    boost::trim(value);
    return value;
}

double planning_environment::RobotModels::PlannerConfig::getParamDouble(const std::string &param, double def)
{
    double value;
    nh_.param(description_ + "_planning/planner_configs/" + config_ + "/" + param, value, def);
    return value;
}

int planning_environment::RobotModels::PlannerConfig::getParamInt(const std::string &param, int def)
{
    int value;
    nh_.param(description_ + "_planning/planner_configs/" + config_ + "/" + param, value, def);
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

void planning_environment::RobotModels::getSelfCollisionGroups(std::vector< std::pair < std::vector<std::string>, std::vector<std::string> > > &groups)
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
	
	std::pair < std::vector<std::string>, std::vector<std::string> > this_group;
	
	// read part 'a'
	nh_.param(description_ + "_collision/" + name + "/a", group_elems, std::string(""));
	std::stringstream group_elems_stream_a(group_elems);
	while (group_elems_stream_a.good() && !group_elems_stream_a.eof())
	{
	    std::string link_name;
	    group_elems_stream_a >> link_name;
	    if (link_name.size() == 0)
		continue;
	    if (urdf_->getLink(link_name))
		this_group.first.push_back(link_name);
	    else
		ROS_ERROR("Unknown link: '%s'", link_name.c_str());
	}
	
	// read part 'b'
	nh_.param(description_ + "_collision/" + name + "/b", group_elems, std::string(""));
	std::stringstream group_elems_stream_b(group_elems);
	while (group_elems_stream_b.good() && !group_elems_stream_b.eof())
	{
	    std::string link_name;
	    group_elems_stream_b >> link_name;
	    if (link_name.size() == 0)
		continue;
	    if (urdf_->getLink(link_name))
		this_group.second.push_back(link_name);
	    else
		ROS_ERROR("Unknown link: '%s'", link_name.c_str());
	}
	
	if (this_group.first.size() > 0 && this_group.second.size() > 0)
	    groups.push_back(this_group);
    }
}
