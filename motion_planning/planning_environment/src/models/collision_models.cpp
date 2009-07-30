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

#include "planning_environment/models/collision_models.h"
#include <collision_space/environmentODE.h>
#include <collision_space/environmentBullet.h>
#include <sstream>
#include <vector>

void planning_environment::CollisionModels::setupModel(boost::shared_ptr<collision_space::EnvironmentModel> &model, const std::vector<std::string> &links)
{
    model->lock();
    model->setRobotModel(kmodel_, links, scale_, padd_);
    
    // form all pairs of links that can collide and add them as self-collision groups
    for (unsigned int i = 0 ; i < self_collision_check_groups_.size() ; ++i)
	for (unsigned int g1 = 0 ; g1 < self_collision_check_groups_[i].first.size() ; ++g1)
	    for (unsigned int g2 = 0 ; g2 < self_collision_check_groups_[i].second.size() ; ++g2)
	    {
		std::vector<std::string> scg;
		scg.push_back(self_collision_check_groups_[i].first[g1]);
		scg.push_back(self_collision_check_groups_[i].second[g2]);
		model->addSelfCollisionGroup(scg);
	    }
    model->updateRobotModel();

    for (unsigned int i = 0 ; i < boundingPlanes_.size() / 4 ; ++i)
    {
	model->addPlane("bounds", boundingPlanes_[i * 4], boundingPlanes_[i * 4 + 1], boundingPlanes_[i * 4 + 2], boundingPlanes_[i * 4 + 3]);
	ROS_INFO("Added static plane %fx + %fy + %fz + %f = 0 for model %p", boundingPlanes_[i * 4], boundingPlanes_[i * 4 + 1], boundingPlanes_[i * 4 + 2], boundingPlanes_[i * 4 + 3], model.get());
    }
    
    model->unlock();    
}

void planning_environment::CollisionModels::loadParams(void)
{
    nh_.param(description_ + "_collision/robot_padd", padd_, 0.01);
    nh_.param(description_ + "_collision/robot_scale", scale_, 1.0);
}

void planning_environment::CollisionModels::loadCollision(const std::vector<std::string> &links)
{
    // a list of static planes bounding the environment
    boundingPlanes_.clear();
    
    std::string planes;
    nh_.param<std::string>("~bounding_planes", planes, std::string());
    
    std::stringstream ss(planes);
    if (!planes.empty())
	while (ss.good() && !ss.eof())
	{
	    double value;
	    ss >> value;
	    boundingPlanes_.push_back(value);
	}
    if (boundingPlanes_.size() % 4 != 0)
    {
	ROS_WARN("~bounding_planes must be a list of 4-tuples (a b c d) that define planes ax+by+cz+d=0");
	boundingPlanes_.resize(boundingPlanes_.size() - (boundingPlanes_.size() % 4));
    }
    
    if (loadedModels())
    {
	ode_collision_model_ = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelODE());
	setupModel(ode_collision_model_, links);
	
	//	bullet_collision_model_ = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelBullet());
	//	setupModel(bullet_collision_model_, links);
    }
}
