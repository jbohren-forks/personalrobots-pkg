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

#include "collision_space/environment.h"

bool collision_space::EnvironmentModel::getVerbose(void) const
{
    return m_verbose;
}

void collision_space::EnvironmentModel::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

void collision_space::EnvironmentModel::setRobotModel(const boost::shared_ptr<planning_models::KinematicModel> &model, const std::vector<std::string> &links, double scale, double padding)
{
    m_robotModel = model;
    m_collisionLinks = links;
    m_selfCollisionTest.resize(links.size());
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	m_selfCollisionTest[i].resize(links.size(), false);
	m_collisionLinkIndex[links[i]] = i;
    }
    m_robotScale = scale;
    m_robotPadd = padding;
}

void collision_space::EnvironmentModel::addSelfCollisionGroup(std::vector<std::string> &links)
{
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	if (m_collisionLinkIndex.find(links[i]) == m_collisionLinkIndex.end())
	{
	    m_msg.error("Unknown link '%s'", links[i].c_str());
	    continue;
	}
	for (unsigned int j = i + 1 ; j < links.size() ; ++j)
	{
	    if (m_collisionLinkIndex.find(links[j]) == m_collisionLinkIndex.end())
	    {
		m_msg.error("Unknown link '%s'", links[j].c_str());
		continue;
	    }
	    m_selfCollisionTest[m_collisionLinkIndex[links[i]]][m_collisionLinkIndex[links[j]]] = true;
	    m_selfCollisionTest[m_collisionLinkIndex[links[j]]][m_collisionLinkIndex[links[i]]] = true;
	}
    }
}

void collision_space::EnvironmentModel::lock(void)
{
    m_lock.lock();
}

void collision_space::EnvironmentModel::unlock(void)
{
    m_lock.unlock();    
}

void collision_space::EnvironmentModel::setSelfCollision(bool selfCollision)
{
    m_selfCollision = selfCollision;
}

bool collision_space::EnvironmentModel::getSelfCollision(void) const
{
    return m_selfCollision;
}
