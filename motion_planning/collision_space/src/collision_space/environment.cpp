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

#include <collision_space/environment.h>

unsigned int collision_space::EnvironmentModel::addRobotModel(planning_models::KinematicModel *model, const std::vector<std::string> &links)
{
    unsigned int pos = m_models.size();
    m_models.push_back(model);
    return pos;
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


unsigned int collision_space::EnvironmentModel::getModelCount(void) const
{
    return m_models.size();    
}

planning_models::KinematicModel* collision_space::EnvironmentModel::getRobotModel(unsigned int model_id) const
{
    return m_models[model_id];
}
