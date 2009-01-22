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

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_

#include <planning_models/kinematic.h>
#include "boost/thread/mutex.hpp"
#include <vector>
#include <string>

/** @htmlinclude ../../manifest.html

    @mainpage
    
    A class describing an environment for a kinematic robot. This is
    the base (abstract) definition. Different implementations are
    possible. The class is aware of a certain set of fixed
    (addStatic*) obstacles that never change, a set of obstacles that
    can change (removed by clearObstacles()) and a set of kinematic
    robots. The class provides functionality for checking whether a
    given robot is in collision. 

    
 */

/** Main namespace */
namespace collision_space
{
    
    class EnvironmentModel
    {
    public:
	
	EnvironmentModel(void)
	{
	    m_selfCollision = true;
	}
	
	virtual ~EnvironmentModel(void)
	{
	    for (unsigned int i = 0 ; i < m_models.size() ; ++i)
		delete m_models[i];
	}
	
	/** Check if a model is in collision */
	virtual bool isCollision(unsigned int model_id) = 0;
	
	/** Remove all obstacles from collision model */
	virtual void clearObstacles(void) = 0;
	
	/** Add a point cloud to the collision space */
	virtual void addPointCloud(unsigned int n, const double* points) = 0;

	/** Add a plane to the collision space. Equation it satisfies is a*x+b*y+c*z = d*/
	virtual void addStaticPlane(double a, double b, double c, double d) = 0;

	/** Add a robot model. Ignore robot links if their name is not specified in the string vector */
	virtual unsigned int addRobotModel(planning_models::KinematicModel *model, const std::vector<std::string> &links);

	/** Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(unsigned int model_id) = 0;

	/** Update the set of bodies that are attached to the robot (re-creates them) */
	virtual void updateAttachedBodies(unsigned int model_id) = 0;

	/** Add a group of links to be checked for self collision */
	virtual void addSelfCollisionGroup(unsigned int model_id, std::vector<std::string> &links) = 0;
	
	/** Get the number of loaded models */
	unsigned int getModelCount(void) const;

	/** Get a specific model */
	planning_models::KinematicModel* getRobotModel(unsigned int model_id) const;
	
	/** Provide interface to a lock. Use carefully! */
	void lock(void);
	
	/** Provide interface to a lock. Use carefully! */
	void unlock(void);
	
	/** Set the status of self collision */
	void setSelfCollision(bool selfCollision);
	
	/** Check if self collision is enabled */
	bool getSelfCollision(void) const;

    protected:
        
	boost::mutex                                  m_lock;
	bool                                          m_selfCollision;
	
	/** List of loaded robot models */	
	std::vector<planning_models::KinematicModel*> m_models;
	
    };
}

#endif
    
