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

/** \Author Ioan Sucan, Matei Ciocarlie */

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_OCTREE_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_OCTREE_

#include <collision_space/environment.h>
#include <octree.h>
#include <vector>

/** @htmlinclude ../../manifest.html

    A class describing an environment for a kinematic robot using ODE */

namespace collision_space
{
    
    static const char OCTREE_CELL_EMPTY    = 0;
    static const char OCTREE_CELL_OCCUPIED = 1;
    
    class EnvironmentModelOctree : public EnvironmentModel
    {
    public:
		
        EnvironmentModelOctree(void) : EnvironmentModel(),
	                               m_octree(0.0f, 0.0f, 0.0f, 0.02f, 0.02f, 0.02f, 1, OCTREE_CELL_EMPTY)
	{ 
	    m_octree.setAutoExpand(true);
	    m_selfCollision = false;
	}
	
	virtual ~EnvironmentModelOctree(void)
	{
	}
	
	/** Check if a model is in collision */
	virtual bool isCollision(unsigned int model_id);
	
	/** Remove all obstacles from collision model */
	virtual void clearObstacles(void);
	
	/** Add a point cloud to the collision space */
	virtual void addPointCloud(unsigned int n, const double *points, double radius = 0.01); 
	
	/** Add a plane to the collision space. Equation it satisfies is a*x+b*y+c*z = d*/
	virtual void addPlane(double a, double b, double c, double d);
	
	/** Add a robot model. Ignore robot links if their name is not specified in the string vector */
	virtual unsigned int addRobotModel(planning_models::KinematicModel *model, const std::vector<std::string> &links);

	/** Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(unsigned int model_id);

	/** Add a group of links to be checked for self collision */
	virtual void addSelfCollisionGroup(unsigned int model_id, std::vector<std::string> &links);

	const scan_utils::Octree<char>* getOctree(void) const;
	
    protected:
	
	scan_utils::Octree<char>                                           m_octree;
	std::vector<std::vector< planning_models::KinematicModel::Link*> > m_modelLinks;
	
    };
}

#endif
