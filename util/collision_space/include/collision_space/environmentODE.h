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

#ifndef KINEMATIC_ENVIRONMENT_MODEL_ODE_
#define KINEMATIC_ENVIRONMENT_MODEL_ODE_

#include <collision_space/environment.h>
#include <ode/ode.h>

/** @htmlinclude ../../manifest.html

    A class describing an environment for a kinematic robot using ODE */

namespace collision_space
{
    
    class EnvironmentModelODE : public EnvironmentModel
    {
    public:
		
        EnvironmentModelODE(void) : EnvironmentModel()
	{
	    m_space = dHashSpaceCreate(0);
	}
	
	~EnvironmentModelODE(void)
	{
	    freeMemory();
	}
	
	dSpaceID getODESpace(void) const;
	
	/** Check if a model is in collision */
	virtual bool isCollision(unsigned int model_id);
	
	/** Remove all obstacles from collision model */
	virtual void clearObstacles(void);

	/** Add a point cloud to the collision space */
	virtual void addPointCloud(unsigned int n, const double *points, double radius = 0.01); 

	/** Add a robot model */
	virtual unsigned int addRobotModel(planning_models::KinematicModel *model);

	/** Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(unsigned int model_id);

    protected:
		
	class ODECollide2
	{
	public:
	    
	    ODECollide2(dSpaceID space = NULL)
	    {	
		m_setup = false;
		if (space)
		    registerSpace(space);
	    }
	    
	    ~ODECollide2(void)
	    {
	    }
	    
	    void registerSpace(dSpaceID space);
	    void registerGeom(dGeomID geom);
	    void clear(void);
	    void setup(void);
	    void collide(dGeomID geom, void *data, dNearCallback *nearCallback);
	    
	private:
	    
	    struct Geom
	    {
		dGeomID id;
		dReal   aabb[6];
	    };
	    
	    struct SortByXYZLow
	    {
		bool operator()(const Geom &a, const Geom &b) 
		{
		    if (a.aabb[0] < b.aabb[0])
			return true;
		    if (a.aabb[0] > b.aabb[0])
			return false;
		    
		    if (a.aabb[2] < b.aabb[2])
			return true;
		    if (a.aabb[2] > b.aabb[2])
			return false;
		    
		    if (a.aabb[4] < b.aabb[4])
			return true;
		    return false;	
		}
	    };
	    
	    struct SortByX
	    {
		bool operator()(const Geom &a, const Geom &b)
		{
		    if (a.aabb[1] < b.aabb[0])
			return true;
		    return false;
		}
	    };
	    
	    bool              m_setup;
	    std::vector<Geom> m_geoms;
	    
	};

	struct kGeom
	{
	    dGeomID                                geom;
	    planning_models::KinematicModel::Link *link;
	};

	void freeMemory(void);	
	
	std::vector< std::vector< kGeom* > > m_kgeoms;

	dSpaceID                             m_space;
	ODECollide2                          m_collide2;
	
    };
}

#endif
