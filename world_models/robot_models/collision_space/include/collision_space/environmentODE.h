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

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_ODE_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_ODE_

#include <collision_space/environment.h>
#include <ode/ode.h>

/** @htmlinclude ../../manifest.html

    A class describing an environment for a kinematic robot using ODE */

namespace collision_space
{
    
    class EnvironmentModelODE : public EnvironmentModel
    {
    public:
		
        EnvironmentModelODE(bool performODEinit = true) : EnvironmentModel()
	{
	    static bool initODE = true;
	    if (initODE)
	    {
		if (performODEinit)
		    dInitODE();
		initODE = false;
	    }
	    
	    m_space = dHashSpaceCreate(0);
	    m_spaceBasicGeoms = dHashSpaceCreate(0);
	}
	
	virtual ~EnvironmentModelODE(void)
	{
	    freeMemory();
	}
	
	/** The space ID for the objects that can be changed in the
	    map. clearObstacles will invalidate this ID. Collision
	    checking on this space is optimized for many small
	    objects. */
	dSpaceID getODESpace(void) const;

	/** Return the space ID for the space in which static objects are added */
	dSpaceID getODEBasicGeomSpace(void) const;

	/** Return the space ID for the space in which the particular model is instanciated */
	dSpaceID getModelODESpace(unsigned int model_id) const;
	
	/** Check if a model is in collision */
	virtual bool isCollision(unsigned int model_id);
	
	/** Remove all obstacles from collision model */
	virtual void clearObstacles(void);

	/** Add a point cloud to the collision space */
	virtual void addPointCloud(unsigned int n, const double *points); 

	/** Add a plane to the collision space. Equation it satisfies is a*x+b*y+c*z = d*/
	virtual void addStaticPlane(double a, double b, double c, double d);

	/** Add a robot model. Ignore robot links if their name is not specified in the string vector */
	virtual unsigned int addRobotModel(planning_models::KinematicModel *model, const std::vector<std::string> &links);

	/** Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(unsigned int model_id);

	/** Add a group of links to be checked for self collision */
	virtual void addSelfCollisionGroup(unsigned int model_id, std::vector<std::string> &links);

	/** Enable/Disable collision checking for specific links */
	virtual void setCollisionCheck(unsigned int model_id, std::string &link, bool state);
	
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
		clear();
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
	    
	    struct SortByXLow
	    {
		bool operator()(const Geom *a, const Geom *b) 
		{
		    if (a->aabb[0] < b->aabb[0])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByYLow
	    {
		bool operator()(const Geom *a, const Geom *b) 
		{
		    if (a->aabb[2] < b->aabb[2])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByZLow
	    {
		bool operator()(const Geom *a, const Geom *b) 
		{
		    if (a->aabb[4] < b->aabb[4])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByXTest
	    {
		bool operator()(const Geom *a, const Geom *b)
		{
		    if (a->aabb[1] < b->aabb[0])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByYTest
	    {
		bool operator()(const Geom *a, const Geom *b)
		{
		    if (a->aabb[3] < b->aabb[2])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByZTest
	    {
		bool operator()(const Geom *a, const Geom *b)
		{
		    if (a->aabb[5] < b->aabb[4])
			return true;
		    return false;
		}
	    };
	    
	    bool               m_setup;
	    std::vector<Geom*> m_geomsX;
	    std::vector<Geom*> m_geomsY;
	    std::vector<Geom*> m_geomsZ;
	    
	    void checkColl(std::vector<Geom*>::iterator posStart, std::vector<Geom*>::iterator posEnd,
			   Geom *g, void *data, dNearCallback *nearCallback);
	};

	struct kGeom
	{
	    dGeomID                                geom;
	    bool                                   enabled;
	    planning_models::KinematicModel::Link *link;
	};
	
	struct ModelInfo
	{
	    std::vector< kGeom* >                    geom;
	    dSpaceID                                 space;
	    std::vector< std::vector<unsigned int> > selfCollision;	    
	};
	
	dGeomID createODEGeom(dSpaceID space, planning_models::KinematicModel::Shape *shape) const;
	void    freeMemory(void);	
	
	std::vector<ModelInfo> m_kgeoms;
	dSpaceID               m_space;
	dSpaceID               m_spaceBasicGeoms;
	
	/* This is where geoms from the world (that can be cleared and recreated) are added; the space for this is m_space */
	ODECollide2            m_collide2;

	/* This is where static geoms from the world (that are not cleared) are added; the space for this is m_spaceBasicGeoms */
	std::vector<dGeomID>   m_basicGeoms;
	
    };
}

#endif
