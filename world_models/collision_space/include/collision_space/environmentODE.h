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

#include "collision_space/environment.h"
#include <ode/ode.h>

namespace collision_space
{
    	
    /** \brief A class describing an environment for a kinematic robot using ODE */
    class EnvironmentModelODE : public EnvironmentModel
    {     
	
	friend void nearCallbackFn(void *data, dGeomID o1, dGeomID o2);
	
    public:
		
        EnvironmentModelODE(void) : EnvironmentModel()
	{
	    dInitODE2(0);
	    m_space = dHashSpaceCreate(0);
	    m_spaceBasicGeoms = dHashSpaceCreate(0);
	}
	
	virtual ~EnvironmentModelODE(void)
	{
	    freeMemory();
	    dCloseODE();
	}
	
	/** \brief The space ID for the objects that can be changed in the
	    map. clearObstacles will invalidate this ID. Collision
	    checking on this space is optimized for many small
	    objects. */
	dSpaceID getODESpace(void) const;

	/** \brief Return the space ID for the space in which static objects are added */
	dSpaceID getODEBasicGeomSpace(void) const;

	/** \brief Return the space ID for the space in which the robot model is instanciated */
	dSpaceID getModelODESpace(void) const;
	
	/** \brief Get the list of contacts (collisions) */
	virtual bool getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count = 1);

	/** \brief Check if a model is in collision */
	virtual bool isCollision(void);

	/** \brief Check if a model is in self collision */
	virtual bool isSelfCollision(void);

	/** \brief Remove all obstacles from collision model */
	virtual void clearObstacles(void);

	/** \brief Add a point cloud to the collision space */
	virtual void addPointCloud(unsigned int n, const double *points); 

	/** \brief Add a plane to the collision space. Equation it satisfies is a*x+b*y+c*z = d*/
	virtual void addStaticPlane(double a, double b, double c, double d);

	/** \brief Add a robot model. Ignore robot links if their name is not
	    specified in the string vector. The scale argument can be
	    used to increase or decrease the size of the robot's
	    bodies (multiplicative factor). The padding can be used to
	    increase or decrease the robot's bodies with by an
	    additive term */
	virtual void addRobotModel(const boost::shared_ptr<planning_models::KinematicModel> &model, const std::vector<std::string> &links, double scale = 1.0, double padding = 0.0);

	/** \brief Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(void);

	/** \brief Update the set of bodies that are attached to the robot (re-creates them) */
	virtual void updateAttachedBodies(void);

	/** \brief Enable/Disable collision checking for specific links. Return the previous value of the state (1 or 0) if succesful; -1 otherwise */
	virtual int setCollisionCheck(const std::string &link, bool state);
	
    protected:
	
	/** \brief Internal function for collision detection */
	void testCollision(void *data);
	void testSelfCollision(void *data);
	void testStaticBodyCollision(void *data);
	void testDynamicBodyCollision(void *data);

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
		bool operator()(const Geom *a, const Geom *b) const
		{
		    if (a->aabb[0] < b->aabb[0])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByYLow
	    {
		bool operator()(const Geom *a, const Geom *b) const
		{
		    if (a->aabb[2] < b->aabb[2])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByZLow
	    {
		bool operator()(const Geom *a, const Geom *b) const
		{
		    if (a->aabb[4] < b->aabb[4])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByXTest
	    {
		bool operator()(const Geom *a, const Geom *b) const
		{
		    if (a->aabb[1] < b->aabb[0])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByYTest
	    {
		bool operator()(const Geom *a, const Geom *b) const
		{
		    if (a->aabb[3] < b->aabb[2])
			return true;
		    return false;
		}
	    };
	    
	    struct SortByZTest
	    {
		bool operator()(const Geom *a, const Geom *b) const
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
	    std::vector<dGeomID>                   geom;
	    bool                                   enabled;
	    planning_models::KinematicModel::Link *link;
	    unsigned int                           index;
	};

	struct ModelInfo
	{
	    std::vector< kGeom* >                    linkGeom;
	    double                                   scale;
	    double                                   padding;
	    dSpaceID                                 space;
	};
	
	dGeomID createODEGeom(dSpaceID space, shapes::Shape *shape, double scale, double padding);
	void    updateGeom(dGeomID geom, btTransform &pose) const;	
	void    freeMemory(void);	
	
	ModelInfo            m_modelGeom;
	dSpaceID             m_space;
	dSpaceID             m_spaceBasicGeoms;
	
	/* This is where geoms from the world (that can be cleared and recreated) are added; the space for this is m_space */
	ODECollide2          m_collide2;

	/* This is where static geoms from the world (that are not cleared) are added; the space for this is m_spaceBasicGeoms */
	std::vector<dGeomID> m_basicGeoms;
	
    private:
	
	/* Pointers for ODE indices; we need this around in ODE's assumed datatype */
	std::vector<dTriIndex*> m_meshIndices;	
	
    };
}

#endif
