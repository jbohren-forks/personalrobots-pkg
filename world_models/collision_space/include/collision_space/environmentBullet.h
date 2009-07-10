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

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_BULLET_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_BULLET_

#include "collision_space/environment.h"

#include "btBulletCollisionCommon.h"

namespace collision_space
{
    	
    /** \brief A class describing an environment for a kinematic robot using bullet */
    class EnvironmentModelBullet : public EnvironmentModel
    {     
    public:
	
        EnvironmentModelBullet(void) : EnvironmentModel(),
				       m_selfCollisionFilterCallback(&m_selfCollisionTest),
				       m_genericCollisionFilterCallback(&m_selfCollisionTest, &m_selfCollision)
	{
	    m_config = new btDefaultCollisionConfiguration();
	    btCollisionDispatcher* dispatcher = new CollisionDispatcher(&m_selfCollisionTest, &m_selfCollision, m_config);
	    btBroadphaseInterface *broadphase = new btDbvtBroadphase();
	    m_world = new btCollisionWorld(dispatcher, broadphase, m_config);
	}
	
	virtual ~EnvironmentModelBullet(void)
	{ 
	    freeMemory();
	}
	
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

	struct kGeom
	{
	    std::vector<btCollisionObject*>        geom;
	    bool                                   enabled;
	    planning_models::KinematicModel::Link *link;
	    unsigned int                           index;
	};

	struct ModelInfo
	{
	    std::vector< kGeom* > linkGeom;
	    double                scale;
	    double                padding; 
	};
	
	struct SelfCollisionFilterCallback : public btOverlapFilterCallback
	{
	    SelfCollisionFilterCallback(std::vector< std::vector<bool> > *test) : selfCollisionTest(test)
	    {
	    }
	    
	    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	    {
		assert(static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL);
		assert(static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL);
		
		kGeom *k0 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer());
		kGeom *k1 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer());
		
		// only check collision between links
		if (!k0 || !k1)
		    return false;
		
		// only consider links that are enabled for self-collision checking
		return selfCollisionTest->at(k0->index)[k1->index];
	    }
	    
	    std::vector< std::vector<bool> > *selfCollisionTest;
	};

	struct GenericCollisionFilterCallback : public btOverlapFilterCallback
	{
	    GenericCollisionFilterCallback(std::vector< std::vector<bool> > *test, bool *checkSelf) : selfCollisionTest(test),
												      enableSelfCollision(checkSelf)
	    {
	    }
	    
	    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	    {
		assert(static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL);
		assert(static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL);
		
		kGeom *k0 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer());
		kGeom *k1 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer());

		// do not check collision between obstacles
		if (!k0 && !k1)
		    return false;

		// do not check disabled links
		if (k0)
		    if (k0->enabled == false)
			return false;
		if (k1)
		    if (k1->enabled == false)
			return false;
		
		// do not check collision between links that should not be self-collision checked
		if (k0 && k1)
		{
		    if (*enableSelfCollision)
			return selfCollisionTest->at(k0->index)[k1->index];
		    else
			return false;
		}
		
		return true;
	    }
	    
	    std::vector< std::vector<bool> > *selfCollisionTest;
	    bool                             *enableSelfCollision;
	};
	
	class CollisionDispatcher : public btCollisionDispatcher
	{
	public:
	    CollisionDispatcher(std::vector< std::vector<bool> > *test, bool *enableSelfCollision,
				btCollisionConfiguration *config) : btCollisionDispatcher(config),
								    m_selfCollisionTest(test),
								    m_enableSelfCollision(enableSelfCollision)
	    {
	    }
	    
	    virtual bool needsCollision(btCollisionObject* b0, btCollisionObject* b1)
	    {	
		kGeom *k0 = reinterpret_cast<kGeom*>(b0->getUserPointer());
		kGeom *k1 = reinterpret_cast<kGeom*>(b1->getUserPointer());
		
		if (k0 || k1)
		{
		    if (k0)
			if (k0->enabled == false)
			    return false;
		    if (k1)
			if (k1->enabled == false)
			    return false;
		    if (k0 && k1)
			return m_selfCollisionTest->at(k0->index)[k1->index];
		    return true;
		}
		else
		    return false;
	    }
	    
	protected:
	    
	    std::vector< std::vector<bool> > *m_selfCollisionTest;
	    bool                             *m_enableSelfCollision;
	};
	
	btCollisionObject* createCollisionBody(shapes::Shape *shape, double scale, double padding);
	void freeMemory(void);
	
	SelfCollisionFilterCallback      m_selfCollisionFilterCallback;
	GenericCollisionFilterCallback   m_genericCollisionFilterCallback;
	
	ModelInfo                        m_modelGeom;
	std::vector<btCollisionObject*>  m_dynamicObstacles;
	btCollisionWorld                *m_world;
	btDefaultCollisionConfiguration *m_config;
	
    };
}

#endif
