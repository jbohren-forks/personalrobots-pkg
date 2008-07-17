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

#ifndef KINEMATIC_ENVIRONMENT_MODEL_SOLID_
#define KINEMATIC_ENVIRONMENT_MODEL_SOLID_

#include <collisionspace/environment.h>
#include <robotmodels/kinematic.h>
#include <SOLID/solid.h>

/** @htmlinclude ../../manifest.html

    A class describing an environment for a kinematic robot using SOLID */

class EnvironmentModelSOLID : public EnvironmentModel
{
 public:

    /* dummy object for SOLID object references */
    struct SOLIDObject
    {
	unsigned int id;	
    };
    
    /* an object in the world */
    struct Object
    {
	Object(void)
	{
	    obj = new SOLIDObject();
	    shape = NULL;
	}
	
	~Object(void)
	{
	    if (obj && shape)
	    {
		dtDeleteObject(obj);
		dtDeleteShape(shape);
	    }
	    if (obj)
		delete obj;
	}
	
	SOLIDObject *obj;
	DtShapeRef   shape;
    };
    
    class KinematicModelSOLID : public KinematicModel
    {
    public:
       	
        KinematicModelSOLID(void) : KinematicModel()
	{
	}
	
	virtual ~KinematicModelSOLID(void)
	{
	    for (unsigned int i = 0 ; i < m_kshapes.size() ; ++i)
		delete m_kshapes[i];
	}
	
	virtual void build(URDF &model, const char *group = NULL);
	
	unsigned int getObjectCount(void) const;
	Object*      getObject(unsigned int index) const;
	
	void updateCollisionPositions(void);
	
    protected:
	
	struct kShape
	{
	    kShape(void)
	    {
		obj = new Object();
		link = NULL;
	    }
	    
	    ~kShape(void)
	    {
		if (obj)
		    delete obj;
	    }	
	    
	    Object *obj;	
	    Link   *link;	
	};
	
	std::vector<kShape*> m_kshapes;
	
	void buildSOLIDShapes(Robot *robot);
	DtShapeRef buildSOLIDShape(Geometry *geom);
	
    };
    
    
    EnvironmentModelSOLID(void) : EnvironmentModel()
    {
	model = dynamic_cast<KinematicModel*>(&m_modelSOLID);
    }
    
    ~EnvironmentModelSOLID(void)
    {
	for (unsigned int i = 0 ; i < m_obstacles.size() ; ++i)
	    delete m_obstacles[i];
    }
    
    /** Check if the model is in collision */
    virtual bool isCollision(void);
    
    /** Add a point cloud to the collision space */
    virtual void addPointCloud(unsigned int n, const double *points, double radius = 0.01); 
    
 protected:
    
    KinematicModelSOLID  m_modelSOLID;    
    std::vector<Object*> m_obstacles;

    
};

#endif
