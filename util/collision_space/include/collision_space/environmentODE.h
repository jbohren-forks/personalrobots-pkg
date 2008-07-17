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

class EnvironmentModelODE : public EnvironmentModel
{
public:

    class KinematicModelODE : public KinematicModel
    {
    public:
	
        KinematicModelODE(void) : KinematicModel()
	{
	    m_space = NULL;
	}
	
	virtual ~KinematicModelODE(void)
	{
	    for (unsigned int i = 0 ; i < m_kgeoms.size() ; ++i)
		delete m_kgeoms[i];
	}
	
	virtual void build(URDF &model, const char *group = NULL);
	
	dSpaceID getODESpace(void) const;
	void     setODESpace(dSpaceID space);

	unsigned int getGeomCount(void) const;
	dGeomID      getGeom(unsigned index) const;
	
	void updateCollisionPositions(void);
	
	
    protected:
	
	dSpaceID m_space;
	
	struct kGeom
	{
	    dGeomID geom;
	    Link   *link;
	};
	
	std::vector<kGeom*> m_kgeoms;    
	
	void buildODEGeoms(Robot *robot);
	dGeomID buildODEGeom(Geometry *geom);
	void setGeomPose(dGeomID geom, libTF::Pose3D &pose) const;
	
    };

    EnvironmentModelODE(void) : EnvironmentModel()
    {
	model = dynamic_cast<KinematicModel*>(&m_modelODE);
	m_space = dHashSpaceCreate(0);
	m_modelODE.setODESpace(m_space);
    }
    
    ~EnvironmentModelODE(void)
    {
	if (m_space)
	    dSpaceDestroy(m_space);
    }

    /** Check if the model is in collision */
    virtual bool isCollision(void);
    
    /** Add a point cloud to the collision space */
    virtual void addPointCloud(unsigned int n, const double *points, double radius = 0.01); 

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

    dSpaceID             m_space;
    ODECollide2          m_collide2;
    KinematicModelODE    m_modelODE;
    std::vector<dGeomID> m_obstacles;
    
};

#endif
