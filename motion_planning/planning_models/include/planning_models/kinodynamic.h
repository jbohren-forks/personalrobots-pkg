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

#ifndef KINODYNAMIC_ROBOT_MODEL_ODE_
#define KINODYNAMIC_ROBOT_MODEL_ODE_

#include <urdf/URDF.h>
#include <libTF/Pose3D.h>

#include <planning_models/kinematic.h>
#include <ode/ode.h>

#include <vector>

/** @htmlinclude ../../manifest.html

    A class describing a kinodynamic robot model for ODE, loaded from URDF */

namespace planning_models
{
    
    /** Definition of an ODE kinodynamic model */
    class KinodynamicModelODE
    {
    public:
	
	KinodynamicModelODE(dWorldID world, dSpaceID space)
	{
	    m_world = world;
	    m_space = space;
	}
	
	virtual ~KinodynamicModelODE(void)
	{
	    for (unsigned int i = 0 ; i < m_parts.size() ; ++i)
		delete m_parts[i];
	}
	
	virtual void build(const robot_desc::URDF &model, bool ignoreSensors = false);
	
	dJointID getJoint(const std::string &name)
	{
	    return m_nameJoint[name];	    
	}

	dBodyID getBody(const std::string &name)
	{
	    return m_namePart[name]->body;
	}

	dGeomID getGeom(const std::string &name)
	{
	    return m_namePart[name]->geom;
	}
	
    protected:
	
	struct ODEPart
	{
	    dBodyID body;
	    dMass   mass;
	    dGeomID geom;
	};	

	void setBodyPosition(dBodyID body, const libTF::Pose3D &pose);	
	void setGeomPosition(dGeomID geom, const libTF::Pose3D &pose);
	dGeomID createODEGeom(dSpaceID space, KinematicModel::Shape *shape) const;
	
	KinematicModel                  m_km;

	dWorldID                        m_world;
	dSpaceID                        m_space;

	std::vector<ODEPart*>           m_parts;
	std::map<std::string, ODEPart*> m_namePart;	
	
	std::vector<dJointID>           m_joints;
	std::map<std::string, dJointID> m_nameJoint;	
	
    };

}

#endif
