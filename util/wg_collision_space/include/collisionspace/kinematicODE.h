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

#ifndef KINEMATIC_ROBOT_ODE_MODEL_
#define KINEMATIC_ROBOT_ODE_MODEL_

#include <robotmodels/kinematic.h>
#include <ode/ode.h>

/** @htmlinclude ../../manifest.html

    A class describing a kinematic robot model loaded from URDF.
    This class also provides collision detection with ODE. */

class KinematicModelODE : public KinematicModel
{
 public:
    
    KinematicModelODE(void) : KinematicModel()
    {
	m_space = NULL;
    }
    
    virtual ~KinematicModelODE(void)
    {
	if (m_space)
	    dSpaceDestroy(m_space);
	for (unsigned int i = 0 ; i < m_kgeoms.size() ; ++i)
	    delete m_kgeoms[i];
    }

    virtual void build(URDF &model, const char *group = NULL);

    dSpaceID getODESpace(void) const;
    unsigned int getGeomCount(void) const;
    dGeomID getGeom(unsigned index) const;

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

#endif
