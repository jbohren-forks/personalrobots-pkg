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

#ifndef DISPLAY_ODE_H
#define DISPLAY_ODE_H

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <vector>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawTriangle dsDrawTriangleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawConvex dsDrawConvexD
#endif

namespace display_ode
{
    
    class DisplayODESpaces
    {
    public:

	// copied from an ODE demo program
	void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
	{
	    int i;
	    
	    if (!g) return;
	    if (!pos) pos = dGeomGetPosition (g);
	    if (!R) R = dGeomGetRotation (g);
	    
	    int type = dGeomGetClass (g);
	    if (type == dBoxClass) {
		dVector3 sides;
		dGeomBoxGetLengths (g,sides);
		dsDrawBox (pos,R,sides);
	    }
	    else if (type == dSphereClass) {
		dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
	    }
	    else if (type == dCapsuleClass) {
		dReal radius,length;
		dGeomCapsuleGetParams (g,&radius,&length);
		dsDrawCapsule (pos,R,length,radius);
	    }
	    else if (type == dCylinderClass) {
		dReal radius,length;
		dGeomCylinderGetParams (g,&radius,&length);
		dsDrawCylinder (pos,R,length,radius);
	    }
	    else if (type == dGeomTransformClass) {
		dGeomID g2 = dGeomTransformGetGeom (g);
		const dReal *pos2 = dGeomGetPosition (g2);
		const dReal *R2 = dGeomGetRotation (g2);
		dVector3 actual_pos;
		dMatrix3 actual_R;
		dMULTIPLY0_331 (actual_pos,R,pos2);
		actual_pos[0] += pos[0];
		actual_pos[1] += pos[1];
		actual_pos[2] += pos[2];
		dMULTIPLY0_333 (actual_R,R,R2);
		drawGeom (g2,actual_pos,actual_R,0);
	    }
	    if (show_aabb) {
		// draw the bounding box for this geom
		dReal aabb[6];
		dGeomGetAABB (g,aabb);
		dVector3 bbpos;
		for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
		dVector3 bbsides;
		for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
		dMatrix3 RI;
		dRSetIdentity (RI);
		dsSetColorAlpha (1,0,0,0.5);
		dsDrawBox (bbpos,RI,bbsides);
	    }
	}
	
	void displaySpace(dSpaceID space)
	{
	    int ngeoms = dSpaceGetNumGeoms(space);
	    for (int i = 0 ; i < ngeoms ; ++i)
	    {
		dGeomID geom = dSpaceGetGeom(space, i);
		drawGeom(geom, NULL, NULL, 0);
	    }
	}
	
	void displaySpaces(void)
	{
	    for (unsigned int i = 0 ; i < m_spaces.size() ; ++i)
	    {
		dsSetColor(m_colors[i].r, m_colors[i].g, m_colors[i].b);
		displaySpace(m_spaces[i]);
	    }	    
	}
	
	void addSpace(dSpaceID space, float r = 0.75, float g = 0.75, float b = 0.75)
	{
	    Color c = {r, g, b};
	    m_colors.push_back(c);
	    m_spaces.push_back(space);
	}
	
	void clear(void)
	{
	    m_spaces.clear();
	    m_colors.clear();
	}
	
    protected:
	struct Color
	{
	    float r, g, b;
	};
	
	std::vector<dSpaceID> m_spaces;
	std::vector<Color>    m_colors;
	
    };
    
}
#endif
