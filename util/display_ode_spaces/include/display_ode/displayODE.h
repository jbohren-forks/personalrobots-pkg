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
	
	void drawSphere(dGeomID geom)
	{
	    dReal radius = dGeomSphereGetRadius(geom);
	    dsDrawSphere(dGeomGetPosition(geom), dGeomGetRotation(geom), radius);    
	}
	
	void drawBox(dGeomID geom)
	{
	    dVector3 sides;	
	    dGeomBoxGetLengths(geom, sides);	
	    dsDrawBox(dGeomGetPosition(geom), dGeomGetRotation(geom), sides);
	}
	
	void drawCylinder(dGeomID geom)
	{
	    dReal radius, length;
	    dGeomCylinderGetParams(geom, &radius, &length);	
	    dsDrawCylinder(dGeomGetPosition(geom), dGeomGetRotation(geom), length, radius);
	}
	
	void displaySpace(dSpaceID space)
	{
	    int ngeoms = dSpaceGetNumGeoms(space);
	    for (int i = 0 ; i < ngeoms ; ++i)
	    {
		dGeomID geom = dSpaceGetGeom(space, i);
		int cls = dGeomGetClass(geom);
		switch (cls)
		{
		case dSphereClass:
		    drawSphere(geom);
		    break;
		case dBoxClass:
		    drawBox(geom);
		    break;
		case dCylinderClass:
		    drawCylinder(geom);
		    break;	    
		default:
		    printf("Geometry class %d not yet implemented\n", cls);	    
		    break;	    
		}
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
