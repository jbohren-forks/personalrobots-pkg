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

#include <robotmodels/kinematicODE.h>

/*

#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawTriangle dsDrawTriangleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawConvex dsDrawConvexD
#endif

namespace gz2ode
{
    
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
}

void gz2ode::displaySpace(dSpaceID space)
{
    int ngeoms = dSpaceGetNumGeoms(space);
    for (int i = 0 ; i < ngeoms ; ++i)
    {
	dGeomID geom = dSpaceGetGeom(space, i);
	int cls = dGeomGetClass(geom);
	switch (cls)
	{
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
*/
int main(int argc, char **argv)
{
    
    URDF model;
    model.load("/u/isucan/ros/ros-pkg/drivers/robot/pr2/pr2Core/include/pr2Core/pr2.xml");
    
    KinematicModelODE km;
    km.build(model, "leftArm");
    printf("number of robots = %d\n", km.getRobotCount());
    
    KinematicModel::Robot *r = km.getRobot(0);    
    printf("state dimension = %d\n", r->stateDimension);
    
    return 0;    
}
