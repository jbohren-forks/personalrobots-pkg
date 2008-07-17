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

#include <collisionspace/environmentSOLID.h>

bool EnvironmentModelSOLID::isCollision(void)
{
    return dtTest();
}

void EnvironmentModelSOLID::addPointCloud(unsigned int n, const double *points, double radius)
{
    Object *obj = new Object();
    obj->shape = dtNewComplexShape();

    for (unsigned int i = 0 ; i < n ; ++i)
    {
	unsigned int i3 = i * 3;
	double x = points[i3    ];
	double y = points[i3 + 1];
	double z = points[i3 + 2];
	
	dtBegin(DT_SIMPLEX);
	dtVertex(x,                   y, z + radius);
	dtVertex(x - radius, y - radius, z - radius);
	dtVertex(x + radius, y - radius, z - radius);
	dtEnd();

	dtBegin(DT_SIMPLEX);
	dtVertex(x - radius, y - radius, z - radius);
	dtVertex(x + radius, y - radius, z - radius);
	dtVertex(x + radius, y + radius, z - radius);
	dtEnd();

	dtBegin(DT_SIMPLEX);
	dtVertex(x,                   y, z + radius);
	dtVertex(x + radius, y - radius, z - radius);
	dtVertex(x + radius, y + radius, z - radius);
	dtEnd();

	dtBegin(DT_SIMPLEX);
	dtVertex(x,                   y, z + radius);
	dtVertex(x + radius, y - radius, z - radius);
	dtVertex(x - radius, y - radius, z - radius);
	dtEnd();
    }
    
    dtEndComplexShape();
    dtCreateObject(obj->obj, obj->shape);
}

void EnvironmentModelSOLID::KinematicModelSOLID::build(URDF &model, const char *group)
{
    KinematicModel::build(model, group);
    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
	buildSOLIDShapes(m_robots[i]);  
    dtDisableCaching();
}

void EnvironmentModelSOLID::KinematicModelSOLID::buildSOLIDShapes(Robot *robot)
{
    for (unsigned int i = 0 ; i < robot->links.size() ; ++i)
    {
	kShape *ks = new kShape();
	ks->link = robot->links[i];
	ks->obj->shape = buildSOLIDShape(robot->links[i]->geom);
	if (!ks->obj->shape)
	{
	    delete ks;
	    continue;
	}
	
	dtCreateObject(ks->obj->obj, ks->obj->shape);	
	m_kshapes.push_back(ks);
    }
}

DtShapeRef EnvironmentModelSOLID::KinematicModelSOLID::buildSOLIDShape(Geometry *geom)
{
    DtShapeRef g = NULL;
    
    switch (geom->type)
    {
    case Geometry::SPHERE:
	g = dtSphere(geom->size[0]);
	break;
    case Geometry::BOX:
	g = dtBox(geom->size[0], geom->size[1], geom->size[2]);
	break;
    case Geometry::CYLINDER:
	g = dtCylinder(geom->size[0], geom->size[1]);
	break;
    default:
	break;
    }
    
    return g;
}

void EnvironmentModelSOLID::KinematicModelSOLID::updateCollisionPositions(void)
{
    for (unsigned int i = 0 ; i < m_kshapes.size() ; ++i)
    {
	dtSelectObject(m_kshapes[i]->obj->obj);
	libTF::Pose3D::Position pos;
	m_kshapes[i]->link->globalTrans.getPosition(pos);
	libTF::Pose3D::Quaternion quat;
	m_kshapes[i]->link->globalTrans.getQuaternion(quat);
	dtLoadIdentity();
	dtTranslate(pos.x, pos.y, pos.z);
	dtRotate(quat.x, quat.y, quat.z, quat.w);	
    }
}

unsigned int EnvironmentModelSOLID::KinematicModelSOLID::getObjectCount(void) const
{
    return m_kshapes.size();
}

EnvironmentModelSOLID::Object* EnvironmentModelSOLID::KinematicModelSOLID::getObject(unsigned int index) const
{
    return m_kshapes[index]->obj;
}
