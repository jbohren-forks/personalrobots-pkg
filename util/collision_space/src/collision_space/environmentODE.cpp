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

#include <collision_space/environmentODE.h>
#include <cassert>

void collision_space::EnvironmentModelODE::KinematicModelODE::build(robot_desc::URDF &model, const char *group)
{
    robot_models::KinematicModel::build(model, group);
    assert(m_space);
    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
	buildODEGeoms(m_robots[i]);
}

void collision_space::EnvironmentModelODE::KinematicModelODE::setGeomPose(dGeomID geom, libTF::Pose3D &pose) const
{
    libTF::Pose3D::Position pos = pose.getPosition();
    dGeomSetPosition(geom, pos.x, pos.y, pos.z);
    libTF::Pose3D::Quaternion quat = pose.getQuaternion();
    dQuaternion q; q[0] = quat.w; q[1] = quat.x; q[2] = quat.y; q[3] = quat.z;
    dGeomSetQuaternion(geom, q);
}

void collision_space::EnvironmentModelODE::KinematicModelODE::updateCollisionPositions(void)
{
    for (unsigned int i = 0 ; i < m_kgeoms.size() ; ++i)
	setGeomPose(m_kgeoms[i]->geom, m_kgeoms[i]->link->globalTrans);
}

void collision_space::EnvironmentModelODE::KinematicModelODE::buildODEGeoms(Robot *robot)
{
    for (unsigned int i = 0 ; i < robot->links.size() ; ++i)
    {
	kGeom *kg = new kGeom();
	kg->link = robot->links[i];
	kg->geom = buildODEGeom(robot->links[i]->geom);
	if (!kg->geom)
	{
	    delete kg;
	    continue;
	}
	m_kgeoms.push_back(kg);
    }
}

dGeomID collision_space::EnvironmentModelODE::KinematicModelODE::buildODEGeom(Geometry *geom)
{
    dGeomID g = NULL;
    
    switch (geom->type)
    {
    case Geometry::SPHERE:
	g = dCreateSphere(m_space, geom->size[0]);
	break;
    case Geometry::BOX:
	g = dCreateBox(m_space, geom->size[0], geom->size[1], geom->size[2]);
	break;
    case Geometry::CYLINDER:
	g = dCreateCylinder(m_space, geom->size[0], geom->size[1]);
	break;
    default:
	break;
    }
    
    return g;
}

dSpaceID collision_space::EnvironmentModelODE::KinematicModelODE::getODESpace(void) const
{
    return m_space;
}

void collision_space::EnvironmentModelODE::KinematicModelODE::setODESpace(dSpaceID space)
{
    m_space = space;
}

unsigned int collision_space::EnvironmentModelODE::KinematicModelODE::getGeomCount(void) const
{
    return m_kgeoms.size();
}

dGeomID collision_space::EnvironmentModelODE::KinematicModelODE::getGeom(unsigned index) const
{
    return m_kgeoms[index]->geom;    
}

void collision_space::EnvironmentModelODE::ODECollide2::registerSpace(dSpaceID space)
{
    int n = dSpaceGetNumGeoms(space);
    for (int i = 0 ; i < n ; ++i)
	registerGeom(dSpaceGetGeom(space, i));
}

void collision_space::EnvironmentModelODE::ODECollide2::registerGeom(dGeomID geom)
{
    Geom g;
    g.id = geom;
    dGeomGetAABB(geom, g.aabb);
    m_geoms.push_back(g);
    m_setup = false;
}
	
void collision_space::EnvironmentModelODE::ODECollide2::clear(void)
{
    m_geoms.clear();
    m_setup = false;
}

void collision_space::EnvironmentModelODE::ODECollide2::setup(void)
{
    if (!m_setup)
    {
	sort(m_geoms.begin(), m_geoms.end(), SortByXYZLow());
	m_setup = true;
    }	    
}

void collision_space::EnvironmentModelODE::ODECollide2::collide(dGeomID geom, void *data, dNearCallback *nearCallback)
{
    assert(m_setup);
    
    Geom g;
    g.id = geom;
    dGeomGetAABB(geom, g.aabb);
    
    std::vector<Geom>::iterator pos = lower_bound(m_geoms.begin(), m_geoms.end(), g, SortByX());
    
    /* pos now identifies the first geom which has an AABB that
       could overlap the AABB of geom on the X axis */
    
    while (pos != m_geoms.end())
    {
	/* we no longer overlap on X */
	if (pos->aabb[0] > g.aabb[1])
	    break;
	
	/* if the boxes are not disjoint along Y, Z, check further */
	if (!(pos->aabb[2] > g.aabb[3] ||
	      pos->aabb[3] < g.aabb[2] ||
	      pos->aabb[4] > g.aabb[5] ||
	      pos->aabb[5] < g.aabb[4]))
	    dSpaceCollide2(geom, pos->id, NULL, nearCallback);
	pos++;
    }
}

struct CollisionData
{
    bool collides;
};
    
static void nearCallbackFn(void *data, dGeomID o1, dGeomID o2)
{
    static const int MAX_CONTACTS = 1;    
    dContact contact[MAX_CONTACTS];
    int numc = dCollide (o1, o2, MAX_CONTACTS,
			 &contact[0].geom, sizeof(dContact));
    if (numc)
	reinterpret_cast<CollisionData*>(data)->collides = true;
}

bool collision_space::EnvironmentModelODE::isCollision(void)
{
    CollisionData cdata;
    cdata.collides = false;
    m_collide2.setup();
    for (int i = m_modelODE.getGeomCount() - 1 ; i >= 0 && !cdata.collides ; --i)
	m_collide2.collide(m_modelODE.getGeom(i), reinterpret_cast<void*>(&cdata), nearCallbackFn);
    return cdata.collides;
}

void collision_space::EnvironmentModelODE::addPointCloud(unsigned int n, const double *points, double radius)
{
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	unsigned int i3 = i * 3;
	dGeomID g = dCreateSphere(m_space, radius);
	dGeomSetPosition(g, points[i3], points[i3 + 1], points[i3 + 2]);
	m_collide2.registerGeom(g);
    }
    m_collide2.setup();
}
