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

void collision_space::EnvironmentModelODE::freeMemory(void)
{ 
    for (unsigned int i = 0 ; i < m_kgeoms.size() ; ++i)
    {
	for (unsigned int j = 0 ; j < m_kgeoms[i].g.size() ; ++j)
	    delete m_kgeoms[i].g[j];
	dSpaceDestroy(m_kgeoms[i].s);
    }    
    if (m_space)
	dSpaceDestroy(m_space);
}

unsigned int collision_space::EnvironmentModelODE::addRobotModel(planning_models::KinematicModel *model)
{
    unsigned int id = collision_space::EnvironmentModel::addRobotModel(model);

    if (m_kgeoms.size() <= id)
    {
	m_kgeoms.resize(id + 1);
	m_kgeoms[id].s = dHashSpaceCreate(0);	
    }
    
    for (unsigned int j = 0 ; j < m_models[id]->getRobotCount() ; ++j)
    {
	planning_models::KinematicModel::Robot *robot = m_models[id]->getRobot(j);
	for (unsigned int i = 0 ; i < robot->links.size() ; ++i)
	{
	    kGeom *kg = new kGeom();
	    kg->link = robot->links[i];
	    planning_models::KinematicModel::Geometry *geom = robot->links[i]->geom;
	    dGeomID g = NULL;
	    switch (geom->type)
	    {
	    case planning_models::KinematicModel::Geometry::SPHERE:
		g = dCreateSphere(m_kgeoms[id].s, geom->size[0]);
		break;
	    case planning_models::KinematicModel::Geometry::BOX:
		g = dCreateBox(m_kgeoms[id].s, geom->size[0], geom->size[1], geom->size[2]);
		break;
	    case planning_models::KinematicModel::Geometry::CYLINDER:
		g = dCreateCylinder(m_kgeoms[id].s, geom->size[0], geom->size[1]);
		break;
	    default:
		break;
	    }
	    if (g)
	    {
		kg->geom = g;
		m_kgeoms[id].g.push_back(kg);
	    }
	    else
		delete kg;
	}
    }
    return id;
}

void collision_space::EnvironmentModelODE::updateRobotModel(unsigned int model_id)
{ 
    const unsigned int n = m_kgeoms[model_id].g.size();
    
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	libTF::Pose3D &pose = m_kgeoms[model_id].g[i]->link->globalTrans;
	dGeomID        geom = m_kgeoms[model_id].g[i]->geom;
	
	libTF::Pose3D::Position pos = pose.getPosition();
	dGeomSetPosition(geom, pos.x, pos.y, pos.z);
	libTF::Pose3D::Quaternion quat = pose.getQuaternion();
	dQuaternion q; q[0] = quat.w; q[1] = quat.x; q[2] = quat.y; q[3] = quat.z;
	dGeomSetQuaternion(geom, q);
    }    
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
	    dSpaceCollide2(geom, pos->id, data, nearCallback);
	pos++;
    }
}

dSpaceID collision_space::EnvironmentModelODE::getODESpace(void) const
{
    return m_space;
}

dSpaceID collision_space::EnvironmentModelODE::getModelODESpace(unsigned int model_id) const
{
    return m_kgeoms[model_id].s;    
}

struct CollisionData
{
    bool collides;
};

static void nearCallbackFn(void *data, dGeomID o1, dGeomID o2)
{
    bool &coll = reinterpret_cast<CollisionData*>(data)->collides;
    if (!coll)
    {
	static const int MAX_CONTACTS = 1;    
	dContact contact[MAX_CONTACTS];
	int numc = dCollide (o1, o2, MAX_CONTACTS,
			     &contact[0].geom, sizeof(dContact));
	if (numc)
	    coll = true;
    }
}

bool collision_space::EnvironmentModelODE::isCollision(unsigned int model_id)
{
    CollisionData cdata;
    cdata.collides = false;
    m_collide2.setup();
    for (int i = m_kgeoms[model_id].g.size() - 1 ; i >= 0 && !cdata.collides ; --i)
	m_collide2.collide(m_kgeoms[model_id].g[i]->geom, reinterpret_cast<void*>(&cdata), nearCallbackFn);
    
    if (m_selfCollision && !cdata.collides)
	dSpaceCollide(m_kgeoms[model_id].s, reinterpret_cast<void*>(&cdata), nearCallbackFn);
    
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

void collision_space::EnvironmentModelODE::clearObstacles(void)
{
    m_collide2.clear();
    freeMemory();
    m_space = dHashSpaceCreate(0);
    m_collide2.setup();
}
