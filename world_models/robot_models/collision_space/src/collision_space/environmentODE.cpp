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

#include <collision_space/environmentODE.h>
#include <algorithm>
#include <map>

void collision_space::EnvironmentModelODE::freeMemory(void)
{ 
    for (unsigned int i = 0 ; i < m_kgeoms.size() ; ++i)
    {
	for (unsigned int j = 0 ; j < m_kgeoms[i].geom.size() ; ++j)
	    delete m_kgeoms[i].geom[j];
	dSpaceDestroy(m_kgeoms[i].space);
    }    
    if (m_space)
	dSpaceDestroy(m_space);
    if (m_spaceBasicGeoms)
	dSpaceDestroy(m_spaceBasicGeoms);
}

unsigned int collision_space::EnvironmentModelODE::addRobotModel(planning_models::KinematicModel *model, const std::vector<std::string> &links)
{
    unsigned int id = collision_space::EnvironmentModel::addRobotModel(model, links);
    
    if (m_kgeoms.size() <= id)
    {
	m_kgeoms.resize(id + 1);
	m_kgeoms[id].space = dHashSpaceCreate(0);	
    }

    std::map<std::string, bool> exists;
    for (unsigned int i = 0 ; i < links.size() ; ++i)
	exists[links[i]] = true;
    
    for (unsigned int j = 0 ; j < m_models[id]->getRobotCount() ; ++j)
    {
	planning_models::KinematicModel::Robot *robot = m_models[id]->getRobot(j);
	for (unsigned int i = 0 ; i < robot->links.size() ; ++i)
	{
	    /* skip this link if we have no geometry or if the link
	       name is not specified as enabled for collision
	       checking */
	    if (!robot->links[i]->shape)
		continue;
	    if (exists.find(robot->links[i]->name) == exists.end())
		continue;
	    
	    kGeom *kg = new kGeom();
	    kg->link = robot->links[i];
	    kg->enabled = true;
	    dGeomID g = createODEGeom(m_kgeoms[id].space, robot->links[i]->shape);
	    if (g)
	    {
		kg->geom = g;
		m_kgeoms[id].geom.push_back(kg);
	    }
	    else
		delete kg;
	}
    }
    return id;
}

dGeomID collision_space::EnvironmentModelODE::createODEGeom(dSpaceID space, planning_models::KinematicModel::Shape *shape) const
{
    dGeomID g = NULL;
    switch (shape->type)
    {
    case planning_models::KinematicModel::Shape::SPHERE:
	{
	    g = dCreateSphere(space, static_cast<planning_models::KinematicModel::Sphere*>(shape)->radius);
	}
	break;
    case planning_models::KinematicModel::Shape::BOX:
	{
	    const double *size = static_cast<planning_models::KinematicModel::Box*>(shape)->size;
	    g = dCreateBox(space, size[0], size[1], size[2]);
	}	
	break;
    case planning_models::KinematicModel::Shape::CYLINDER:
	{
	    g = dCreateCylinder(space, static_cast<planning_models::KinematicModel::Cylinder*>(shape)->radius,
				static_cast<planning_models::KinematicModel::Cylinder*>(shape)->length);
	}
	break;
    default:
	break;
    }
    return g;
}

void collision_space::EnvironmentModelODE::updateRobotModel(unsigned int model_id)
{ 
    const unsigned int n = m_kgeoms[model_id].geom.size();
    
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	btTransform &pose = m_kgeoms[model_id].geom[i]->link->globalTrans;
	dGeomID      geom = m_kgeoms[model_id].geom[i]->geom;
	
	btVector3 pos = pose.getOrigin();
	dGeomSetPosition(geom, pos.getX(), pos.getY(), pos.getZ());
	btQuaternion quat = pose.getRotation();
	dQuaternion q; 
	q[0] = quat.getW(); q[1] = quat.getX(); q[2] = quat.getY(); q[3] = quat.getZ();
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
    Geom *g = new Geom();
    g->id = geom;
    dGeomGetAABB(geom, g->aabb);
    m_geomsX.push_back(g);
    m_geomsY.push_back(g);
    m_geomsZ.push_back(g);
    m_setup = false;
}
	
void collision_space::EnvironmentModelODE::ODECollide2::clear(void)
{
    for (unsigned int i = 0 ; i < m_geomsX.size() ; ++i)
	delete m_geomsX[i];
    m_geomsX.clear();
    m_geomsY.clear();
    m_geomsZ.clear();
    m_setup = false;
}

void collision_space::EnvironmentModelODE::ODECollide2::setup(void)
{
    if (!m_setup)
    {
	std::sort(m_geomsX.begin(), m_geomsX.end(), SortByXLow());
	std::sort(m_geomsY.begin(), m_geomsY.end(), SortByYLow());
	std::sort(m_geomsZ.begin(), m_geomsZ.end(), SortByZLow());
	m_setup = true;
    }	    
}

void collision_space::EnvironmentModelODE::ODECollide2::checkColl(std::vector<Geom*>::iterator posStart, std::vector<Geom*>::iterator posEnd,
								  Geom *g, void *data, dNearCallback *nearCallback)
{
    /* posStart now identifies the first geom which has an AABB
       that could overlap the AABB of geom on the X axis. posEnd
       identifies the first one that cannot overlap. */
    
    while (posStart < posEnd)
    {
	/* if the boxes are not disjoint along Y, Z, check further */
	if (!((*posStart)->aabb[2] > g->aabb[3] ||
	      (*posStart)->aabb[3] < g->aabb[2] ||
	      (*posStart)->aabb[4] > g->aabb[5] ||
	      (*posStart)->aabb[5] < g->aabb[4]))
	    dSpaceCollide2(g->id, (*posStart)->id, data, nearCallback);
	posStart++;
    }
}

void collision_space::EnvironmentModelODE::ODECollide2::collide(dGeomID geom, void *data, dNearCallback *nearCallback)
{
    static const int CUTOFF = 100;

    assert(m_setup);

    Geom g;
    g.id = geom;
    dGeomGetAABB(geom, g.aabb);
    
    std::vector<Geom*>::iterator posStart1 = std::lower_bound(m_geomsX.begin(), m_geomsX.end(), &g, SortByXTest());
    if (posStart1 != m_geomsX.end())
    {
	std::vector<Geom*>::iterator posEnd1 = std::upper_bound(posStart1, m_geomsX.end(), &g, SortByXTest());
	int                          d1      = posEnd1 - posStart1;
	
	/* Doing two binary searches on the sorted-by-y array takes
	   log(n) time, which should be around 12 steps. Each step
	   should be just a few ops, so a cut-off like 100 is
	   appropriate. */
	if (d1 > CUTOFF)
	{
	    std::vector<Geom*>::iterator posStart2 = std::lower_bound(m_geomsY.begin(), m_geomsY.end(), &g, SortByYTest());
	    if (posStart2 != m_geomsY.end())
	    {
		std::vector<Geom*>::iterator posEnd2 = std::upper_bound(posStart2, m_geomsY.end(), &g, SortByYTest());
		int                          d2      = posEnd2 - posStart2;
		
		if (d2 > CUTOFF)
		{
		    std::vector<Geom*>::iterator posStart3 = std::lower_bound(m_geomsZ.begin(), m_geomsZ.end(), &g, SortByZTest());
		    if (posStart3 != m_geomsZ.end())
		    {
			std::vector<Geom*>::iterator posEnd3 = std::upper_bound(posStart3, m_geomsZ.end(), &g, SortByZTest());
			int                          d3      = posEnd3 - posStart3;
			if (d3 > CUTOFF)
			{
			    if (d3 <= d2 && d3 <= d1)
				checkColl(posStart3, posEnd3, &g, data, nearCallback);
			    else
				if (d2 <= d3 && d2 <= d1)
				    checkColl(posStart2, posEnd2, &g, data, nearCallback);
				else
				    checkColl(posStart1, posEnd1, &g, data, nearCallback);
			}
			else
			    checkColl(posStart3, posEnd3, &g, data, nearCallback);   
		    }
		}
		else
		    checkColl(posStart2, posEnd2, &g, data, nearCallback);   
	    }
	}
	else 
	    checkColl(posStart1, posEnd1, &g, data, nearCallback);
    }
}

dSpaceID collision_space::EnvironmentModelODE::getODESpace(void) const
{
    return m_space;
}

dSpaceID collision_space::EnvironmentModelODE::getODEBasicGeomSpace(void) const
{
    return m_spaceBasicGeoms;
}

dSpaceID collision_space::EnvironmentModelODE::getModelODESpace(unsigned int model_id) const
{
    return m_kgeoms[model_id].space;    
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
    
    /* check self collision */
    if (m_selfCollision)
    {
	for (int i = m_kgeoms[model_id].selfCollision.size() - 1 ; i >= 0 ; --i)
	{
	    const std::vector<unsigned int> &vec = m_kgeoms[model_id].selfCollision[i];
	    unsigned int n = vec.size();
	    
	    for (unsigned int j = 0 ; j < n ; ++j)
		for (unsigned int k = j + 1 ; k < n ; ++k)
		{
		    // dSpaceCollide2 expects AABBs to be computed, so
		    // we force that by calling dGeomGetAABB. Since we
		    // get the data anyway, we attempt to speed things
		    // up using it.
		    dGeomID g1 = m_kgeoms[model_id].geom[vec[j]]->geom;
		    dGeomID g2 = m_kgeoms[model_id].geom[vec[k]]->geom;
		    
		    dReal aabb1[6], aabb2[6];		    
		    dGeomGetAABB(g1, aabb1);
		    dGeomGetAABB(g2, aabb2);
		    
		    if (!(aabb1[2] > aabb2[3] ||
			  aabb1[3] < aabb2[2] ||
			  aabb1[4] > aabb2[5] ||
			  aabb1[5] < aabb2[4])) 
			dSpaceCollide2(g1, g2, reinterpret_cast<void*>(&cdata), nearCallbackFn);

		    if (cdata.collides)
			goto OUT1;
		}
	}
    }
    
    /* check collision with standalone ode bodies */
 OUT1:

    if (!cdata.collides)
    {
	for (int i = m_kgeoms[model_id].geom.size() - 1 ; i >= 0 ; --i)
	{
	    /* skip disabled bodies */
	    if (!m_kgeoms[model_id].geom[i]->enabled)
		continue;
	    dGeomID g1 = m_kgeoms[model_id].geom[i]->geom;
	    dReal aabb1[6];
	    dGeomGetAABB(g1, aabb1);
	    for (int j = m_basicGeoms.size() - 1 ; j >= 0 ; --j)
	    {
		dGeomID g2 = m_basicGeoms[j];
		dReal aabb2[6];
		dGeomGetAABB(g2, aabb2);

		if (!(aabb1[2] > aabb2[3] ||
		      aabb1[3] < aabb2[2] ||
		      aabb1[4] > aabb2[5] ||
		      aabb1[5] < aabb2[4]))
		    dSpaceCollide2(g1, g2, reinterpret_cast<void*>(&cdata), nearCallbackFn);
		
		if (cdata.collides)
		    goto OUT2;
	    }
	}	
    }
    
    /* check collision with pointclouds */
 OUT2:

    if (!cdata.collides)
    {
	m_collide2.setup();
	for (int i = m_kgeoms[model_id].geom.size() - 1 ; i >= 0 && !cdata.collides ; --i)
	    if (m_kgeoms[model_id].geom[i]->enabled)
		m_collide2.collide(m_kgeoms[model_id].geom[i]->geom, reinterpret_cast<void*>(&cdata), nearCallbackFn);
    }
    
    return cdata.collides;
}

void collision_space::EnvironmentModelODE::addPointCloud(unsigned int n, const double *points)
{
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	unsigned int i4 = i * 4;
	dGeomID g = dCreateSphere(m_space, points[i4 + 3]);
	dGeomSetPosition(g, points[i4], points[i4 + 1], points[i4 + 2]);
	m_collide2.registerGeom(g);
    }
    m_collide2.setup();
}

void collision_space::EnvironmentModelODE::addStaticPlane(double a, double b, double c, double d)
{
    dGeomID g = dCreatePlane(m_spaceBasicGeoms, a, b, c, d);
    m_basicGeoms.push_back(g);
}

void collision_space::EnvironmentModelODE::clearObstacles(void)
{
    m_collide2.clear();
    if (m_space)
	dSpaceDestroy(m_space);
    m_space = dHashSpaceCreate(0);
    m_collide2.setup();
}

void collision_space::EnvironmentModelODE::addSelfCollisionGroup(unsigned int model_id, std::vector<std::string> &links)
{
    if (model_id < m_kgeoms.size())
    {
	unsigned int pos = m_kgeoms[model_id].selfCollision.size();
	m_kgeoms[model_id].selfCollision.resize(pos + 1);
	for (unsigned int i = 0 ; i < links.size() ; ++i)
	{
	    for (unsigned int j = 0 ; j < m_kgeoms[model_id].geom.size() ; ++j)
		if (m_kgeoms[model_id].geom[j]->link->name == links[i])
		    m_kgeoms[model_id].selfCollision[pos].push_back(j);
	}
    }
}

void collision_space::EnvironmentModelODE::setCollisionCheck(unsigned int model_id, std::string &link, bool state)
{ 
    if (model_id < m_kgeoms.size())
    {
	for (unsigned int j = 0 ; j < m_kgeoms[model_id].geom.size() ; ++j)
	{
	    if (m_kgeoms[model_id].geom[j]->link->name == link)
	    {
		m_kgeoms[model_id].geom[j]->enabled = state;
		break;
	    }
	}
    }
}
