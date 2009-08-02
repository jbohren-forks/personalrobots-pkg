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

#include "collision_space/environmentODE.h"
#include <boost/thread.hpp>
#include <cassert>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <map>

static int          ODEInitCount = 0;
static boost::mutex ODEInitCountLock;

static std::map<boost::thread::id, int> ODEThreadMap;
static boost::mutex                     ODEThreadMapLock;

collision_space::EnvironmentModelODE::EnvironmentModelODE(void) : EnvironmentModel()
{
    ODEInitCountLock.lock();
    if (ODEInitCount == 0)
    {
	m_msg.message("Initializing ODE");
	dInitODE2(0);
    }
    ODEInitCount++;
    ODEInitCountLock.unlock();
    
    checkThreadInit();

    m_modelGeom.space = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XZY);
}

collision_space::EnvironmentModelODE::~EnvironmentModelODE(void)
{
    freeMemory();
    ODEInitCountLock.lock();
    ODEInitCount--;
    if (ODEInitCount == 0)
    {
	m_msg.message("Closing ODE");
	dCloseODE();
    }
    ODEInitCountLock.unlock();
}

void collision_space::EnvironmentModelODE::checkThreadInit(void) const
{
    boost::thread::id id = boost::this_thread::get_id();
    ODEThreadMapLock.lock();
    if (ODEThreadMap.find(id) == ODEThreadMap.end())
    {
	ODEThreadMap[id] = 1;
	m_msg.message("Initializing new thread (%d total)", (int)ODEThreadMap.size());
	dAllocateODEDataForThread(dAllocateMaskAll);
    }
    ODEThreadMapLock.unlock();
}

void collision_space::EnvironmentModelODE::freeMemory(void)
{ 
    for (unsigned int j = 0 ; j < m_modelGeom.linkGeom.size() ; ++j)
	delete m_modelGeom.linkGeom[j];
    if (m_modelGeom.space)
	dSpaceDestroy(m_modelGeom.space);
    for (std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.begin() ; it != m_collNs.end() ; ++it)
	delete it->second;
}

void collision_space::EnvironmentModelODE::setRobotModel(const boost::shared_ptr<planning_models::KinematicModel> &model, const std::vector<std::string> &links, double scale, double padding)
{
    collision_space::EnvironmentModel::setRobotModel(model, links, scale, padding);
    createODERobotModel();
}

void collision_space::EnvironmentModelODE::createODERobotModel(void)
{
    for (unsigned int i = 0 ; i < m_collisionLinks.size() ; ++i)
    {
	/* skip this link if we have no geometry or if the link
	   name is not specified as enabled for collision
	   checking */
	planning_models::KinematicModel::Link *link = m_robotModel->getLink(m_collisionLinks[i]);
	if (!link || !link->shape)
	    continue;
	
	kGeom *kg = new kGeom();
	kg->link = link;
	kg->enabled = true;
	kg->index = i;
	dGeomID g = createODEGeom(m_modelGeom.space, m_modelGeom.storage, link->shape, m_robotScale, m_robotPadd);
	assert(g);
	dGeomSetData(g, reinterpret_cast<void*>(kg));
	kg->geom.push_back(g);
	for (unsigned int k = 0 ; k < kg->link->attachedBodies.size() ; ++k)
	{
	    dGeomID ga = createODEGeom(m_modelGeom.space, m_modelGeom.storage, kg->link->attachedBodies[k]->shape, m_robotScale, m_robotPadd);
	    assert(ga);
	    dGeomSetData(ga, reinterpret_cast<void*>(kg));
	    kg->geom.push_back(ga);
	}
	m_modelGeom.linkGeom.push_back(kg);
    } 
}

dGeomID collision_space::EnvironmentModelODE::createODEGeom(dSpaceID space, ODEStorage &storage, const shapes::StaticShape *shape)
{
    dGeomID g = NULL;
    switch (shape->type)
    {
    case shapes::PLANE:
	{
	    const shapes::Plane *p = static_cast<const shapes::Plane*>(shape);
	    g = dCreatePlane(space, p->a, p->b, p->c, p->d);
	}
	break;
    default:
	break;
    }
    return g;
}

dGeomID collision_space::EnvironmentModelODE::createODEGeom(dSpaceID space, ODEStorage &storage, const shapes::Shape *shape, double scale, double padding)
{
    dGeomID g = NULL;
    switch (shape->type)
    {
    case shapes::SPHERE:
	{
	    g = dCreateSphere(space, static_cast<const shapes::Sphere*>(shape)->radius * scale + padding);
	}
	break;
    case shapes::BOX:
	{
	    const double *size = static_cast<const shapes::Box*>(shape)->size;
	    g = dCreateBox(space, size[0] * scale + padding * 2.0, size[1] * scale + padding * 2.0, size[2] * scale + padding * 2.0);
	}	
	break;
    case shapes::CYLINDER:
	{
	    g = dCreateCylinder(space, static_cast<const shapes::Cylinder*>(shape)->radius * scale + padding,
				static_cast<const shapes::Cylinder*>(shape)->length * scale + padding * 2.0);
	}
	break;
    case shapes::MESH:
	{
	    const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
	    if (mesh->vertexCount > 0 && mesh->triangleCount > 0)
	    {		
		// copy indices for ODE
		int icount = mesh->triangleCount * 3;
		dTriIndex *indices = new dTriIndex[icount];
		for (int i = 0 ; i < icount ; ++i)
		    indices[i] = mesh->triangles[i];
		
		// copt vertices for ODE
		double *vertices = new double[mesh->vertexCount* 3];
		double sx = 0.0, sy = 0.0, sz = 0.0;
		for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
		{
		    unsigned int i3 = i * 3;
		    vertices[i3] = mesh->vertices[i3];
		    vertices[i3 + 1] = mesh->vertices[i3 + 1];
		    vertices[i3 + 2] = mesh->vertices[i3 + 2];
		    sx += vertices[i3];
		    sy += vertices[i3 + 1];
		    sz += vertices[i3 + 2];
		}
		// the center of the mesh
		sx /= (double)mesh->vertexCount;
		sy /= (double)mesh->vertexCount;
		sz /= (double)mesh->vertexCount;

		// scale the mesh
		for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
		{
		    unsigned int i3 = i * 3;
		    
		    // vector from center to the vertex
		    double dx = vertices[i3] - sx;
		    double dy = vertices[i3 + 1] - sy;
		    double dz = vertices[i3 + 2] - sz;
		    
		    // length of vector
		    double norm = sqrt(dx * dx + dy * dy + dz * dz);
		    
		    // the new distance of the vertex from the center
		    double fact = scale + padding/norm;
		    vertices[i3] = sx + dx * fact;
		    vertices[i3 + 1] = sy + dy * fact;
		    vertices[i3 + 2] = sz + dz * fact;		    
		}
		
		dTriMeshDataID data = dGeomTriMeshDataCreate();
		dGeomTriMeshDataBuildDouble(data, vertices, sizeof(double) * 3, mesh->vertexCount, indices, icount, sizeof(dTriIndex) * 3);
		g = dCreateTriMesh(space, data, NULL, NULL, NULL);
		unsigned int p = storage.mesh.size();
		storage.mesh.resize(p + 1);
		storage.mesh[p].vertices = vertices;
		storage.mesh[p].indices = indices;
		storage.mesh[p].data = data;
		storage.mesh[p].nVertices = mesh->vertexCount;
		storage.mesh[p].nIndices = icount;
	    }
	}
	
    default:
	break;
    }
    return g;
}

void collision_space::EnvironmentModelODE::updateGeom(dGeomID geom,  const btTransform &pose) const
{
    btVector3 pos = pose.getOrigin();
    dGeomSetPosition(geom, pos.getX(), pos.getY(), pos.getZ());
    btQuaternion quat = pose.getRotation();
    dQuaternion q; 
    q[0] = quat.getW(); q[1] = quat.getX(); q[2] = quat.getY(); q[3] = quat.getZ();
    dGeomSetQuaternion(geom, q);
}

void collision_space::EnvironmentModelODE::updateAttachedBodies(void)
{
    const unsigned int n = m_modelGeom.linkGeom.size();    
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	kGeom *kg = m_modelGeom.linkGeom[i];
	
	/* clear previosly attached bodies */
	for (unsigned int k = 1 ; k < kg->geom.size() ; ++k)
	    dGeomDestroy(kg->geom[k]);
	kg->geom.resize(1);
	
	/* create new set of attached bodies */	
	const unsigned int nab = kg->link->attachedBodies.size();
	for (unsigned int k = 0 ; k < nab ; ++k)
	{
	    dGeomID ga = createODEGeom(m_modelGeom.space, m_modelGeom.storage, kg->link->attachedBodies[k]->shape, m_robotScale, m_robotPadd);
	    assert(ga);
	    dGeomSetData(ga, reinterpret_cast<void*>(kg));
	    kg->geom.push_back(ga);
	}
    }
}

void collision_space::EnvironmentModelODE::updateRobotModel(void)
{ 
    const unsigned int n = m_modelGeom.linkGeom.size();
    
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	updateGeom(m_modelGeom.linkGeom[i]->geom[0], m_modelGeom.linkGeom[i]->link->globalTrans);
	const unsigned int nab = m_modelGeom.linkGeom[i]->link->attachedBodies.size();
	for (unsigned int k = 0 ; k < nab ; ++k)
	    updateGeom(m_modelGeom.linkGeom[i]->geom[k + 1], m_modelGeom.linkGeom[i]->link->attachedBodies[k]->globalTrans);
    }    
}

bool collision_space::EnvironmentModelODE::ODECollide2::empty(void) const
{
    return m_geomsX.empty();
}

void collision_space::EnvironmentModelODE::ODECollide2::registerSpace(dSpaceID space)
{
    int n = dSpaceGetNumGeoms(space);
    for (int i = 0 ; i < n ; ++i)
	registerGeom(dSpaceGetGeom(space, i));
}

void collision_space::EnvironmentModelODE::ODECollide2::unregisterGeom(dGeomID geom)
{
    setup();
    
    Geom g;
    g.id = geom;
    dGeomGetAABB(geom, g.aabb);
    
    Geom *found = NULL;
    
    std::vector<Geom*>::iterator posStart1 = std::lower_bound(m_geomsX.begin(), m_geomsX.end(), &g, SortByXTest());
    std::vector<Geom*>::iterator posEnd1   = std::upper_bound(posStart1, m_geomsX.end(), &g, SortByXTest());
    while (posStart1 < posEnd1)
    {
	if ((*posStart1)->id == geom)
	{
	    found = *posStart1;
	    m_geomsX.erase(posStart1);
	    break;
	}
	++posStart1;
    }

    std::vector<Geom*>::iterator posStart2 = std::lower_bound(m_geomsY.begin(), m_geomsY.end(), &g, SortByYTest());
    std::vector<Geom*>::iterator posEnd2   = std::upper_bound(posStart2, m_geomsY.end(), &g, SortByYTest());
    while (posStart2 < posEnd2)
    {
	if ((*posStart2)->id == geom)
	{
	    assert(found == *posStart2);
	    m_geomsY.erase(posStart2);
	    break;
	}
	++posStart2;
    }
    
    std::vector<Geom*>::iterator posStart3 = std::lower_bound(m_geomsZ.begin(), m_geomsZ.end(), &g, SortByZTest());
    std::vector<Geom*>::iterator posEnd3   = std::upper_bound(posStart3, m_geomsZ.end(), &g, SortByZTest());
    while (posStart3 < posEnd3)
    {
	if ((*posStart3)->id == geom)
	{
	    assert(found == *posStart3);
	    m_geomsZ.erase(posStart3);
	    break;
	}
	++posStart3;
    }
    
    assert(found);
    delete found;
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

void collision_space::EnvironmentModelODE::ODECollide2::getGeoms(std::vector<dGeomID> &geoms) const
{
    geoms.resize(m_geomsX.size());
    for (unsigned int i = 0 ; i < geoms.size() ; ++i)
	geoms[i] = m_geomsX[i]->id;
}

void collision_space::EnvironmentModelODE::ODECollide2::checkColl(std::vector<Geom*>::const_iterator posStart, std::vector<Geom*>::const_iterator posEnd,
								  Geom *g, void *data, dNearCallback *nearCallback) const
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

void collision_space::EnvironmentModelODE::ODECollide2::collide(dGeomID geom, void *data, dNearCallback *nearCallback) const
{
    static const int CUTOFF = 100;

    assert(m_setup);

    Geom g;
    g.id = geom;
    dGeomGetAABB(geom, g.aabb);
    
    std::vector<Geom*>::const_iterator posStart1 = std::lower_bound(m_geomsX.begin(), m_geomsX.end(), &g, SortByXTest());
    if (posStart1 != m_geomsX.end())
    {
	std::vector<Geom*>::const_iterator posEnd1 = std::upper_bound(posStart1, m_geomsX.end(), &g, SortByXTest());
	int                                d1      = posEnd1 - posStart1;
	
	/* Doing two binary searches on the sorted-by-y array takes
	   log(n) time, which should be around 12 steps. Each step
	   should be just a few ops, so a cut-off like 100 is
	   appropriate. */
	if (d1 > CUTOFF)
	{
	    std::vector<Geom*>::const_iterator posStart2 = std::lower_bound(m_geomsY.begin(), m_geomsY.end(), &g, SortByYTest());
	    if (posStart2 != m_geomsY.end())
	    {
		std::vector<Geom*>::const_iterator posEnd2 = std::upper_bound(posStart2, m_geomsY.end(), &g, SortByYTest());
		int                                d2      = posEnd2 - posStart2;
		
		if (d2 > CUTOFF)
		{
		    std::vector<Geom*>::const_iterator posStart3 = std::lower_bound(m_geomsZ.begin(), m_geomsZ.end(), &g, SortByZTest());
		    if (posStart3 != m_geomsZ.end())
		    {
			std::vector<Geom*>::const_iterator posEnd3 = std::upper_bound(posStart3, m_geomsZ.end(), &g, SortByZTest());
			int                                d3      = posEnd3 - posStart3;
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

namespace collision_space
{
    

    void nearCallbackFn(void *data, dGeomID o1, dGeomID o2)
    {
	EnvironmentModelODE::CollisionData *cdata = reinterpret_cast<EnvironmentModelODE::CollisionData*>(data);
	
	if (cdata->done)
	    return;
	
	if (cdata->selfCollisionTest)
	{
	    dSpaceID s1 = dGeomGetSpace(o1);
	    dSpaceID s2 = dGeomGetSpace(o2);
	    if (s1 == s2 && s1 == cdata->selfSpace)
	    {
		EnvironmentModelODE::kGeom* kg1 = reinterpret_cast<EnvironmentModelODE::kGeom*>(dGeomGetData(o1));
		EnvironmentModelODE::kGeom* kg2 = reinterpret_cast<EnvironmentModelODE::kGeom*>(dGeomGetData(o2));
		if (kg1 && kg2)
		{
		    if (cdata->selfCollisionTest->at(kg1->index)[kg2->index] == false)
			return;
		    else
		    {
			cdata->link1 = kg1->link;
			cdata->link2 = kg2->link;
		    }
		}
	    }
	}
	
	if (cdata->contacts)
	{
	    static const int MAX_CONTACTS = 3;
	    dContact contact[MAX_CONTACTS];
	    int numc = dCollide (o1, o2, MAX_CONTACTS,
				 &contact[0].geom, sizeof(dContact));
	    if (numc)
	    {
		cdata->collides = true;
		for (int i = 0 ; i < numc ; ++i)
		{
		    if (cdata->contacts->size() < cdata->max_contacts)
		    {
			collision_space::EnvironmentModelODE::Contact add;
			
			add.pos.setX(contact[i].geom.pos[0]);
			add.pos.setY(contact[i].geom.pos[1]);
			add.pos.setZ(contact[i].geom.pos[2]);
			
			add.normal.setX(contact[i].geom.normal[0]);
			add.normal.setY(contact[i].geom.normal[1]);
			add.normal.setZ(contact[i].geom.normal[2]);
			
			add.depth = contact[i].geom.depth;
			
			add.link1 = cdata->link1;
			add.link2 = cdata->link2;
			
			cdata->contacts->push_back(add);
		    }
		    else
			break;
		}
	    }
	    if (cdata->contacts->size() >= cdata->max_contacts)
		cdata->done = true;
	}
	else
	{
	    static const int MAX_CONTACTS = 1;    
	    dContact contact[MAX_CONTACTS];
	    int numc = dCollide (o1, o2, MAX_CONTACTS,
				 &contact[0].geom, sizeof(dContact));
	    if (numc)
	    {
		cdata->collides = true;
		cdata->done = true;
	    }
	}
    }
}

bool collision_space::EnvironmentModelODE::getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count)
{
    CollisionData cdata;
    cdata.contacts = &contacts;
    cdata.max_contacts = max_count;
    contacts.clear();
    checkThreadInit();
    testCollision(&cdata);
    return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isCollision(void)
{
    CollisionData cdata;
    checkThreadInit();
    testCollision(&cdata);
    return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isSelfCollision(void)
{
    CollisionData cdata;
    checkThreadInit();
    testSelfCollision(&cdata);
    return cdata.collides;
}

void collision_space::EnvironmentModelODE::testSelfCollision(CollisionData *cdata)
{     
    cdata->selfCollisionTest = &m_selfCollisionTest;
    cdata->selfSpace = m_modelGeom.space;
    dSpaceCollide(m_modelGeom.space, cdata, nearCallbackFn);
}

void collision_space::EnvironmentModelODE::testBodyCollision(CollisionNamespace *cn, CollisionData *cdata)
{ 
    if (cn->collide2.empty())
    {
	// if there is no collide2 structure, then there is a list of geoms
	for (int i = m_modelGeom.linkGeom.size() - 1 ; i >= 0 && !cdata->done ; --i)
	{
	    /* skip disabled bodies */
	    if (!m_modelGeom.linkGeom[i]->enabled)
		continue;
	    const unsigned int ng = m_modelGeom.linkGeom[i]->geom.size();
	    cdata->link1 = m_modelGeom.linkGeom[i]->link;	    
	    
	    for (unsigned int ig = 0 ; ig < ng && !cdata->done ; ++ig)
	    {
		dGeomID g1 = m_modelGeom.linkGeom[i]->geom[ig];
		dReal aabb1[6];
		dGeomGetAABB(g1, aabb1);
		for (int j = cn->geoms.size() - 1 ; j >= 0 ; --j)
		{
		    dGeomID g2 = cn->geoms[j];
		    dReal aabb2[6];
		    dGeomGetAABB(g2, aabb2);
		    
		    if (!(aabb1[2] > aabb2[3] ||
			  aabb1[3] < aabb2[2] ||
			  aabb1[4] > aabb2[5] ||
			  aabb1[5] < aabb2[4]))
			dSpaceCollide2(g1, g2, cdata, nearCallbackFn);
		    
		    if (cdata->collides && m_verbose)
			m_msg.inform("Collision between body in namespace '%s' and link '%s'\n",
				     cn->name.c_str(), m_modelGeom.linkGeom[i]->link->name.c_str());
		}
	    }
	}
    }
    else
    {
	cn->collide2.setup();
	for (int i = m_modelGeom.linkGeom.size() - 1 ; i >= 0 && !cdata->done ; --i)
	    if (m_modelGeom.linkGeom[i]->enabled)
	    {
		const unsigned int ng = m_modelGeom.linkGeom[i]->geom.size();
		cdata->link1 = m_modelGeom.linkGeom[i]->link;	    
		for (unsigned int ig = 0 ; ig < ng && !cdata->done ; ++ig)
		{
		    cn->collide2.collide(m_modelGeom.linkGeom[i]->geom[ig], cdata, nearCallbackFn);
		    if (cdata->collides && m_verbose)
			m_msg.inform("Collision between body in namespace '%s' and link '%s'\n",
				     cn->name.c_str(), m_modelGeom.linkGeom[i]->link->name.c_str());
		}
	    }
    }
}

void collision_space::EnvironmentModelODE::testCollision(CollisionData *cdata)
{
    /* check self collision */
    if (m_selfCollision)
    	testSelfCollision(cdata);
    
    if (!cdata->done)
    {
        cdata->link2 = NULL;
        /* check collision with other ode bodies */
	for (std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.begin() ; it != m_collNs.end() && !cdata->done ; ++it)
	    testBodyCollision(it->second, cdata);
	cdata->done = true;
    }
}

void collision_space::EnvironmentModelODE::addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<btTransform> &poses)
{
    assert(shapes.size() == poses.size());
    std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.find(ns);
    CollisionNamespace* cn = NULL;    
    if (it == m_collNs.end())
    {
	cn = new CollisionNamespace(ns);
	m_collNs[ns] = cn;
    }
    else
	cn = it->second;

    unsigned int n = shapes.size();
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	dGeomID g = createODEGeom(cn->space, cn->storage, shapes[i], 1.0, 0.0);
	assert(g);
	dGeomSetData(g, reinterpret_cast<void*>(shapes[i]));
	updateGeom(g, poses[i]);
	cn->collide2.registerGeom(g);
	m_objects->addObject(ns, shapes[i], poses[i]);
    }
    cn->collide2.setup();
}

void collision_space::EnvironmentModelODE::addObject(const std::string &ns, shapes::Shape *shape, const btTransform &pose)
{
    std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.find(ns);
    CollisionNamespace* cn = NULL;    
    if (it == m_collNs.end())
    {
	cn = new CollisionNamespace(ns);
	m_collNs[ns] = cn;
    }
    else
	cn = it->second;
    
    dGeomID g = createODEGeom(cn->space, cn->storage, shape, 1.0, 0.0);
    assert(g);
    dGeomSetData(g, reinterpret_cast<void*>(shape));
    updateGeom(g, pose);
    cn->geoms.push_back(g);
    m_objects->addObject(ns, shape, pose);
}

void collision_space::EnvironmentModelODE::addObject(const std::string &ns, shapes::StaticShape* shape)
{   
    std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.find(ns);
    CollisionNamespace* cn = NULL;    
    if (it == m_collNs.end())
    {
	cn = new CollisionNamespace(ns);
	m_collNs[ns] = cn;
    }
    else
	cn = it->second;

    dGeomID g = createODEGeom(cn->space, cn->storage, shape);
    assert(g);
    dGeomSetData(g, reinterpret_cast<void*>(shape));
    cn->geoms.push_back(g);
    m_objects->addObject(ns, shape);
}

void collision_space::EnvironmentModelODE::clearObjects(void)
{
    for (std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.begin() ; it != m_collNs.end() ; ++it)
	delete it->second;
    m_collNs.clear();
    m_objects->clearObjects();
}

void collision_space::EnvironmentModelODE::clearObjects(const std::string &ns)
{
    std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.find(ns);
    if (it != m_collNs.end())
	it->second->clear();
    m_objects->clearObjects(ns);
}

void collision_space::EnvironmentModelODE::removeCollidingObjects(const shapes::StaticShape *shape)
{
    checkThreadInit();
    dSpaceID space = dSimpleSpaceCreate(0);
    ODEStorage storage;
    dGeomID g = createODEGeom(space, storage, shape);
    removeCollidingObjects(g);
    dSpaceDestroy(space);
}

void collision_space::EnvironmentModelODE::removeCollidingObjects(const shapes::Shape *shape, const btTransform &pose)
{   
    checkThreadInit();
    dSpaceID space = dSimpleSpaceCreate(0);
    ODEStorage storage;
    dGeomID g = createODEGeom(space, storage, shape, 1.0, 0.0);
    updateGeom(g, pose);
    removeCollidingObjects(g);
    dSpaceDestroy(space);
}

void collision_space::EnvironmentModelODE::removeCollidingObjects(dGeomID geom)
{
    CollisionData cdata;
    for (std::map<std::string, CollisionNamespace*>::iterator it = m_collNs.begin() ; it != m_collNs.end() ; ++it)
    {
	std::map<void*, bool> remove;
	
	// update the set of geoms
	unsigned int n = it->second->geoms.size();
	std::vector<dGeomID> replaceGeoms;
	replaceGeoms.reserve(n);
	for (unsigned int j = 0 ; j < n ; ++j)
	{
	    cdata.done = cdata.collides = false;
	    dSpaceCollide2(geom, it->second->geoms[j], &cdata, nearCallbackFn);
	    if (cdata.collides)
	    {
		remove[dGeomGetData(it->second->geoms[j])] = true;
		dGeomDestroy(it->second->geoms[j]);
	    }
	    else
	    {
		replaceGeoms.push_back(it->second->geoms[j]);
		remove[dGeomGetData(it->second->geoms[j])] = false;
	    }
	}
	it->second->geoms = replaceGeoms;
	
	// update the collide2 structure
	std::vector<dGeomID> geoms;
	it->second->collide2.getGeoms(geoms);
	n = geoms.size();
	for (unsigned int j = 0 ; j < n ; ++j)
	{
	    cdata.done = cdata.collides = false;
	    dSpaceCollide2(geom, geoms[j], &cdata, nearCallbackFn);
	    if (cdata.collides)
	    {
		remove[dGeomGetData(geoms[j])] = true;
		it->second->collide2.unregisterGeom(geoms[j]);
		dGeomDestroy(geoms[j]);
	    }
	    else
		remove[dGeomGetData(geoms[j])] = false;
	}
	
	EnvironmentObjects::NamespaceObjects &no = m_objects->getObjects(it->first);
	
	std::vector<shapes::Shape*> replaceShapes;
	std::vector<btTransform>    replaceShapePoses;
	n = no.shape.size();
	replaceShapes.reserve(n);
	replaceShapePoses.reserve(n);
	for (unsigned int i = 0 ; i < n ; ++i)
	    if (remove[reinterpret_cast<void*>(no.shape[i])])
		delete no.shape[i];
	    else
	    {
		replaceShapes.push_back(no.shape[i]);
		replaceShapePoses.push_back(no.shapePose[i]);
	    }
	no.shape = replaceShapes;
	no.shapePose = replaceShapePoses;
	
	std::vector<shapes::StaticShape*> replaceStaticShapes;
	n = no.staticShape.size();
	replaceStaticShapes.resize(n);
	for (unsigned int i = 0 ; i < n ; ++i)
	    if (remove[reinterpret_cast<void*>(no.staticShape[i])])
		delete no.staticShape[i];
	    else
		replaceStaticShapes.push_back(no.staticShape[i]);
	no.staticShape = replaceStaticShapes;
    }
}

int collision_space::EnvironmentModelODE::setCollisionCheck(const std::string &link, bool state)
{ 
    int result = -1;
    for (unsigned int j = 0 ; j < m_modelGeom.linkGeom.size() ; ++j)
    {
	if (m_modelGeom.linkGeom[j]->link->name == link)
	{
	    result = m_modelGeom.linkGeom[j]->enabled ? 1 : 0;
	    m_modelGeom.linkGeom[j]->enabled = state;
	    break;
	}
    }

    return result;    
}

dGeomID collision_space::EnvironmentModelODE::copyGeom(dSpaceID space, ODEStorage &storage, dGeomID geom, ODEStorage &sourceStorage) const
{
    int c = dGeomGetClass(geom);
    dGeomID ng = NULL;
    bool location = true;
    switch (c)
    {
    case dSphereClass:
	ng = dCreateSphere(space, dGeomSphereGetRadius(geom));
	break;
    case dBoxClass:
	{
	    dVector3 r;
	    dGeomBoxGetLengths(geom, r);
	    ng = dCreateBox(space, r[0], r[1], r[2]);
	}
	break;
    case dCylinderClass:
	{
	    dReal r, l;
	    dGeomCylinderGetParams(geom, &r, &l);
	    ng = dCreateCylinder(space, r, l);
	}
	break;
    case dPlaneClass:
	{
	    dVector4 p;
	    dGeomPlaneGetParams(geom, p);
	    ng = dCreatePlane(space, p[0], p[1], p[2], p[3]);
	    location = false;
	}
	break;
    case dTriMeshClass:
	{
	    dTriMeshDataID tdata = dGeomTriMeshGetData(geom);
	    dTriMeshDataID cdata = dGeomTriMeshDataCreate();
	    for (unsigned int i = 0 ; i < sourceStorage.mesh.size() ; ++i)
		if (sourceStorage.mesh[i].data == tdata)
		{
		    unsigned int p = storage.mesh.size();
		    storage.mesh.resize(p + 1);
		    storage.mesh[p].nVertices = sourceStorage.mesh[i].nVertices;
		    storage.mesh[p].nIndices = sourceStorage.mesh[i].nIndices;
		    storage.mesh[p].indices = new dTriIndex[storage.mesh[p].nIndices];
		    for (int j = 0 ; j < storage.mesh[p].nIndices ; ++j)
			storage.mesh[p].indices[j] = sourceStorage.mesh[i].indices[j];
		    storage.mesh[p].vertices = new double[storage.mesh[p].nVertices];
		    for (int j = 0 ; j < storage.mesh[p].nVertices ; ++j)
			storage.mesh[p].vertices[j] = sourceStorage.mesh[i].vertices[j];
		    dGeomTriMeshDataBuildDouble(cdata, storage.mesh[p].vertices, sizeof(double) * 3, storage.mesh[p].nVertices, storage.mesh[p].indices, storage.mesh[p].nIndices, sizeof(dTriIndex) * 3);
		    storage.mesh[p].data = cdata;
		    break;
		}
	    ng = dCreateTriMesh(space, cdata, NULL, NULL, NULL);
	}
	break;
    default:
	assert(0); // this should never happen
	break;
    }
    
    if (ng && location)
    {
	const dReal *pos = dGeomGetPosition(geom);
	dGeomSetPosition(ng, pos[0], pos[1], pos[2]);
	dQuaternion q;
	dGeomGetQuaternion(geom, q);
	dGeomSetQuaternion(ng, q);
    }
    
    return ng;
}

collision_space::EnvironmentModel* collision_space::EnvironmentModelODE::clone(void) const
{
    EnvironmentModelODE *env = new EnvironmentModelODE();
    env->m_collisionLinks = m_collisionLinks;
    env->m_collisionLinkIndex = m_collisionLinkIndex;
    env->m_selfCollisionTest = m_selfCollisionTest;	
    env->m_selfCollision = m_selfCollision;
    env->m_verbose = m_verbose;
    env->m_robotScale = m_robotScale;
    env->m_robotPadd = m_robotPadd;
    env->m_robotModel = boost::shared_ptr<planning_models::KinematicModel>(m_robotModel->clone());
    env->createODERobotModel();
    for (unsigned int j = 0 ; j < m_modelGeom.linkGeom.size() ; ++j)
	env->m_modelGeom.linkGeom[j]->enabled = m_modelGeom.linkGeom[j]->enabled;

    for (std::map<std::string, CollisionNamespace*>::const_iterator it = m_collNs.begin() ; it != m_collNs.end() ; ++it)
    {
	// construct a map of the shape pointers we have; this points to the index positions where they are stored;
	std::map<void*, int> shapePtrs;
	const EnvironmentObjects::NamespaceObjects &ns = m_objects->getObjects(it->first);
	unsigned int n = ns.staticShape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    shapePtrs[ns.staticShape[i]] = -1 - i;
	n = ns.shape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    shapePtrs[ns.shape[i]] = i;
    
	// copy the collision namespace structure, geom by geom
	CollisionNamespace *cn = new CollisionNamespace(it->first);
	env->m_collNs[it->first] = cn;
	n = it->second->geoms.size();
	cn->geoms.reserve(n);
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    dGeomID newGeom = copyGeom(cn->space, cn->storage, it->second->geoms[i], it->second->storage);
	    int idx = shapePtrs[dGeomGetData(it->second->geoms[i])];
	    if (idx < 0) // static geom
	    {
		shapes::StaticShape *newShape = shapes::cloneShape(ns.staticShape[-idx - 1]);
		dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
		env->m_objects->addObject(it->first, newShape);
	    }
	    else // movable geom
	    {
		shapes::Shape *newShape = shapes::cloneShape(ns.shape[idx]);
		dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
		env->m_objects->addObject(it->first, newShape, ns.shapePose[idx]);
	    }
	    cn->geoms.push_back(newGeom);
	}
	std::vector<dGeomID> geoms;
	it->second->collide2.getGeoms(geoms);
	n = geoms.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    dGeomID newGeom = copyGeom(cn->space, cn->storage, geoms[i], it->second->storage);
	    int idx = shapePtrs[dGeomGetData(geoms[i])];
	    if (idx < 0) // static geom
	    {
		shapes::StaticShape *newShape = shapes::cloneShape(ns.staticShape[-idx - 1]);
		dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
		env->m_objects->addObject(it->first, newShape);
	    }
	    else // movable geom
	    {
		shapes::Shape *newShape = shapes::cloneShape(ns.shape[idx]);
		dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
		env->m_objects->addObject(it->first, newShape, ns.shapePose[idx]);
	    }
	    cn->collide2.registerGeom(newGeom);
	}
    }
    
    return env;    
}
