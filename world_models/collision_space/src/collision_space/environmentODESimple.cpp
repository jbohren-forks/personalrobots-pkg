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

#include "collision_space/environmentODESimple.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <map>

void collision_space::EnvironmentModelODESimple::freeMemory(void)
{ 
    for (unsigned int j = 0 ; j < m_modelGeom.linkGeom.size() ; ++j)
	delete m_modelGeom.linkGeom[j];
    if (m_modelGeom.space)
	dSpaceDestroy(m_modelGeom.space);
    if (m_space)
	dSpaceDestroy(m_space);
    if (m_spaceBasicGeoms)
	dSpaceDestroy(m_spaceBasicGeoms);
}

void collision_space::EnvironmentModelODESimple::addRobotModel(const boost::shared_ptr<planning_models::KinematicModel> &model, const std::vector<std::string> &links, double scale, double padding)
{
    collision_space::EnvironmentModel::addRobotModel(model, links, scale, padding);
    
    m_modelGeom.space = dHashSpaceCreate(0);	
    m_modelGeom.scale = scale;
    m_modelGeom.padding = padding;

    std::map<std::string, bool> exists;
    for (unsigned int i = 0 ; i < links.size() ; ++i)
	exists[links[i]] = true;
    
    for (unsigned int j = 0 ; j < m_robotModel->getRobotCount() ; ++j)
    {
	planning_models::KinematicModel::Robot *robot = m_robotModel->getRobot(j);
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
	    dGeomID g = createODEGeom(m_modelGeom.space, robot->links[i]->shape, scale, padding);
	    assert(g);
	    kg->geom.push_back(g);
	    for (unsigned int k = 0 ; k < kg->link->attachedBodies.size() ; ++k)
	    {
		dGeomID ga = createODEGeom(m_modelGeom.space, kg->link->attachedBodies[k]->shape, scale, padding);
		assert(ga);
		kg->geom.push_back(ga);
	    }
	    m_modelGeom.linkGeom.push_back(kg);
	}
    }
}

dGeomID collision_space::EnvironmentModelODESimple::createODEGeom(dSpaceID space, planning_models::shapes::Shape *shape, double scale, double padding) const
{
    dGeomID g = NULL;
    switch (shape->type)
    {
    case planning_models::shapes::Shape::SPHERE:
	{
	    g = dCreateSphere(space, static_cast<planning_models::shapes::Sphere*>(shape)->radius * scale + padding);
	}
	break;
    case planning_models::shapes::Shape::BOX:
	{
	    const double *size = static_cast<planning_models::shapes::Box*>(shape)->size;
	    g = dCreateBox(space, size[0] * scale + padding, size[1] * scale + padding, size[2] * scale + padding);
	}	
	break;
    case planning_models::shapes::Shape::CYLINDER:
	{
	    g = dCreateCylinder(space, static_cast<planning_models::shapes::Cylinder*>(shape)->radius * scale + padding,
				static_cast<planning_models::shapes::Cylinder*>(shape)->length * scale + padding);
	}
	break;
    default:
	break;
    }
    return g;
}

void collision_space::EnvironmentModelODESimple::updateGeom(dGeomID geom, btTransform &pose) const
{
    btVector3 pos = pose.getOrigin();
    dGeomSetPosition(geom, pos.getX(), pos.getY(), pos.getZ());
    btQuaternion quat = pose.getRotation();
    dQuaternion q; 
    q[0] = quat.getW(); q[1] = quat.getX(); q[2] = quat.getY(); q[3] = quat.getZ();
    dGeomSetQuaternion(geom, q);
}

void collision_space::EnvironmentModelODESimple::updateAttachedBodies(void)
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
	    dGeomID ga = createODEGeom(m_modelGeom.space, kg->link->attachedBodies[k]->shape, m_modelGeom.scale, m_modelGeom.padding);
	    assert(ga);
	    kg->geom.push_back(ga);
	}
    }
}

void collision_space::EnvironmentModelODESimple::updateRobotModel(void)
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

dSpaceID collision_space::EnvironmentModelODESimple::getODESpace(void) const
{
    return m_space;
}

dSpaceID collision_space::EnvironmentModelODESimple::getODEBasicGeomSpace(void) const
{
    return m_spaceBasicGeoms;
}

dSpaceID collision_space::EnvironmentModelODESimple::getModelODESpace(void) const
{
    return m_modelGeom.space;    
}

struct CollisionData
{
    CollisionData(void)
    {
	done = false;
	collides = false;
	max_contacts = 0;
	contacts = NULL;
	link1 = link2 = NULL;
    }
    
    bool                                                        done;
    
    bool                                                        collides;
    unsigned int                                                max_contacts;
    std::vector<collision_space::EnvironmentModelODESimple::Contact> *contacts;
    
    planning_models::KinematicModel::Link                      *link1;
    planning_models::KinematicModel::Link                      *link2;
};

static void nearCallbackFn(void *data, dGeomID o1, dGeomID o2)
{
    CollisionData *cdata = reinterpret_cast<CollisionData*>(data);

    if (cdata->done)
	return;
    
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
		    collision_space::EnvironmentModelODESimple::Contact add;
		    
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

bool collision_space::EnvironmentModelODESimple::getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count)
{
    CollisionData cdata;
    cdata.contacts = &contacts;
    cdata.max_contacts = max_count;
    contacts.clear();
    testCollision(reinterpret_cast<void*>(&cdata));
    return cdata.collides;
}

bool collision_space::EnvironmentModelODESimple::isCollision(void)
{
    CollisionData cdata;
    testCollision(reinterpret_cast<void*>(&cdata));
    return cdata.collides;
}

void collision_space::EnvironmentModelODESimple::testSelfCollision(void *data)
{ 
    CollisionData *cdata = reinterpret_cast<CollisionData*>(data);
    for (int i = m_modelGeom.selfCollision.size() - 1 ; i >= 0 && !cdata->done ; --i)
    {
	const std::vector<unsigned int> &vec = m_modelGeom.selfCollision[i];
	unsigned int n = vec.size();
	
	for (unsigned int j = 0 ; j < n && !cdata->done ; ++j)
	{
	    cdata->link1 = m_modelGeom.linkGeom[vec[j]]->link;
	    const unsigned int njg = m_modelGeom.linkGeom[vec[j]]->geom.size();
	    
	    for (unsigned int k = j + 1 ; k < n && !cdata->done; ++k)
	    {
		// dSpaceCollide2 expects AABBs to be computed, so
		// we force that by calling dGeomGetAABB. Since we
		// get the data anyway, we attempt to speed things
		// up using it.
		
		const unsigned int nkg = m_modelGeom.linkGeom[vec[k]]->geom.size();
		cdata->link2 = m_modelGeom.linkGeom[vec[k]]->link;
		
		/* this will account for attached bodies as well */
		for (unsigned int jg = 0 ; jg < njg && !cdata->done; ++jg)
		    for (unsigned int kg = 0 ; kg < nkg && !cdata->done; ++kg)
		    {
			dGeomID g1 = m_modelGeom.linkGeom[vec[j]]->geom[jg];
			dGeomID g2 = m_modelGeom.linkGeom[vec[k]]->geom[kg];
			
			dReal aabb1[6], aabb2[6];		    
			dGeomGetAABB(g1, aabb1);
			dGeomGetAABB(g2, aabb2);
			
			if (!(aabb1[2] > aabb2[3] ||
			      aabb1[3] < aabb2[2] ||
			      aabb1[4] > aabb2[5] ||
			      aabb1[5] < aabb2[4])) 
			    dSpaceCollide2(g1, g2, data, nearCallbackFn);
			
			if (cdata->collides && m_verbose)
			    m_msg.inform("Self-collision between '%s' and '%s'\n",
					 m_modelGeom.linkGeom[vec[j]]->link->name.c_str(), m_modelGeom.linkGeom[vec[k]]->link->name.c_str());
		    }
	    }
	}
    }  
}
void collision_space::EnvironmentModelODESimple::testStaticBodyCollision(void *data)
{  
    CollisionData *cdata = reinterpret_cast<CollisionData*>(data);
    cdata->link2 = NULL;
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
	    for (int j = m_basicGeoms.size() - 1 ; j >= 0 ; --j)
	    {
		dGeomID g2 = m_basicGeoms[j];
		dReal aabb2[6];
		dGeomGetAABB(g2, aabb2);
		
		if (!(aabb1[2] > aabb2[3] ||
		      aabb1[3] < aabb2[2] ||
		      aabb1[4] > aabb2[5] ||
		      aabb1[5] < aabb2[4]))
		    dSpaceCollide2(g1, g2, data, nearCallbackFn);
		
		if (cdata->collides && m_verbose)
		    m_msg.inform("Collision between static body and link '%s'\n",
				 m_modelGeom.linkGeom[i]->link->name.c_str());
	    }
	}
    }
}

void collision_space::EnvironmentModelODESimple::testDynamicBodyCollision(void *data)
{ 
    CollisionData *cdata = reinterpret_cast<CollisionData*>(data);
    cdata->link2 = NULL;
    for (int i = m_modelGeom.linkGeom.size() - 1 ; i >= 0 && !cdata->done ; --i)
	if (m_modelGeom.linkGeom[i]->enabled)
	{
	    const unsigned int ng = m_modelGeom.linkGeom[i]->geom.size();
	    cdata->link1 = m_modelGeom.linkGeom[i]->link;	    
	    for (unsigned int ig = 0 ; ig < ng && !cdata->done ; ++ig)
	    {
		dSpaceCollide2((dGeomID)m_space, m_modelGeom.linkGeom[i]->geom[ig], data, nearCallbackFn);
		if (cdata->collides && m_verbose)
		    m_msg.inform("Collision between dynamic body and link '%s'\n",
				 m_modelGeom.linkGeom[i]->link->name.c_str());
	    }
	}
}

void collision_space::EnvironmentModelODESimple::testCollision(void *data)
{
    CollisionData *cdata = reinterpret_cast<CollisionData*>(data);
    
    /* check self collision */
    if (m_selfCollision)
	testSelfCollision(data);
    
    /* check collision with standalone ode bodies */
    if (!cdata->done)
	testStaticBodyCollision(data);
    
    /* check collision with pointclouds */
    if (!cdata->done)
	testDynamicBodyCollision(data);
    
    cdata->done = true;
}

void collision_space::EnvironmentModelODESimple::addPointCloud(unsigned int n, const double *points)
{
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	unsigned int i4 = i * 4;
	dGeomID g = dCreateSphere(m_space, points[i4 + 3]);
	dGeomSetPosition(g, points[i4], points[i4 + 1], points[i4 + 2]);
    }
}

void collision_space::EnvironmentModelODESimple::addStaticPlane(double a, double b, double c, double d)
{
    dGeomID g = dCreatePlane(m_spaceBasicGeoms, a, b, c, d);
    m_basicGeoms.push_back(g);
}

void collision_space::EnvironmentModelODESimple::clearObstacles(void)
{
    if (m_space)
	dSpaceDestroy(m_space);
    m_space = dHashSpaceCreate(0);
}

void collision_space::EnvironmentModelODESimple::addSelfCollisionGroup(std::vector<std::string> &links)
{
    unsigned int pos = m_modelGeom.selfCollision.size();
    m_modelGeom.selfCollision.resize(pos + 1);
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	for (unsigned int j = 0 ; j < m_modelGeom.linkGeom.size() ; ++j)
	    if (m_modelGeom.linkGeom[j]->link->name == links[i])
		m_modelGeom.selfCollision[pos].push_back(j);
    }

}

int collision_space::EnvironmentModelODESimple::setCollisionCheck(const std::string &link, bool state)
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
