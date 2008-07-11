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

#include <collisionspace/environmentODE.h>

class ODECollide2
{
public:
    
    ODECollide2(dSpaceID space = NULL)
    {	
	m_setup = false;
	if (space)
	    registerSpace(space);
    }
    
    ~ODECollide2(void)
    {
    }
    
    void registerSpace(dSpaceID space)
    {
	int n = dSpaceGetNumGeoms(space);
	for (int i = 0 ; i < n ; ++i)
	    registerGeom(dSpaceGetGeom(space, i));
    }
    
    void registerGeom(dGeomID geom)
    {
	Geom g;
	g.id = geom;
	dGeomGetAABB(geom, g.aabb);
	m_geoms.push_back(g);
	m_setup = false;
    }
    
    void clear(void)
    {
	m_geoms.clear();
	m_setup = false;
    }
    
    void setup(void)
    {
	sort(m_geoms.begin(), m_geoms.end(), SortByXYZLow());
	m_setup = true;
    }
        
    void collide(dGeomID geom, void *data, dNearCallback *nearCallback)
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
    
    
private:

    struct Geom
    {
	dGeomID id;
	dReal   aabb[6];
    };
    
    struct SortByXYZLow
    {
	bool operator()(const Geom &a, const Geom &b) 
	{
	    if (a.aabb[0] < b.aabb[0])
		return true;
	    if (a.aabb[0] > b.aabb[0])
		return false;
	    
	    if (a.aabb[2] < b.aabb[2])
		return true;
	    if (a.aabb[2] > b.aabb[2])
		return false;
	    
	    if (a.aabb[4] < b.aabb[4])
		return true;
	    return false;	
	}
    };
    
    struct SortByX
    {
	bool operator()(const Geom &a, const Geom &b)
	{
	    if (a.aabb[1] < b.aabb[0])
		return true;
	    return false;
	}
    };
    
    bool              m_setup;
    std::vector<Geom> m_geoms;
    
};

bool EnvironmentModelODE::isCollision(void)
{
}
    
void EnvironmentModelODE::addPointCloud(void)
{
}
