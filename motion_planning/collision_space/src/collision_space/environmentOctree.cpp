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

/** \Author Ioan Sucan, Matei Ciocarlie */

#include <collision_space/environmentOctree.h>

unsigned int collision_space::EnvironmentModelOctree::addRobotModel(planning_models::KinematicModel *model, const std::vector<std::string> &links)
{
    unsigned int id = collision_space::EnvironmentModel::addRobotModel(model, links);
    if (id <= m_modelLinks.size())
	m_modelLinks.resize(id + 1);
    
    std::vector< planning_models::KinematicModel::Link*> allLinks;
    
    std::map<std::string, bool> exists;
    for (unsigned int i = 0 ; i < links.size() ; ++i)
	exists[links[i]] = true;
    
    for (unsigned int i = 0 ; i < allLinks.size() ; ++i)
	if (exists.find(allLinks[i]->name) != exists.end())
	    m_modelLinks[id].push_back(allLinks[i]);
    
    return id;
}

void collision_space::EnvironmentModelOctree::updateRobotModel(unsigned int model_id)
{ 
}

bool collision_space::EnvironmentModelOctree::isCollision(unsigned int model_id)
{
    bool result = false;
    
    const std::vector<planning_models::KinematicModel::Link*> &links = m_modelLinks[model_id];
    
    for (unsigned int i = 0 ; !result && i < links.size() ; ++i)
    {
	switch (links[i]->shape->type)
	{
	case planning_models::KinematicModel::Shape::BOX:
	    {
		/** \todo math here is a bit clumsy.... should be fixed when bette libTF is available */
		NEWMAT::Matrix mat = links[i]->globalTrans.asMatrix();
		float axes[3][3];
		
		for (unsigned int j=0; j<3; j++) 
		    for (unsigned int k=0; k<3; k++)
			axes[j][k] = mat.element(j,k);
		
		libTF::Position libTFpos;
		links[i]->globalTrans.getPosition(libTFpos);
		float pos[3] = {libTFpos.x, libTFpos.y, libTFpos.z};
		const double *size = static_cast<planning_models::KinematicModel::Box*>(links[i]->shape)->size;
		const float sizef[3] = {size[0], size[1], size[2]};
		
		result = m_octree.intersectsBox(pos, sizef, axes);
	    }
	    
	    break;

	case planning_models::KinematicModel::Shape::CYLINDER:
	    {
		NEWMAT::Matrix mat = links[i]->globalTrans.asMatrix();
		float axes[3][3];
		
		for (unsigned int j=0; j<3; j++) 
		    for (unsigned int k=0; k<3; k++)
			axes[j][k] = mat.element(j,k);
		
		libTF::Position libTFpos;
		links[i]->globalTrans.getPosition(libTFpos);
		float pos[3] = {libTFpos.x, libTFpos.y, libTFpos.z };
		float radius = static_cast<planning_models::KinematicModel::Cylinder*>(links[i]->shape)->radius;
		float length = static_cast<planning_models::KinematicModel::Cylinder*>(links[i]->shape)->length;
		float L = radius * 2.0;
		const float sizef[3] = {length, L, L};
		
		result = m_octree.intersectsBox(pos, sizef, axes);
	    }
	    break;
		
	case planning_models::KinematicModel::Shape::SPHERE:
	    {
		libTF::Position libTFpos;
		links[i]->globalTrans.getPosition(libTFpos);
		float pos[3] = {libTFpos.x, libTFpos.y, libTFpos.z};
		float radius = static_cast<planning_models::KinematicModel::Sphere*>(links[i]->shape)->radius;
		result = m_octree.intersectsSphere(pos, radius);
	    }
	    break;
	default:
	    fprintf(stderr, "Geometry type not implemented: %d\n", links[i]->shape->type);
	    break;	    
	}
    }

    return result;    
}

void collision_space::EnvironmentModelOctree::addPointCloud(unsigned int n, const double *points, double radius)
{
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	unsigned int i3 = i * 3;
	m_octree.insert(points[i3], points[i3 + 1], points[i3 + 2], OCTREE_CELL_OCCUPIED);	
    }
}

void collision_space::EnvironmentModelOctree::clearObstacles(void)
{
    m_octree.clear();
}

const scan_utils::Octree<char>* collision_space::EnvironmentModelOctree::getOctree(void) const
{
    return &m_octree;    
}

void collision_space::EnvironmentModelOctree::addPlane(double a, double b, double c, double d)
{
    fprintf(stderr, "Octree collision checking does not support planes\n");    
}

void collision_space::EnvironmentModelOctree::addSelfCollisionGroup(unsigned int model_id, std::vector<std::string> &links)
{
    fprintf(stderr, "Octree collision checking does not support self collision\n");    
}
