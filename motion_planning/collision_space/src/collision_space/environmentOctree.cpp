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

#include <collision_space/environmentOctree.h>

unsigned int collision_space::EnvironmentModelOctree::addRobotModel(planning_models::KinematicModel *model)
{
    unsigned int id = collision_space::EnvironmentModel::addRobotModel(model);
    if (id <= m_modelLinks.size())
	m_modelLinks.resize(id + 1);
    m_models[id]->getLinks(m_modelLinks[id]);
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
	switch (links[i]->geom->type)
	{
	case planning_models::KinematicModel::Geometry::BOX:
	    {
		NEWMAT::Matrix mat = links[i]->globalTrans.asMatrix();
		float axes[3][3];
		
		for (unsigned int j=0; j<3; j++) 
		    for (unsigned int k=0; k<3; k++)
			axes[j][k] = mat.element(j,k);
		
		libTF::Pose3D::Position libTFpos;
		links[i]->globalTrans.getPosition(libTFpos);
		float pos[3] = {libTFpos.x, libTFpos.y, libTFpos.z };
		const double *size = links[i]->geom->size;
		const float sizef[3] = {size[0], size[1], size[2]};
		
		result = m_octree.intersectsBox(pos, sizef, axes);
	    }
	    
	    break;

	case planning_models::KinematicModel::Geometry::CYLINDER:
	    {
		NEWMAT::Matrix mat = links[i]->globalTrans.asMatrix();
		float axes[3][3];
		
		for (unsigned int j=0; j<3; j++) 
		    for (unsigned int k=0; k<3; k++)
			axes[j][k] = mat.element(j,k);
		
		libTF::Pose3D::Position libTFpos;
		links[i]->globalTrans.getPosition(libTFpos);
		float pos[3] = {libTFpos.x, libTFpos.y, libTFpos.z };
		const double *size = links[i]->geom->size;
		const float L = size[1]*sqrt(2);
		const float sizef[3] = {size[0], L, L};
		
		result = m_octree.intersectsBox(pos, sizef, axes);
	    }
	    break;
		
	case planning_models::KinematicModel::Geometry::SPHERE:
	    {
		libTF::Pose3D::Position libTFpos;
		links[i]->globalTrans.getPosition(libTFpos);
		float pos[3] = {libTFpos.x, libTFpos.y, libTFpos.z };
		result = m_octree.intersectsSphere(pos, links[i]->geom->size[0]);
	    }
	    break;
	default:
	    fprintf(stderr, "Geometry type not implemented: %d\n", links[i]->geom->type);
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
