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

#include "collision_space/bodies.h"
#include "collision_space/output.h"
#include <LinearMath/btConvexHull.h>
#include <cmath>


static collision_space::msg::Interface MSG;

collision_space::bodies::Body* collision_space::bodies::createBodyFromShape(const planning_models::shapes::Shape *shape)
{
    Body *body = NULL;
    
    switch (shape->type)
    {
    case planning_models::shapes::Shape::BOX:
	body = new collision_space::bodies::Box(shape);
	break;
    case planning_models::shapes::Shape::SPHERE:
	body = new collision_space::bodies::Sphere(shape);
	break;
    case planning_models::shapes::Shape::CYLINDER:
	body = new collision_space::bodies::Cylinder(shape);
	break;
    case planning_models::shapes::Shape::MESH:
	body = new collision_space::bodies::ConvexMesh(shape);
	break;
    default:
	MSG.error("Creating body from shape: Unknown shape type %d", shape->type);
	break;
    }
    
    return body;
}

void collision_space::bodies::mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere)
{
    if (spheres.empty())
    {
	mergedSphere.center.setValue(btScalar(0), btScalar(0), btScalar(0));
	mergedSphere.radius = 0.0;
    }
    else
    {
	mergedSphere = spheres[0];
	for (unsigned int i = 1 ; i < spheres.size() ; ++i)
	{
	    if (spheres[i].radius <= 0.0)
		continue;
	    double d = spheres[i].center.distance(mergedSphere.center);
	    if (d + mergedSphere.radius <= spheres[i].radius)
	    {
		mergedSphere.center = spheres[i].center;
		mergedSphere.radius = spheres[i].radius;
	    }
	    else
		if (d + spheres[i].radius > mergedSphere.radius)
		{
		    btVector3 delta = mergedSphere.center - spheres[i].center;
		    mergedSphere.radius = (delta.length() + spheres[i].radius + mergedSphere.radius)/2.0;
		    mergedSphere.center = delta.normalized() * (mergedSphere.radius - spheres[i].radius) + spheres[i].center;
		}
	}
    }
}

bool collision_space::bodies::Sphere::containsPoint(const btVector3 &p) const 
{
    return (m_center - p).length2() < m_radius2;
}

void collision_space::bodies::Sphere::useDimensions(const planning_models::shapes::Shape *shape) // radius
{
    m_radius = static_cast<const planning_models::shapes::Sphere*>(shape)->radius;
}

void collision_space::bodies::Sphere::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_center = m_pose.getOrigin();
}

double collision_space::bodies::Sphere::computeVolume(void) const
{
    return 4.0 * M_PI * m_radiusU * m_radiusU * m_radiusU / 3.0;
}

void collision_space::bodies::Sphere::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusU;
}

bool collision_space::bodies::Cylinder::containsPoint(const btVector3 &p) const 
{
    btVector3 v = p - m_center;		
    double pH = v.dot(m_normalH);
    
    if (fabs(pH) > m_length2)
	return false;
    
    double pB1 = v.dot(m_normalB1);
    double remaining = m_radius2 - pB1 * pB1;
    
    if (remaining < 0.0)
	return false;
    else
    {
	double pB2 = v.dot(m_normalB2);
	return pB2 * pB2 < remaining;
    }		
}

void collision_space::bodies::Cylinder::useDimensions(const planning_models::shapes::Shape *shape) // (length, radius)
{
    m_length = static_cast<const planning_models::shapes::Cylinder*>(shape)->length;
    m_radius = static_cast<const planning_models::shapes::Cylinder*>(shape)->radius;
}

void collision_space::bodies::Cylinder::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_length2 = m_scale * m_length / 2.0 + m_padding;
    m_center = m_pose.getOrigin();
    m_radiusB = std::max(m_radiusU, m_length2);
    
    const btMatrix3x3& basis = m_pose.getBasis();
    m_normalB1 = basis.getColumn(0);
    m_normalB2 = basis.getColumn(1);
    m_normalH  = basis.getColumn(2);
}

double collision_space::bodies::Cylinder::computeVolume(void) const
{
    return 2.0 * M_PI * m_radiusU * m_radiusU * m_length2;
}

void collision_space::bodies::Cylinder::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool collision_space::bodies::Box::containsPoint(const btVector3 &p) const 
{
    btVector3 v = p - m_center;
    double pL = v.dot(m_normalL);
    
    if (fabs(pL) > m_length2)
	return false;
    
    double pW = v.dot(m_normalW);
    
    if (fabs(pW) > m_width2)
	return false;
    
    double pH = v.dot(m_normalH);
    
    if (fabs(pH) > m_height2)
	return false;
    
    return true;
}

void collision_space::bodies::Box::useDimensions(const planning_models::shapes::Shape *shape) // (x, y, z) = (length, width, height)
{
    const double *size = static_cast<const planning_models::shapes::Box*>(shape)->size;
    m_length = size[0];
    m_width  = size[1];
    m_height = size[2];
}

void collision_space::bodies::Box::updateInternalData(void) 
{
    double s2 = m_scale / 2.0;
    m_length2 = m_length * s2 + m_padding;
    m_width2  = m_width * s2 + m_padding;
    m_height2 = m_height * s2 + m_padding;
    
    m_center  = m_pose.getOrigin();
    
    m_radiusB = sqrt(m_length2 * m_length2 + m_width2 * m_width2 + m_height2 * m_height2);

    const btMatrix3x3& basis = m_pose.getBasis();
    m_normalL = basis.getColumn(0);
    m_normalW = basis.getColumn(1);
    m_normalH = basis.getColumn(2);
}

double collision_space::bodies::Box::computeVolume(void) const
{
    return 8.0 * m_length2 * m_width2 * m_height2;
}

void collision_space::bodies::Box::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool collision_space::bodies::ConvexMesh::containsPoint(const btVector3 &p) const
{
    btVector3 ip = (m_iPose * p) / m_scale;
    return isPointInsidePlanes(ip);
}

void collision_space::bodies::ConvexMesh::useDimensions(const planning_models::shapes::Shape *shape)
{  
    const planning_models::shapes::Mesh *mesh = static_cast<const planning_models::shapes::Mesh*>(shape);
    m_planes.clear();
    m_triangles.clear();
    m_vertices.clear();
    m_radiusB = 0.0;
    m_meshCenter.setValue(btScalar(0), btScalar(0), btScalar(0));

    btVector3 *vertices = new btVector3[mesh->vertexCount];
    for(unsigned int i = 0; i < mesh->vertexCount ; ++i)
    {
	vertices[i].setX(mesh->vertices[3 * i    ]);
	vertices[i].setY(mesh->vertices[3 * i + 1]);
	vertices[i].setZ(mesh->vertices[3 * i + 2]);
    }
    
    HullDesc hd(QF_TRIANGLES, mesh->vertexCount, vertices);
    HullResult hr;
    HullLibrary hl;
    if (hl.CreateConvexHull(hd, hr) == QE_OK)
    {
	MSG.inform("Convex hull has %d(%d) vertices (down from %d), %d faces", hr.m_OutputVertices.size(), hr.mNumOutputVertices, mesh->vertexCount, hr.mNumFaces);

	m_vertices.reserve(hr.m_OutputVertices.size());
	btVector3 sum(btScalar(0), btScalar(0), btScalar(0));	
	for (int j = 0 ; j < hr.m_OutputVertices.size() ; ++j)
	{
	    m_vertices.push_back(hr.m_OutputVertices[j]);
	    sum = sum + hr.m_OutputVertices[j];
	}
	
	m_meshCenter = sum / (double)(hr.m_OutputVertices.size());
	for (unsigned int j = 0 ; j < m_vertices.size() ; ++j)
	{
	    double dist = m_vertices[j].distance2(m_meshCenter);
	    if (dist > m_radiusB)
		m_radiusB = dist;
	}
	m_radiusB = sqrt(m_radiusB) * m_scale + m_padding;
	
	m_triangles.reserve(hr.m_Indices.size());
	for (int j = 0 ; j < hr.m_Indices.size() ; ++j)
	    m_triangles.push_back(hr.m_Indices[j]);
	
	for (unsigned int j = 0 ; j < hr.mNumFaces ; ++j)
	{
	    const btVector3 &p1 = hr.m_OutputVertices[hr.m_Indices[j * 3    ]];
	    const btVector3 &p2 = hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]];
	    const btVector3 &p3 = hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]];
	    
	    btVector3 edge1 = (p2 - p1);
	    btVector3 edge2 = (p3 - p1);
	    
	    edge1.normalize();
	    edge2.normalize();

	    btVector3 planeNormal = edge1.cross(edge2);
	    
	    if (planeNormal.length2() > btScalar(1e-6))
	    {
		planeNormal.normalize();
		btVector4 planeEquation(planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), -planeNormal.dot(p1));

		unsigned int behindPlane = countVerticesBehindPlane(planeEquation);
		if (behindPlane > 0)
		{
		    btVector4 planeEquation2(-planeEquation.getX(), -planeEquation.getY(), -planeEquation.getZ(), -planeEquation.getW());
		    unsigned int behindPlane2 = countVerticesBehindPlane(planeEquation2);
		    if (behindPlane2 < behindPlane)
		    {
			planeEquation.setValue(planeEquation2.getX(), planeEquation2.getY(), planeEquation2.getZ(), planeEquation2.getW());
			behindPlane = behindPlane2;
		    }
		}
		
		if (behindPlane > 0)
		    MSG.warn("Approximate plane: %u of %u points are behind the plane", behindPlane, (unsigned int)m_vertices.size());
		
		m_planes.push_back(planeEquation);
	    }
	}
    }
    else
	MSG.error("Unable to compute convex hull.");
    
    hl.ReleaseResult(hr);    
    delete[] vertices;
}

void collision_space::bodies::ConvexMesh::updateInternalData(void) 
{
    m_iPose = m_pose.inverse();
    m_center = m_pose.getOrigin() + m_meshCenter;
}

void collision_space::bodies::ConvexMesh::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool collision_space::bodies::ConvexMesh::isPointInsidePlanes(const btVector3& point) const
{
    unsigned int numplanes = m_planes.size();
    for (unsigned int i = 0 ; i < numplanes ; ++i)
    {
	const btVector4& plane = m_planes[i];
	btScalar dist = btScalar(plane.dot(point)) + plane.getW() - btScalar(m_padding) - btScalar(1e-6);
	if (dist > btScalar(0))
	    return false;
    }
    return true;
}

unsigned int collision_space::bodies::ConvexMesh::countVerticesBehindPlane(const btVector4& planeNormal) const
{
    unsigned int numvertices = m_vertices.size();
    unsigned int result = 0;
    for (unsigned int i = 0 ; i < numvertices ; ++i)
    {
	btScalar dist = btScalar(planeNormal.dot(m_vertices[i])) + btScalar(planeNormal.getW()) - btScalar(1e-6);
	if (dist > btScalar(0))
	    result++;
    }
    return result;
}

double collision_space::bodies::ConvexMesh::computeVolume(void) const
{
    double volume = 0.0;
    for (unsigned int i = 0 ; i < m_triangles.size() / 3 ; ++i)
    {
	const btVector3 &v1 = m_vertices[m_triangles[3*i + 0]];
        const btVector3 &v2 = m_vertices[m_triangles[3*i + 1]];
	const btVector3 &v3 = m_vertices[m_triangles[3*i + 2]];
	volume += v1.x()*v2.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() -v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() - v3.x()*v2.y()*v1.z();
    }
    return fabs(volume)/6.0;
}
