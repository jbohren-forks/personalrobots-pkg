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

#include "geometric_shapes/bodies.h"
#include <LinearMath/btConvexHull.h>
#include <algorithm>
#include <iostream>
#include <cmath>

bodies::Body* bodies::createBodyFromShape(const shapes::Shape *shape)
{
    Body *body = NULL;
    
    if (shape)
	switch (shape->type)
	{
	case shapes::BOX:
	    body = new bodies::Box(shape);
	    break;
	case shapes::SPHERE:
	    body = new bodies::Sphere(shape);
	    break;
	case shapes::CYLINDER:
	    body = new bodies::Cylinder(shape);
	    break;
	case shapes::MESH:
	    body = new bodies::ConvexMesh(shape);
	    break;
	default:
	    std::cerr << "Creating body from shape: Unknown shape type" << shape->type << std::endl;
	    break;
	}
    
    return body;
}

void bodies::mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere)
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

namespace bodies
{
    static const double ZERO = 1e-9;
    
    /* Compute the square of the distance between a ray and a point */
    /* Note: this requires 'dir' to be normalized */
    static inline double distanceSQR(const btVector3& p, const btVector3& origin, const btVector3& dir)
    {
	btVector3 a = p - origin;
	double d = dir.dot(a);
	return a.length2() - d * d;
    }
}

bool bodies::Sphere::containsPoint(const btVector3 &p) const 
{
    return (m_center - p).length2() < m_radius2;
}

void bodies::Sphere::useDimensions(const shapes::Shape *shape) // radius
{
    m_radius = static_cast<const shapes::Sphere*>(shape)->radius;
}

void bodies::Sphere::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_center = m_pose.getOrigin();
}

double bodies::Sphere::computeVolume(void) const
{
    return 4.0 * M_PI * m_radiusU * m_radiusU * m_radiusU / 3.0;
}

void bodies::Sphere::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusU;
}

bool bodies::Sphere::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections)
{
    if (distanceSQR(m_center, origin, dir) > m_radius2) return false;
    bool result = false;
    
    btVector3 cp = origin - m_center;
    double dpcpv = cp.dot(dir);
    
    btVector3 w = cp - dpcpv * dir;
    btVector3 Q = m_center + w;
    double x = m_radius2 - w.length2();
    
    if (fabs(x) < ZERO)
    { 
	w = Q - origin;
	double dpQv = w.dot(dir);
	if (dpQv > ZERO)
	{
	    if (intersections)
		intersections->push_back(Q);
	    result = true;
	}
    } else 
	if (x > 0.0)
	{    
	    x = sqrt(x);
	    w = dir * x;
	    btVector3 A = Q - w;
	    btVector3 B = Q + w;
	    w = A - origin;
	    double dpAv = w.dot(dir);
	    w = B - origin;
	    double dpBv = w.dot(dir);
	    
	    if (dpAv > ZERO)
	    {	
		if (intersections)
		    intersections->push_back(A);
		result = true;
	    }
	    
	    if (dpBv > ZERO)
	    {
		if (intersections)
		    intersections->push_back(B);
		result = true;
	    }
	}
    return result;
}

bool bodies::Cylinder::containsPoint(const btVector3 &p) const 
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

void bodies::Cylinder::useDimensions(const shapes::Shape *shape) // (length, radius)
{
    m_length = static_cast<const shapes::Cylinder*>(shape)->length;
    m_radius = static_cast<const shapes::Cylinder*>(shape)->radius;
}

void bodies::Cylinder::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_length2 = m_scale * m_length / 2.0 + m_padding;
    m_center = m_pose.getOrigin();
    m_radiusB = sqrt(m_length2 * m_length2 + m_radius2);
    
    const btMatrix3x3& basis = m_pose.getBasis();
    m_normalB1 = basis.getColumn(0);
    m_normalB2 = basis.getColumn(1);
    m_normalH  = basis.getColumn(2);
    
    btVector3 tmp = (m_normalB1 + m_normalB2) * m_radiusU;    
    m_corner1 = m_center - tmp - m_normalH * m_length2;
    m_corner2 = m_center + tmp + m_normalH * m_length2;
}

double bodies::Cylinder::computeVolume(void) const
{
    return 2.0 * M_PI * m_radiusU * m_radiusU * m_length2;
}

void bodies::Cylinder::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool bodies::Cylinder::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections)
{
    if (distanceSQR(m_center, origin, dir) > m_radiusB) return false;

    double t_near = -INFINITY;
    double t_far  = INFINITY;
    
    for (int i = 0; i < 3; i++)
    {
	btVector3 &vN = i == 0 ? m_normalB1 : (i == 1 ? m_normalB2 : m_normalH);
	double dp = vN.dot(dir);
	
	if (fabs(dp) > ZERO)
	{
	    double t1 = vN.dot(m_corner1 - origin) / dp;
	    double t2 = vN.dot(m_corner2 - origin) / dp;
	    
	    if (t1 > t2)
		std::swap(t1, t2);
	    
	    if (t1 > t_near)
		t_near = t1;
	    
	    if (t2 < t_far)
		t_far = t2;
	    
	    if (t_near > t_far)
		return false;
	    
	    if (t_far < 0.0)
		return false;
	}
	else
	{
	    if (i == 0)
	    {
		if ((std::min(m_corner1.y(), m_corner2.y()) > origin.y() ||
		     std::max(m_corner1.y(), m_corner2.y()) < origin.y()) && 
		    (std::min(m_corner1.z(), m_corner2.z()) > origin.z() ||
		     std::max(m_corner1.z(), m_corner2.z()) < origin.z()))
		    return false;
	    }
	    else
	    {
		if (i == 1)
		{
		    if ((std::min(m_corner1.x(), m_corner2.x()) > origin.x() ||
			 std::max(m_corner1.x(), m_corner2.x()) < origin.x()) && 
			(std::min(m_corner1.z(), m_corner2.z()) > origin.z() ||
			 std::max(m_corner1.z(), m_corner2.z()) < origin.z()))
			return false;
		}
		else
		    if ((std::min(m_corner1.x(), m_corner2.x()) > origin.x() ||
			 std::max(m_corner1.x(), m_corner2.x()) < origin.x()) && 
			(std::min(m_corner1.y(), m_corner2.y()) > origin.y() ||
			 std::max(m_corner1.y(), m_corner2.y()) < origin.y()))
			return false;
	    }
	}
    }
    
    // at this point we know there is intersection with the box around the cylinder
    // \todo we need to figure out if this intersection is on the cylinder as well
    
    if (intersections)
    {
	if (t_far - t_near > ZERO)
	{
	    intersections->push_back(t_near * dir + origin);
	    intersections->push_back(t_far  * dir + origin);
	}
	else
	    intersections->push_back(t_far * dir + origin);
    }
    
    return true;
}

bool bodies::Box::containsPoint(const btVector3 &p) const 
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

void bodies::Box::useDimensions(const shapes::Shape *shape) // (x, y, z) = (length, width, height)
{
    const double *size = static_cast<const shapes::Box*>(shape)->size;
    m_length = size[0];
    m_width  = size[1];
    m_height = size[2];
}

void bodies::Box::updateInternalData(void) 
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

    m_corner1 = m_center - m_normalL * m_length2 - m_normalW * m_width2 - m_normalH * m_height2;
    m_corner2 = m_center + m_normalL * m_length2 + m_normalW * m_width2 + m_normalH * m_height2;
}

double bodies::Box::computeVolume(void) const
{
    return 8.0 * m_length2 * m_width2 * m_height2;
}

void bodies::Box::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool bodies::Box::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections)
{  
    if (distanceSQR(m_center, origin, dir) > m_radiusB) return false;

    double t_near = -INFINITY;
    double t_far  = INFINITY;
    
    for (int i = 0; i < 3; i++)
    {
	btVector3 &vN = i == 0 ? m_normalL : (i == 1 ? m_normalW : m_normalH);
	double dp = vN.dot(dir);
	
	if (fabs(dp) > ZERO)
	{
	    double t1 = vN.dot(m_corner1 - origin) / dp;
	    double t2 = vN.dot(m_corner2 - origin) / dp;
	    
	    if (t1 > t2)
		std::swap(t1, t2);
	    
	    if (t1 > t_near)
		t_near = t1;
	    
	    if (t2 < t_far)
		t_far = t2;
	    
	    if (t_near > t_far)
		return false;
	    
	    if (t_far < 0.0)
		return false;
	}
	else
	{
	    if (i == 0)
	    {
		if ((std::min(m_corner1.y(), m_corner2.y()) > origin.y() ||
		     std::max(m_corner1.y(), m_corner2.y()) < origin.y()) && 
		    (std::min(m_corner1.z(), m_corner2.z()) > origin.z() ||
		     std::max(m_corner1.z(), m_corner2.z()) < origin.z()))
		    return false;
	    }
	    else
	    {
		if (i == 1)
		{
		    if ((std::min(m_corner1.x(), m_corner2.x()) > origin.x() ||
			 std::max(m_corner1.x(), m_corner2.x()) < origin.x()) && 
			(std::min(m_corner1.z(), m_corner2.z()) > origin.z() ||
			 std::max(m_corner1.z(), m_corner2.z()) < origin.z()))
			return false;
		}
		else
		    if ((std::min(m_corner1.x(), m_corner2.x()) > origin.x() ||
			 std::max(m_corner1.x(), m_corner2.x()) < origin.x()) && 
			(std::min(m_corner1.y(), m_corner2.y()) > origin.y() ||
			 std::max(m_corner1.y(), m_corner2.y()) < origin.y()))
			return false;
	    }
	}
    }
    
    if (intersections)
    {
	if (t_far - t_near > ZERO)
	{
	    intersections->push_back(t_near * dir + origin);
	    intersections->push_back(t_far  * dir + origin);
	}
	else
	    intersections->push_back(t_far * dir + origin);
    }
    
    return true;
}

bool bodies::ConvexMesh::containsPoint(const btVector3 &p) const
{
    btVector3 ip = (m_iPose * p) / m_scale;
    return isPointInsidePlanes(ip);
}

void bodies::ConvexMesh::useDimensions(const shapes::Shape *shape)
{  
    const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
    m_planes.clear();
    m_triangles.clear();
    m_vertices.clear();
    m_vertDists.clear();
    m_meshRadiusB = 0.0;
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
	std::cout << "Convex hull has " << hr.m_OutputVertices.size() << " vertices (down from " << mesh->vertexCount << "), " << hr.mNumFaces << " faces" << std::endl;

	m_vertices.reserve(hr.m_OutputVertices.size());
	btVector3 sum(0, 0, 0);	
	
	for (int j = 0 ; j < hr.m_OutputVertices.size() ; ++j)
	{
	    m_vertices.push_back(hr.m_OutputVertices[j]);
	    m_vertDists.push_back(hr.m_OutputVertices[j].length());
	    sum = sum + hr.m_OutputVertices[j];
	}
	
	m_meshCenter = sum / (double)(hr.m_OutputVertices.size());
	for (unsigned int j = 0 ; j < m_vertices.size() ; ++j)
	{
	    double dist = m_vertices[j].distance2(m_meshCenter);
	    if (dist > m_meshRadiusB)
		m_meshRadiusB = dist;
	}
	m_meshRadiusB = sqrt(m_meshRadiusB);
	
	m_triangles.reserve(hr.m_Indices.size());
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
		    std::cerr << "Approximate plane: " << behindPlane << " of " << m_vertices.size() << " points are behind the plane" << std::endl;
		
		m_planes.push_back(planeEquation);

		m_triangles.push_back(hr.m_Indices[j * 3 + 0]);
		m_triangles.push_back(hr.m_Indices[j * 3 + 1]);
		m_triangles.push_back(hr.m_Indices[j * 3 + 2]);
	    }
	}
    }
    else
	std::cerr << "Unable to compute convex hull.";
    
    hl.ReleaseResult(hr);    
    delete[] vertices;
}

void bodies::ConvexMesh::updateInternalData(void) 
{
    m_iPose = m_pose.inverse();
    m_center = m_pose.getOrigin() + m_meshCenter;
    m_radiusB = m_meshRadiusB * m_scale + m_padding;
}

void bodies::ConvexMesh::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool bodies::ConvexMesh::isPointInsidePlanes(const btVector3& point) const
{
    unsigned int numplanes = m_planes.size();
    for (unsigned int i = 0 ; i < numplanes ; ++i)
    {
	const btVector4& plane = m_planes[i];
	btScalar dist = plane.dot(point) + plane.getW() - m_padding - btScalar(1e-6);
	if (dist > btScalar(0))
	    return false;
    }
    return true;
}

unsigned int bodies::ConvexMesh::countVerticesBehindPlane(const btVector4& planeNormal) const
{
    unsigned int numvertices = m_vertices.size();
    unsigned int result = 0;
    for (unsigned int i = 0 ; i < numvertices ; ++i)
    {
	btScalar dist = planeNormal.dot(m_vertices[i]) + planeNormal.getW() - btScalar(1e-6);
	if (dist > btScalar(0))
	    result++;
    }
    return result;
}

double bodies::ConvexMesh::computeVolume(void) const
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

namespace bodies
{   

    // temp structure for intersection points (used for ordering them)
    struct intersc
    {
	intersc(const btVector3 &_pt, const double _tm) : pt(_pt), time(_tm) {}
	
	btVector3 pt;
	double    time;
    };
    
    struct interscOrder
    {
	bool operator()(const intersc &a, const intersc &b) const
	{
	    return a.time < b.time;
	}
	
    };
    
}

bool bodies::ConvexMesh::intersectsRay(const btVector3& origin, const btVector3& dir, std::vector<btVector3> *intersections)
{
    if (distanceSQR(m_center, origin, dir) > m_radiusB * m_radiusB) return false;
    
    // transform the ray into the coordinate frame of the mesh
    btVector3 orig(m_iPose * origin);
    btVector3 dr(m_iPose.getBasis() * dir);
    
    std::vector<intersc> ipts;
    
    bool result = false;
    
    // for each triangle 
    const unsigned int nt = m_triangles.size() / 3;
    for (unsigned int i = 0 ; i < nt ; ++i)
    {
	btScalar tmp = m_planes[i].dot(dr);
	if (fabs(tmp) > ZERO)
	{
	    double t = -(m_planes[i].dot(orig) + m_planes[i].getW()) / tmp;
	    if (t > 0.0)
	    {
		const int i3 = 3*i;
		const int v1 = m_triangles[i3 + 0];
		const int v2 = m_triangles[i3 + 1];
		const int v3 = m_triangles[i3 + 2];

		// this is not the best way to do scaling ....
		const btVector3 a = m_vertices[v1] * (m_scale + (m_vertDists[v1] > ZERO ? m_padding / m_vertDists[v1] : 0.0));
		const btVector3 b = m_vertices[v2] * (m_scale + (m_vertDists[v2] > ZERO ? m_padding / m_vertDists[v2] : 0.0));
		const btVector3 c = m_vertices[v3] * (m_scale + (m_vertDists[v3] > ZERO ? m_padding / m_vertDists[v3] : 0.0));
		
		btVector3 cb(c - b);
		btVector3 ab(a - b);
		
		// intersection of the plane defined by the triangle and the ray
		btVector3 P(orig + dr * t);
		
		// check if it is inside the triangle
		btVector3 pb(P - b);
		btVector3 c1(cb.cross(pb));
		btVector3 c2(cb.cross(ab));
		if (c1.dot(c2) < 0.0)
		    continue;
		btVector3 ca(c - a);
		btVector3 pa(P - a);
		btVector3 ba(-ab);
		
		c1 = ca.cross(pa);
		c2 = ca.cross(ba);
		if (c1.dot(c2) < 0.0)
		    continue;
		
		c1 = ba.cross(pa);
		c2 = ba.cross(ca);
		
		if (c1.dot(c2) < 0.0)
		    continue;
		
		result = true;
		if (intersections)
		{
		    intersc ip(P, t);
		    ipts.push_back(ip);
		}
		else
		    break;
	    }
	}
    }

    if (intersections)
    {
	std::sort(ipts.begin(), ipts.end(), interscOrder());
	for (unsigned int i = 0 ; i < ipts.size() ; ++i)
	    intersections->push_back(ipts[i].pt);
    }
    
    return result;
}
