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

#ifndef COLLISION_SPACE_UTIL_
#define COLLISION_SPACE_UTIL_

#include <libTF/Pose3D.h>
#include <cmath>

namespace collision_space
{

    class Object
    {
    public:
	Object(void)
	{
	    m_scale = 1.0;	    
	}

	virtual ~Object(void)
	{
	}
	
	void setScale(double scale)
	{
	    m_scale = scale;
	    updateInternalData();
	}
	
	void setPose(const libTF::Pose3D &pose)
	{
	    m_pose = pose;
	    updateInternalData();
	}
	
	virtual void setDimensions(const double *dims)
	{
	    useDimensions(dims);
	    updateInternalData();	    
	}
	
	bool containsPoint(double x, double y, double z) const
	{
	    libTF::Pose3D::Position pt = { x, y, z };
	    return containsPoint(pt);
	}
	
	virtual bool containsPoint(const libTF::Pose3D::Position &p) const = 0;	
	
    protected:
	
	virtual void updateInternalData(void) = 0;
	virtual void useDimensions(const double *dims) = 0;
	
	libTF::Pose3D m_pose;	
	double        m_scale;
	
    };
    
    class Sphere : public Object
    {
    public:
        Sphere(void) : Object()
	{
	    m_radius = 0.0;
	}
	
	virtual ~Sphere(void)
	{
	}
	
	virtual bool containsPoint(const libTF::Pose3D::Position &p) const 
	{
	    double dx = m_center.x - p.x;
	    double dy = m_center.y - p.y;
	    double dz = m_center.z - p.z;
	    return dx * dx + dy * dy + dz * dz < m_radius2;
	}
		
    protected:

	virtual void useDimensions(const double *dims) // radius
	{
	    m_radius = dims[0];
	}
	
	virtual void updateInternalData(void)
	{
	    m_radius2 = m_radius * m_radius * m_scale * m_scale;
	    
	    m_pose.getPosition(m_center);
	}
	
	libTF::Pose3D::Position m_center;
	double                  m_radius;	
	double                  m_radius2;	
	
    };
        
    class Cylinder : public Object
    {
    public:
        Cylinder(void) : Object()
	{
	    m_length = m_radius = 0.0;
	}
	
	virtual ~Cylinder(void)
	{
	}
	
	virtual bool containsPoint(const libTF::Pose3D::Position &p) const 
	{
	    double vx = p.x - m_center.x;
	    double vy = p.y - m_center.y;
	    double vz = p.z - m_center.z;
	    
	    double pH = vx * m_normalH.x + vy * m_normalH.y + vz * m_normalH.z;
	    
	    if (fabs(pH) > m_length2)
		return false;
	    
	    double pB1 = vx * m_normalB1.x + vy * m_normalB1.y + vz * m_normalB1.z;
	    double pB2 = vx * m_normalB2.x + vy * m_normalB2.y + vz * m_normalB2.z;
	    
	    return pB1 * pB2 < m_radius2;
	}
	
    protected:
	
	virtual void useDimensions(const double *dims) // (length, radius)
	{
	    m_length = dims[0];
	    m_radius = dims[1];
	}

	virtual void updateInternalData(void)
	{
	    m_radius2 = m_radius * m_radius * m_scale * m_scale;
	    m_length2 = m_scale * m_length / 2.0;
	    
	    m_pose.getPosition(m_center);

	    m_normalH.x = m_normalH.y = 0.0; m_normalH.z = 1.0;
	    m_pose.applyToVector(m_normalH);

	    m_normalB1.y = m_normalB1.z = 0.0; m_normalB1.x = 1.0;
	    m_pose.applyToVector(m_normalB1);
	    
	    m_normalB2.x = m_normalB2.z = 0.0; m_normalB2.y = 1.0;
	    m_pose.applyToVector(m_normalB2);
	}
	
	libTF::Pose3D::Position m_center;
	libTF::Pose3D::Vector   m_normalH;
	libTF::Pose3D::Vector   m_normalB1;
	libTF::Pose3D::Vector   m_normalB2;

	double                  m_length;
	double                  m_length2;	
	double                  m_radius;	
	double                  m_radius2;
    };
    

    class Box : public Object
    {
    public: 
        Box(void) : Object()
	{
	    m_length = m_width = m_height = 0.0;
	}
	
	virtual ~Box(void)
	{
	}
	
	virtual bool containsPoint(const libTF::Pose3D::Position &p) const 
	{
	    double vx = p.x - m_center.x;
	    double vy = p.y - m_center.y;
	    double vz = p.z - m_center.z;
	    
	    double pL = vx * m_normalL.x + vy * m_normalL.y + vz * m_normalL.z;
	    
	    if (fabs(pL) > m_length2)
		return false;
	    
	    double pW = vx * m_normalW.x + vy * m_normalW.y + vz * m_normalW.z;

	    if (fabs(pW) > m_width2)
		return false;
	    
	    double pH = vx * m_normalH.x + vy * m_normalH.y + vz * m_normalH.z;

	    if (fabs(pH) > m_height2)
		return false;
	    
	    return true;
	}
	
    protected:
	
	virtual void useDimensions(const double *dims) // (x, y, z) = (length, width, height)
	{
	    m_length = dims[0];
	    m_width  = dims[1];
	    m_height = dims[2];
	}
	
	virtual void updateInternalData(void) 
	{
	    m_length2 = m_scale * m_length / 2.0;
	    m_width2  = m_scale * m_width / 2.0;
	    m_height2 = m_scale * m_height / 2.0;

	    m_pose.getPosition(m_center);
	    
	    m_normalH.x = m_normalH.y = 0.0; m_normalH.z = 1.0;
	    m_pose.applyToVector(m_normalH);

	    m_normalL.y = m_normalL.z = 0.0; m_normalL.x = 1.0;
	    m_pose.applyToVector(m_normalL);
	    
	    m_normalW.x = m_normalW.z = 0.0; m_normalW.y = 1.0;
	    m_pose.applyToVector(m_normalW);
	}
	
	libTF::Pose3D::Position m_center;
	libTF::Pose3D::Vector   m_normalL;
	libTF::Pose3D::Vector   m_normalW;
	libTF::Pose3D::Vector   m_normalH;

	double                  m_length;
	double                  m_width;
	double                  m_height;	
	double                  m_length2;
	double                  m_width2;
	double                  m_height2;	
    };
    
}

#endif
