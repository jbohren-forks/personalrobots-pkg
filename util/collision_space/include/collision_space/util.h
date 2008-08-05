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
	}
	
	void setPose(const libTF::Pose3D &pose)
	{
	    m_pose = pose;
	    m_pose.invert();
	}
	
	bool containsPoint(const libTF::Pose3D::Position &p) const
	{
	    /* bring point in the reference frame described by pose */
	    libTF::Pose3D::Position pt = p;
	    m_pose.applyToPosition(pt);

	    /* since the body is centered at origin, scaling the body is equivalent 
	     * to scaling the coordinates of the point */
	    pt.x /= m_scale;
	    pt.y /= m_scale;
	    pt.z /= m_scale;
	    
	    return containsPt(pt);
	}
	
	bool containsPoint(double x, double y, double z) const
	{
	    /* bring point in the reference frame described by pose */
	    libTF::Pose3D::Position pt = { x, y, z };
	    m_pose.applyToPosition(pt);
	    
	    /* since the body is centered at origin, scaling the body is equivalent 
	     * to scaling the coordinates of the point */
	    pt.x /= m_scale;
	    pt.y /= m_scale;
	    pt.z /= m_scale;
	    
	    return containsPt(pt);
	}
	
    protected:
	
	virtual bool containsPt(const libTF::Pose3D::Position &p) const = 0;

	libTF::Pose3D m_pose;	
	double        m_scale;
	
    };
    
    class Sphere : public Object
    {
    public:
        Sphere(double radius = 0) : Object()
	{
	    setRadius(radius);	    
	}
	
	virtual ~Sphere(void)
	{
	}
	
	void setRadius(double radius)
	{
	    m_radius = radius;
	    m_radius2 = radius * radius;	    
	}
		
    protected:

	virtual bool containsPt(const libTF::Pose3D::Position &p) const 
	{
	    return p.x * p.x + p.y * p.y + p.z * p.z < m_radius2;
	}

	double m_radius2;	
	double m_radius;	
    };
    
    class Cylinder : public Object
    {
    public:
        Cylinder(double length = 0.0, double radius = 0.0) : Object()
	{
	    setDimensions(length, radius);	    
	}
	
	virtual ~Cylinder(void)
	{
	}
	
	void setDimensions(double length, double radius)
	{
	    m_length = length;
	    m_length2 = length / 2.0;
	    m_radius = radius;
	    m_radius2 = radius * radius;
	}
	
    protected:

	virtual bool containsPt(const libTF::Pose3D::Position &p) const 
	{
	    if (fabs(p.z) > m_length2)
		return false;
	    return p.x * p.x + p.y * p.y < m_radius2;
	}
	
	double m_length;
	double m_length2;	
	double m_radius;	
	double m_radius2;
    };
    
    class Box : public Object
    {
    public: 
        Box(double length = 0.0, double width = 0.0, double height = 0.0) : Object()
	{
	    setDimensions(length, width, height);	    
	}
	
	virtual ~Box(void)
	{
	}
	
	void setDimensions(double length, double width, double height) // x, y, z
	{
	    m_length = length;
	    m_length2 = length / 2.0;	    
	    m_width = width;
	    m_width2 = width / 2.0;	    
	    m_height = height;
	    m_height2 = height / 2.0;	    
	}	

    protected:
	
	virtual bool containsPt(const libTF::Pose3D::Position &p) const 
	{
	    return fabs(p.x) < m_length2 && fabs(p.y) < m_width2 && fabs(p.z) < m_height2;
	}
	
	double m_length;
	double m_width;
	double m_height;	
	double m_length2;
	double m_width2;
	double m_height2;	
    };
    

}

#endif
