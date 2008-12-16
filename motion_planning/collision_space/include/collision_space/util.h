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

#ifndef COLLISION_SPACE_UTIL_
#define COLLISION_SPACE_UTIL_

#include <LinearMath/btTransform.h>
#include <cmath>

/**
   This set of classes allows quickly detecting whether a given point
   is inside an object or not. Only basic (simple) types of objects
   are supported: spheres, cylinders, boxes. This capability is useful
   when removing points from inside the robot (when the robot sees its
   arms, for example).
*/

namespace collision_space
{

    namespace bodies
    {
	
	class Shape
	{
	public:
	    Shape(void)
	    {
		m_scale = 1.0;	    
	    }
	    
	    virtual ~Shape(void)
	    {
	    }
	    
	    void setScale(double scale)
	    {
		m_scale = scale;
		updateInternalData();
	    }
	    
	    void setPose(const btTransform &pose)
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
		return containsPoint(btVector3(btScalar(x), btScalar(y), btScalar(z)));
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const = 0;	
	    
	protected:
	    
	    virtual void updateInternalData(void) = 0;
	    virtual void useDimensions(const double *dims) = 0;
	    
	    btTransform m_pose;	
	    double      m_scale;	    
	};
	
	class Sphere : public Shape
	{
	public:
        Sphere(void) : Shape()
	    {
		m_radius = 0.0;
	    }
	    
	    virtual ~Sphere(void)
	    {
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const 
	    {
		return (m_center - p).length2() < m_radius2;
	    }
	    
	protected:
	    
	    virtual void useDimensions(const double *dims) // radius
	    {
		m_radius = dims[0];
	    }
	    
	    virtual void updateInternalData(void)
	    {
		m_radius2 = m_radius * m_radius * m_scale * m_scale;
		m_center = m_pose.getOrigin();
	    }
	    
	    btVector3 m_center;
	    double    m_radius;	
	    double    m_radius2;		    
	};
        
	class Cylinder : public Shape
	{
	public:
        Cylinder(void) : Shape(), m_normalH(btScalar(0.0), btScalar(0.0), btScalar(1.0))
	    {
		m_length = m_radius = 0.0;
	    }
	    
	    virtual ~Cylinder(void)
	    {
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const 
	    {
		btVector3 v = p - m_center;		
		double pH = v.dot(m_normalH);
		
		if (fabs(pH) > m_length2)
		    return false;
		
		double pB1 = v.dot(m_normalB1);
		double pB2 = v.dot(m_normalB2);
		
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
		m_center = m_pose.getOrigin();
		
		m_normalH.setValue(btScalar(0.0), btScalar(0.0), btScalar(1.0));
		m_normalH = m_pose * m_normalH;
		
		m_normalB1.setValue(btScalar(1.0), btScalar(0.0), btScalar(0.0));
		m_normalB1 = m_pose * m_normalB1;

		m_normalB2.setValue(btScalar(0.0), btScalar(1.0), btScalar(0.0));
		m_normalB2 = m_pose * m_normalB2;
	    }
	    
	    btVector3 m_center;
	    btVector3 m_normalH;
	    btVector3 m_normalB1;
	    btVector3 m_normalB2;
	    
	    double    m_length;
	    double    m_length2;	
	    double    m_radius;	
	    double    m_radius2;
	};
	
	
	class Box : public Shape
	{
	public: 
        Box(void) : Shape()
	    {
		m_length = m_width = m_height = 0.0;
	    }
	    
	    virtual ~Box(void)
	    {
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const 
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
		
		m_center = m_pose.getOrigin();
		
		m_normalH.setValue(btScalar(0.0), btScalar(0.0), btScalar(1.0));
		m_normalH = m_pose * m_normalH;

		m_normalL.setValue(btScalar(1.0), btScalar(0.0), btScalar(0.0));
		m_normalL = m_pose * m_normalL;

		m_normalW.setValue(btScalar(0.0), btScalar(1.0), btScalar(0.0));
		m_normalW = m_pose * m_normalW;
	    }
	    
	    btVector3 m_center;
	    btVector3 m_normalL;
	    btVector3 m_normalW;
	    btVector3 m_normalH;
	    
	    double    m_length;
	    double    m_width;
	    double    m_height;	
	    double    m_length2;
	    double    m_width2;
	    double    m_height2;	
	};
	
    }
}

#endif
