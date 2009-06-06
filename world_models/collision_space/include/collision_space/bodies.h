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

#ifndef COLLISION_SPACE_POINT_INCLUSION_
#define COLLISION_SPACE_POINT_INCLUSION_

#include <planning_models/shapes.h>
#include <LinearMath/btTransform.h>
#include <cstdlib>
#include <vector>

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
	
	struct BoundingSphere
	{
	    btVector3 center;
	    double    radius;
	};
		
	/** A body is a shape + its pose. Point inclusion can be
	    tested, volumes and bounding spheres can be computed.*/
	class Body
	{
	public:
	    
	    Body(void)
	    {
		m_scale = 1.0;
		m_padding = 0.0;
		m_pose.setIdentity();
	    }
	    
	    virtual ~Body(void)
	    {
	    }
	    
	    /** If the dimension of the body should be scaled, this
		method sets the scale. Default is 1.0 */
	    void setScale(double scale)
	    {
		m_scale = scale;
		updateInternalData();
	    }
	    
	    /** Retrieve the current scale */
	    double getScale(void) const
	    {
		return m_scale;
	    }
	    
	    /** If constant padding should be added to the body, this
		method sets the padding. Default is 0.0 */
	    void setPadding(double padd)
	    {
		m_padding = padd;
		updateInternalData();
	    }
	    
	    /** Retrieve the current padding */
	    double getPadding(void) const
	    {
		return m_padding;
	    }
	    
	    /** Set the pose of the body. Default is identity */
	    void setPose(const btTransform &pose)
	    {
		m_pose = pose;
		updateInternalData();
	    }
	    
	    /** Retrieve the pose of the body */
	    const btTransform& getPose(void) const
	    {
		return m_pose;
	    }
	    
	    /** Set the dimensions of the body (from corresponding shape) */
	    void setDimensions(const planning_models::shapes::Shape *shape)
	    {
		useDimensions(shape);
		updateInternalData();
	    }
	    
	    /** Check is a point is inside the body */
	    bool containsPoint(double x, double y, double z) const
	    {
		return containsPoint(btVector3(btScalar(x), btScalar(y), btScalar(z)));
	    }
	    
	    /** Check is a point is inside the body */
	    virtual bool containsPoint(const btVector3 &p) const = 0;	
	    
	    /** Compute the volume of the body. This method includes
		changes induced by scaling and padding */
	    virtual double computeVolume(void) const = 0;
	    
	    /** Compute the bounding radius for the body, in its current
		pose. Scaling and padding are accounted for. */
	    virtual void computeBoundingSphere(BoundingSphere &sphere) const = 0;
	    
	protected:
	    
	    virtual void updateInternalData(void) = 0;
	    virtual void useDimensions(const planning_models::shapes::Shape *shape) = 0;
	    
	    btTransform m_pose;	
	    double      m_scale;
	    double      m_padding;	    
	};
	
	class Sphere : public Body
	{
	public:
            Sphere(void) : Body()
	    {
		m_radius = 0.0;
	    }
	    
	    Sphere(const planning_models::shapes::Shape *shape) : Body()
	    {
		setDimensions(shape);
	    }
	    
	    virtual ~Sphere(void)
	    {
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const;
	    virtual double computeVolume(void) const;
	    virtual void computeBoundingSphere(BoundingSphere &sphere) const;

	protected:
	    
	    virtual void useDimensions(const planning_models::shapes::Shape *shape);
	    virtual void updateInternalData(void);
	    
	    btVector3 m_center;
	    double    m_radius;	
	    double    m_radiusU;
	    double    m_radius2;		    
	};
        
	class Cylinder : public Body
	{
	public:
	    Cylinder(void) : Body()
	    {
		m_length = m_radius = 0.0;
	    }

	    Cylinder(const planning_models::shapes::Shape *shape) : Body()
	    {
		setDimensions(shape);
	    }
	    
	    virtual ~Cylinder(void)
	    {
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const;
	    virtual double computeVolume(void) const;
	    virtual void computeBoundingSphere(BoundingSphere &sphere) const;

	protected:
	    
	    virtual void useDimensions(const planning_models::shapes::Shape *shape);
	    virtual void updateInternalData(void);

	    btVector3 m_center;
	    btVector3 m_normalH;
	    btVector3 m_normalB1;
	    btVector3 m_normalB2;
	    
	    double    m_length;
	    double    m_length2;	
	    double    m_radius;
	    double    m_radiusU;
	    double    m_radiusB;
	    double    m_radius2;
	};
	
	
	class Box : public Body
	{
	public: 
	    Box(void) : Body()
	    {
		m_length = m_width = m_height = 0.0;
	    }
	    
	    Box(const planning_models::shapes::Shape *shape) : Body()
	    {
		setDimensions(shape);
	    }
	    
	    virtual ~Box(void)
	    {
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const;
	    virtual double computeVolume(void) const;
	    virtual void computeBoundingSphere(BoundingSphere &sphere) const;

	protected:
	    
	    virtual void useDimensions(const planning_models::shapes::Shape *shape); // (x, y, z) = (length, width, height)	    
	    virtual void updateInternalData(void);
	    
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
	    double    m_radiusB;
	};
	

	class ConvexMesh : public Body
	{
	public:
	    
	    ConvexMesh(void) : Body()
	    {
		m_meshCenter.setValue(btScalar(0), btScalar(0), btScalar(0));
	    }

	    ConvexMesh(const planning_models::shapes::Shape *shape) : Body()
	    {
		m_meshCenter.setValue(btScalar(0), btScalar(0), btScalar(0));
		setDimensions(shape);
	    }
	    
	    virtual ~ConvexMesh(void)
	    {
	    }
	    
	    virtual bool containsPoint(const btVector3 &p) const;
	    virtual double computeVolume(void) const;
	    virtual void computeBoundingSphere(BoundingSphere &sphere) const;

	protected:
	    
	    virtual void useDimensions(const planning_models::shapes::Shape *shape);
	    virtual void updateInternalData(void);
	    
	    unsigned int countVerticesBehindPlane(const btVector4& planeNormal) const;
	    bool isPointInsidePlanes(const btVector3& point) const;
	    
	    std::vector<btVector4>    m_planes;
	    std::vector<btVector3>    m_vertices;
	    std::vector<unsigned int> m_triangles;
	    btTransform               m_iPose;

	    btVector3                 m_center;
	    btVector3                 m_meshCenter;
	    double                    m_radiusB;
	};
	
	
	/** Create a body from a given shape */
	Body* createBodyFromShape(const planning_models::shapes::Shape *shape);

	/** Compute a bounding sphere to enclose a set of bounding spheres */
	void mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere);
    }
}

#endif
