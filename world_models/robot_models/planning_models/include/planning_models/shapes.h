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

#ifndef PLANNING_MODELS_SHAPES_
#define PLANNING_MODELS_SHAPES_

namespace planning_models
{
    
    namespace shapes
    {
	
	/** A basic definition of a shape. Shapes to be centered at origin */
	class Shape
	{		    
	public:	    
	    Shape(void)
	    {
		type = UNKNOWN;
	    }
	    
	    virtual ~Shape(void)
	    {
	    }
	    
	    enum { UNKNOWN, SPHERE, CYLINDER, BOX } 
		type;
	    
	};
	
	/** Definition of a sphere */
	class Sphere : public Shape
	{
	public:
	    Sphere(void) : Shape()
	    {
		type   = SPHERE;
		radius = 0.0;
	    }
	    
	    Sphere(double r) : Shape()
	    {
		type   = SPHERE;
		radius = r;
	    }
	    
	    double radius; 
	};
	
	/** Definition of a cylinder */
	class Cylinder : public Shape
	{
	public:
	    Cylinder(void) : Shape()
	    {
		type   = CYLINDER;
		length = radius = 0.0;
	    }
	    
	    Cylinder(double r, double l) : Shape()
	    {
		type   = CYLINDER;
		length = l;
		radius = r;
	    }
	    
	    double length, radius; 
	};
	
	/** Definition of a box */
	class Box : public Shape
	{
	public:
	    Box(void) : Shape()
	    {
		type = BOX;
		size[0] = size[1] = size[2] = 0.0;
	    }
	    
	    Box(double x, double y, double z) : Shape()
	    {
		type = BOX;
		size[0] = x;
		size[1] = y;
		size[2] = z;
	    }
	    
	    /** x, y, z */
	    double size[3]; 
	};
	
    }
}

#endif
    
