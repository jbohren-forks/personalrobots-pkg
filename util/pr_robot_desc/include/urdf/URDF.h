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

#ifndef URDF_PARSER_
#define URDF_PARSER_

#include <string>
#include <vector>
#include <map>

/** @htmlinclude ../../manifest.html
    
    Universal Robot Description Format (URDF) parser */

class URDF
{
 public:
    
    struct Link
    {
	
	struct Geometry
	{
	    Geometry(void)
	    {
		type = UNKNOWN;
		size[0] = size[1] = size[2] = 0.0;
	    }
	    
	    virtual ~Geometry(void)
	    {
	    }
	    
	    virtual void print(FILE *out = stdout, std::string indent = "");
	    
	    enum
		{
		    UNKNOWN, SPHERE, BOX, CYLINDER, MESH
		}       type;
	    std::string name;
	    double      size[3];
	    std::string filename;
	};
	
	struct Actuator
	{
	    Actuator(void)
	    {
		polymap[0] = polymap[1] = polymap[2] = 0.0;
		reduction = 0.0;
		port = 0;
	    }
	    
	    virtual ~Actuator(void)
	    {
	    }
	    
	    virtual void print(FILE *out = stdout, std::string indent = "");
	    
	    std::string  name;
	    std::string  motor;
	    std::string  ip;
	    unsigned int port;
	    double       reduction;
	    double       polymap[3];
	};
	
	struct Joint
	{
	    Joint(void)
	    {
		axis[0] = axis[1] = axis[2] = 0.0;
		anchor[0] = anchor[1] = anchor[2] = 0.0;
		calibration[0] = calibration[1] = 0.0;
		limit[0] = limit[1] = 0.0;
		type = UNKNOWN;
	    }
	    
	    virtual ~Joint(void)
	    {
		for (unsigned int i = 0 ; i < actuators.size() ; ++i)
		    delete actuators[i];
	    }
	    
	    virtual void print(FILE *out = stdout, std::string indent = "");
	    
	    enum
		{
		    UNKNOWN, FIXED, REVOLUTE, PRISMATIC, FLOATING
		}                  type;
	    std::string            name;
	    double                 axis[3];
	    double                 anchor[3];
	    double                 limit[2];
	    double                 calibration[2];
	    std::vector<Actuator*> actuators;
	};
	
	struct Collision
	{
	    Collision(void)
	    {
		xyz[0] = xyz[1] = xyz[2] = 0.0;
		rpy[0] = rpy[1] = rpy[2] = 0.0;
		geometry = new Geometry();
	    }
	    
	    virtual ~Collision(void)
	    {
		if (geometry)
		    delete geometry;
	    }

	    virtual void print(FILE *out = stdout, std::string indent = "");
	    
	    std::string  name;	    
	    double       xyz[3];
	    double       rpy[3];
	    std::string  material;
	    Geometry    *geometry;
	};
	
	struct Inertial
	{
	    Inertial(void)
	    {
		mass = 0.0;
		com[0] = com[1] = com[2] = 0.0;
		inertia[0] = inertia[1] = inertia[2] = inertia[3] = inertia[4] = inertia[5] = 0.0;
	    }
	    
	    virtual ~Inertial(void)
	    {
	    }
	    
	    virtual void print(FILE *out = stdout, std::string indent = "");

	    std::string name;
	    double      mass;
	    double      inertia[6];
	    double      com[3];
	};
	
	struct Visual
	{
	    Visual(void)
	    {
		xyz[0] = xyz[1] = xyz[2] = 0.0;
		rpy[0] = rpy[1] = rpy[2] = 0.0;
		geometry = new Geometry();
	    }
	    
	    virtual ~Visual(void)
	    {
		if (geometry)
		    delete geometry;
	    }
	    
	    virtual void print(FILE *out = stdout, std::string indent = "");

	    std::string  name;
	    double       xyz[3];
	    double       rpy[3];
	    std::string  material;
	    Geometry    *geometry;
	};
	
	Link(void)
	{
	    parent = NULL;
	    xyz[0] = xyz[1] = xyz[2] = 0.0;
	    rpy[0] = rpy[1] = rpy[2] = 0.0;
	    inertial  = new Inertial();
	    visual    = new Visual();
	    collision = new Collision();
	    joint     = new Joint();
	}
	
	virtual ~Link(void)
	{
	    if (inertial)
		delete inertial;
	    if (visual)
		delete visual;
	    if (collision)
		delete collision;
	    if (joint)
		delete joint;
	}
	
	virtual void print(FILE *out = stdout, std::string indent = "");
	
	Link              *parent;
	std::string        parentName;
	std::string        name;
	std::vector<Link*> children;
	
	Inertial          *inertial;
	Visual            *visual;
	Collision         *collision;
	Joint             *joint;
	
	double             rpy[3];
	double             xyz[3];
    };
    
    struct Sensor : public Link
    {
	
	Sensor(void)
	{
	    type = UNKNOWN;
	}
	
	virtual ~Sensor(void)
	{
	}
	
	virtual void print(FILE *out = stdout, std::string indent = "");
	
	enum
	{
	    UNKNOWN, CAMERA
	}           type;
	std::string calibration;
    };
    
    explicit URDF(const char *filename = NULL)
    {   
	m_paths.push_back("");
	if (filename)
	    load(filename);
    }
    
    virtual ~URDF(void)
    {
	freeMemory();
    }
    
    virtual void clear(void);
    virtual bool load(const char *filename);
    virtual void print(FILE *out = stdout);
    
 protected:

    /* free the memory allocate in this class */
    void freeMemory(void);
    
    void  addPath(const char *filename);
    char* findFile(const char *filename);

    /* parse the URDF document */
    virtual bool parse(const void *data);
    virtual void ignoreNode(const void *data);
    
    std::string                  m_source;
    std::vector<std::string>     m_paths;
    
    std::string                  m_name;
    std::map<std::string, Link*> m_links; // contains sensors too (casted down)
    std::vector<Link*>           m_roots; // contains the links that are connected to the world (have no parent)    

    std::map<std::string, Link::Collision*> m_collision;
    std::map<std::string, Link::Joint*>     m_joints;
    std::map<std::string, Link::Inertial*>  m_inertial;
    std::map<std::string, Link::Visual*>    m_visual;
    std::map<std::string, Link::Geometry*>  m_geoms;
    std::map<std::string, Link::Actuator*>  m_actuators;

 private:
    
    /* utility functions for parsing */
    void loadLink(const void *data);
    void loadSensor(const void *data);
    void loadActuator(const void *data, Link::Actuator *actuator);
    void loadJoint(const void *data, Link::Joint *joint);
    void loadGeometry(const void *data, Link::Geometry *geometry);
    void loadCollision(const void *data, Link::Collision *collision);
    void loadVisual(const void *data, Link::Visual *visual);
    void loadInertial(const void *data, Link::Inertial *inertial);
    void defaultConstants(void);
    void getChildrenAndAttributes(const void *data, std::vector<const void *> &children, std::vector<const void *> &attributes) const;
    unsigned int loadValues(const void *data, unsigned int count, double *vals);
    std::string  extractName(std::vector<const void *> &attributes);    
    
    
    /* temporary storage for information during parsing; should not be used elsewhere */
    std::map<std::string, std::string> m_constants;  // constants 
    std::map<std::string, const void*> m_templates;  // templates
    std::vector<const void*>           m_stage2;     // xml nodes that should be processed after all templates and constants are read
    std::vector<void*>                 m_docs;       // pointer to loaded documents
    
};


#endif
