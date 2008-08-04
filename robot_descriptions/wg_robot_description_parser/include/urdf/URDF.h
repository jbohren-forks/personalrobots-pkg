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

#include <tinyxml/tinyxml.h>
#include <istream>
#include <string>
#include <vector>
#include <map>

/** @htmlinclude ../../manifest.html
    
    Universal Robot Description Format (URDF) parser */

namespace robot_desc
{
    
    class URDF
    {
    public:
        
	/* This class encapsulates data that can be attached to various tags in the format */
	class Data
	{
	public:
            
	    Data(void)
	    {
	    }
            
	    virtual ~Data(void)
	    {
	    }
            
	    void getDataTagTypes(std::vector<std::string> &types) const;
	    void getDataTagNames(const std::string &type, std::vector<std::string> &names) const;
	    std::map<std::string, std::string> getDataTagValues(const std::string &type, const std::string &name) const;
	    std::map<std::string, const TiXmlElement*> getDataTagXML(const std::string &type, const std::string &name) const;
	    
	    bool hasDefault(const std::string &key) const;
	    std::string getDefaultValue(const std::string &key) const;
	    const TiXmlElement* getDefaultXML(const std::string &key) const;
	    
	    virtual void print(FILE *out = stdout, std::string indent = "") const;
            
	    void add(const std::string &type, const std::string &name, const std::string &key, const std::string &value);
	    void add(const std::string &type, const std::string &name, const std::string &key, const TiXmlElement *value);
            
	    Data& operator=(const Data &rhs)
	    { 
		m_data = rhs.m_data;
		return *this;
	    }
	protected:
            
	    struct Element
	    {
		Element(void)
		{
		    xml = NULL;
		}
                
		std::string         str;
		const TiXmlElement *xml;
	    };
            
	    std::map < std::string, std::map < std::string, std::map < std::string, Element > > > m_data;
	};
        
	struct Group;
        
	struct Actuator
	{
	    Actuator(void)
	    {
		isSet["name"] = false;
	    }
            
	    virtual ~Actuator(void)
	    {
	    }
            
	    virtual void print(FILE *out = stdout, std::string indent = "") const;
            
	    std::string                 name;
	    Data                        data;
	    std::map<std::string, bool> isSet;
	};
	
	struct Transmission
	{
	    Transmission(void)
	    {
		isSet["name"] = false;
	    }
            
	    virtual ~Transmission(void)
	    {
	    }
            
	    virtual void print(FILE *out = stdout, std::string indent = "") const;
            
	    std::string                 name;
	    Data                        data;
	    std::map<std::string, bool> isSet;
	};
        
	struct Link
	{
	    
	    struct Geometry
	    {
		Geometry(void)
		{
		    type = UNKNOWN;
		    nsize = -1;
		    size[0] = size[1] = size[2] = 0.0;
		    isSet["name"] = false;
		    isSet["type"] = false;
		    isSet["size"] = false;
		    isSet["filename"] = false;
		}
                
		virtual ~Geometry(void)
		{
		}
                
		virtual void print(FILE *out = stdout, std::string indent = "") const;
                
		enum
		    {
			UNKNOWN, SPHERE, BOX, CYLINDER, MESH
		    }       type;
		std::string                 name;
		double                      size[3];
		int                         nsize;	
		std::string                 filename;
		Data                        data;
		std::map<std::string, bool> isSet;
	    };
            
	    struct Joint
	    {
		Joint(void)
		{
		    axis[0] = axis[1] = axis[2] = 0.0;
		    anchor[0] = anchor[1] = anchor[2] = 0.0;
		    limit[0] = limit[1] = 0.0;
		    type = UNKNOWN;
		    isSet["name"] = false;
		    isSet["type"] = false;
		    isSet["axis"] = false;
		    isSet["anchor"] = false;
		    isSet["limit"] = false;
		    isSet["calibration"] = false;
		}
		
		virtual ~Joint(void)
		{
		}
                
		virtual void print(FILE *out = stdout, std::string indent = "") const;
                
		enum
		    {
			UNKNOWN, FIXED, REVOLUTE, PRISMATIC, PLANAR, FLOATING
		    }                       type;
		std::string                 name;	
		double                      axis[3];       // vector describing the axis of rotation: (x,y,z)
		double                      anchor[3];     // point about which the axis defines the rotation: (x,y,z)
		double                      limit[2];      // the joint limits: (min, max)
		std::string                 calibration;
		Data                        data;
		std::map<std::string, bool> isSet;
	    };
	    
	    struct Collision
	    {
		Collision(void)
		{
		    xyz[0] = xyz[1] = xyz[2] = 0.0;
		    rpy[0] = rpy[1] = rpy[2] = 0.0;
		    verbose = false;
		    geometry = new Geometry();
		    isSet["name"] = false;
		    isSet["verbose"] = false;
		    isSet["xyz"] = false;
		    isSet["rpy"] = false;
		    isSet["material"] = false;
		    isSet["geometry"] = false;
		}
                
		virtual ~Collision(void)
		{
		    if (geometry)
			delete geometry;
		}
                
		virtual void print(FILE *out = stdout, std::string indent = "") const;
                
		std::string                 name;
		bool                        verbose;
		double                      xyz[3];
		double                      rpy[3];
		std::string                 material;
		Geometry                   *geometry;
		Data                        data;    
		std::map<std::string, bool> isSet;
	    };
            
	    struct Inertial
	    {
		Inertial(void)
		{
		    mass = 0.0;
		    com[0] = com[1] = com[2] = 0.0;
		    inertia[0] = inertia[1] = inertia[2] = inertia[3] = inertia[4] = inertia[5] = 0.0;
		    isSet["name"] = false;
		    isSet["mass"] = false;
		    isSet["inertia"] = false;
		    isSet["com"] = false;
		}
                
		virtual ~Inertial(void)
		{
		}
                
		virtual void print(FILE *out = stdout, std::string indent = "") const;
                
		std::string                 name;
		double                      mass;
		double                      inertia[6];
		double                      com[3];
		Data                        data;
		std::map<std::string, bool> isSet;
	    };
            
	    struct Visual
	    {
		Visual(void)
		{
		    xyz[0] = xyz[1] = xyz[2] = 0.0;
		    rpy[0] = rpy[1] = rpy[2] = 0.0;
		    scale[0] = scale[1] = scale[2] = 1.0;
		    geometry = new Geometry();
		    isSet["name"] = false;
		    isSet["xyz"] = false;
		    isSet["rpy"] = false;
		    isSet["scale"] = false;
		    isSet["material"] = false;
		    isSet["geometry"] = false;
		}
                
		virtual ~Visual(void)
		{
		    if (geometry)
			delete geometry;
		}
                
		virtual void print(FILE *out = stdout, std::string indent = "") const;
                
		std::string                 name;
		double                      xyz[3];
		double                      rpy[3];
		double                      scale[3];
		std::string                 material;
		Geometry                   *geometry;
		Data                        data;
		std::map<std::string, bool> isSet;
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
		isSet["name"] = false;
		isSet["parent"] = false;
		isSet["inertial"] = false;
		isSet["visual"] = false;
		isSet["collision"] = false;
		isSet["joint"] = false;
		isSet["xyz"] = false;
		isSet["rpy"] = false;
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
            
	    bool insideGroup(Group *group) const;
	    bool insideGroup(const std::string &group) const;
	    
	    virtual bool canSense(void) const;
	    virtual void print(FILE *out = stdout, std::string indent = "") const;
            
	    Link                       *parent;
	    std::string                 parentName;
	    std::string                 name;
	    std::vector<Link*>          children;
            
	    Inertial                   *inertial;
	    Visual                     *visual;
	    Collision                  *collision;
	    Joint                      *joint;
            
	    double                      rpy[3];
	    double                      xyz[3];
	    Data                        data;
	    
	    std::vector<Group*>         groups;
	    std::vector<bool>           inGroup;

	    std::map<std::string, bool> isSet;
	};
        
	struct Sensor : public Link
	{	    
	    Sensor(void)
	    {
		type = UNKNOWN;
		isSet["type"] = false;
		isSet["calibration"] = false;		
	    }
            
	    virtual ~Sensor(void)
	    {
	    }
            
	    virtual bool canSense(void) const;
	    virtual void print(FILE *out = stdout, std::string indent = "") const;
            
	    enum
	    {
		UNKNOWN, LASER, CAMERA, STEREO_CAMERA
	    }           type;
	    std::string calibration;
	};
	
	struct Frame
	{
	    Frame(void)
	    {
		link = NULL;
		type = UNKNOWN;
		xyz[0] = xyz[1] = xyz[2] = 0.0;
		rpy[0] = rpy[1] = rpy[2] = 0.0;
		isSet["name"] = false;
		isSet["parent"] = false;
		isSet["child"] = false;
		isSet["rpy"] = false;
		isSet["xyz"] = false;		
	    }
	    
	    virtual ~Frame(void)
	    {
	    }
	    
	    virtual void print(FILE *out = stdout, std::string indent = "") const;

	    std::string                 name;
	    std::string                 linkName;
	    Link*                       link;
	    
	    enum
	    {
		UNKNOWN, PARENT, CHILD
	    }                           type;
	    
	    double                      rpy[3];
	    double                      xyz[3];
	    Data                        data;

	    std::vector<Group*>         groups;
	    std::map<std::string, bool> isSet;
	};
	
	struct Group
	{
	    Group(void)
	    {
	    }
            
	    virtual ~Group(void)
	    {
	    }
	    
	    bool hasFlag(const std::string &flag) const;
	    bool isRoot(const Link* link) const;
	    
	    std::string              name;
	    std::vector<std::string> flags;
	    std::vector<std::string> linkNames;
	    std::vector<Link*>       links;
	    std::vector<Link*>       linkRoots;
	    std::vector<std::string> frameNames;
	    std::vector<Frame*>      frames;
	};
	
	explicit
	URDF(const char *filename = NULL)
	{   
	    m_paths.push_back("");
	    if (filename)
		loadFile(filename);
	}
        
	virtual ~URDF(void)
	{
	    freeMemory();
	}
        
	virtual void clear(void);
	virtual bool loadFile(const char *filename);
	virtual bool loadFile(FILE *file);
	virtual bool loadString(const char *data);
	virtual bool loadStream(std::istream &is);
	virtual void print(FILE *out = stdout) const;
        
	bool containsCycle(unsigned int index) const;
	void sanityCheck(void) const;
	
	const std::string& getRobotName(void) const;
	unsigned int getDisjointPartCount(void) const;
	Link* getDisjointPart(unsigned int index) const;
	bool isRoot(const Link* link) const;

	void getLinks(std::vector<Link*> &links) const;
	void getActuators(std::vector<Actuator*> &actuators) const;
	void getTransmissions(std::vector<Transmission*> &transmissions) const;
        void getFrames(std::vector<Frame*> &frames) const;
	
	void getGroupNames(std::vector<std::string> &groups) const;
	Group* getGroup(const std::string &name) const;
	void getGroups(std::vector<Group*> &groups) const;
	
	const Data& getData(void) const;
        
    protected:
        
	/* free the memory allocate in this class */
	void freeMemory(void);
        
	void  addPath(const char *filename);
	char* findFile(const char *filename);
        
	/* parse the URDF document */
	virtual bool parse(const TiXmlNode *node);
	virtual void ignoreNode(const TiXmlNode* node);
        
	/* file processing data elements */
	std::string                  m_source;
	std::vector<std::string>     m_paths;
        
	/* parsed datastructures */
	std::string                          m_name;
	std::map<std::string, Link*>         m_links;     // contains sensors too (casted down)
	std::map<std::string, Group*>        m_groups;
	std::map<std::string, Actuator*>     m_actuators;
	std::map<std::string, Transmission*> m_transmissions;
	std::map<std::string, Frame*>        m_frames;
	Data                                 m_data; // information from data tags
        
	/* simple computed datastructures */
	std::vector<Link*> m_linkRoots; // contains the links that are connected to the world (have no parent)
	
	/* easy access maps */
	std::map<std::string, Link::Collision*> m_collision;
	std::map<std::string, Link::Joint*>     m_joints;
	std::map<std::string, Link::Inertial*>  m_inertial;
	std::map<std::string, Link::Visual*>    m_visual;
	std::map<std::string, Link::Geometry*>  m_geoms;
	
	
        
    private:
        
	/* utility functions for parsing */
	void loadLink(const TiXmlNode *node);
	void loadSensor(const TiXmlNode *node);
	void loadFrame(const TiXmlNode *node);
	void loadActuator(const TiXmlNode *node);
	void loadTransmission(const TiXmlNode *node);
	void loadJoint(const TiXmlNode *node, const std::string &defaultName, Link::Joint *joint);
	void loadGeometry(const TiXmlNode *node, const std::string &defaultName, Link::Geometry *geometry);
	void loadCollision(const TiXmlNode *node, const std::string &defaultName, Link::Collision *collision);
	void loadVisual(const TiXmlNode *node, const std::string &defaultName, Link::Visual *visual);
	void loadInertial(const TiXmlNode *node, const std::string &defaultName, Link::Inertial *inertial);
	void loadData(const TiXmlNode *node, Data *data);
        
	void defaultConstants(void);
	
	void getChildrenAndAttributes(const TiXmlNode* node, std::vector<const TiXmlNode*> &children, std::vector<const TiXmlAttribute*> &attributes) const;
	
	unsigned int loadDoubleValues(const TiXmlNode *node, unsigned int count, double *vals);
	unsigned int loadBoolValues  (const TiXmlNode *node, unsigned int count, bool   *vals);
	
	std::string  extractName(std::vector<const TiXmlAttribute*> &attributes, const std::string &defaultName);    
	
	void clearDocs(void);    
	
	/* print the error location, if known */
	void errorLocation(void) const;
	
	/* temporary storage for information during parsing; should not be used elsewhere */
	std::map<std::string, std::string>      m_constants;  // constants 
	std::map<std::string, const TiXmlNode*> m_templates;  // templates
	std::vector<const TiXmlNode*>           m_stage2;     // xml nodes that should be processed after all templates and constants are read
	std::vector<TiXmlDocument*>             m_docs;       // pointer to loaded documents
	
	std::string                             m_location;   // approximate location in file (used for error messages)
    };
    
} // namespace robot_desc

#endif

