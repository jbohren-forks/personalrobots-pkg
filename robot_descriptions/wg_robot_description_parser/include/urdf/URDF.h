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
#include <iostream>
#include <string>
#include <vector>
#include <map>

/** @htmlinclude ../../manifest.html
    
    Universal Robot Description Format (URDF) parser */

namespace robot_desc
{
    /** This class contains a parser for URDF documents */
    class URDF
    {
    public:
        
	/** This class encapsulates data that can be attached to various tags in the format */
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
	    
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
            
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
		    str = NULL;		    
		}
		
		~Element(void)
		{
		    if (str)
			delete str;		    
		}		
                
		std::string        *str; // allocated locally 
		const TiXmlElement *xml; // allocated in the XML document
	    };
            
	    std::map < std::string, std::map < std::string, std::map < std::string, Element > > > m_data;
	};
        

	/** Forward declaration of groups */
	struct Group;
        
	/** This class defines actuator instances */
	struct Actuator
	{
	    Actuator(void)
	    {
		isSet["name"] = false;
	    }
            
	    virtual ~Actuator(void)
	    {
	    }
            
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
            
	    std::string                 name;
	    Data                        data;
	    std::map<std::string, bool> isSet;
	};
	
	/** This class defines transmission instances */
	struct Transmission
	{
	    Transmission(void)
	    {
		isSet["name"] = false;
	    }
            
	    virtual ~Transmission(void)
	    {
	    }
            
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
            
	    std::string                 name;
	    Data                        data;
	    std::map<std::string, bool> isSet;
	};

	
	/** This class defines link instances */
	struct Link
	{
	    
	    /** Class for link geometry instances */
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
                
		virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
                
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

	    /** Class for link joint instances (connects a link to its parent) */
	    struct Joint
	    {
		Joint(void)
		{
		    axis[0] = axis[1] = axis[2] = 0.0;
		    anchor[0] = anchor[1] = anchor[2] = 0.0;
		    limit[0] = limit[1] = 0.0;
		    velocityLimit = 0.0;
		    effortLimit = 0.0;
		    type = UNKNOWN;
		    isSet["name"] = false;
		    isSet["type"] = false;
		    isSet["axis"] = false;
		    isSet["anchor"] = false;
		    isSet["effortLimit"] = false;
		    isSet["velocityLimit"] = false;
		    isSet["calibration"] = false;
		}
		
		virtual ~Joint(void)
		{
		}
                
		virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
                
		enum
		    {
			UNKNOWN, FIXED, REVOLUTE, PRISMATIC, PLANAR, FLOATING
		    }                       type;
		std::string                 name;	
		double                      axis[3];       // vector describing the axis of rotation: (x,y,z)
		double                      anchor[3];     // point about which the axis defines the rotation: (x,y,z)
		double                      limit[2];      // the joint limits: (min, max)
		double                      effortLimit;
		double                      velocityLimit;
		std::string                 calibration;
		Data                        data;
		std::map<std::string, bool> isSet;
	    };
	    
	    /** Class for link collision component instances */
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
                
		virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
                
		std::string                 name;
		bool                        verbose;
		double                      xyz[3];
		double                      rpy[3];
		std::string                 material;
		Geometry                   *geometry;
		Data                        data;    
		std::map<std::string, bool> isSet;
	    };
	    
	    /** Class for link inertial component instances */
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
                
		virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
                
		std::string                 name;
		double                      mass;
		double                      inertia[6];
		double                      com[3];
		Data                        data;
		std::map<std::string, bool> isSet;
	    };
	    
	    /** Class for link visual component instances */
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
                
		virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
                
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
            
	    /** Check if the link instance is inside some group */
	    bool insideGroup(Group *group) const;

	    /** Check if the link instance is inside some group */
	    bool insideGroup(const std::string &group) const;
	    
	    /** Check if the link instance is also a sensor */
	    virtual bool canSense(void) const;
	    
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
            
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
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
            
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
	    
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;

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
	    
	    /** Check if this group has a specific flag */
	    bool hasFlag(const std::string &flag) const;
	    
	    /** Check if a specific link is a root in this group */
	    bool isRoot(const Link* link) const;
	    
	    std::string              name;
	    std::vector<std::string> flags;
	    std::vector<std::string> linkNames;
	    std::vector<Link*>       links;
	    std::vector<Link*>       linkRoots;
	    std::vector<std::string> frameNames;
	    std::vector<Frame*>      frames;
	};
	
	/** Constructor. If a filename if specified as argument, that file is parsed. */
	explicit
	URDF(const char *filename = NULL)
	{   
	    m_paths.push_back("");
	    if (filename)
		loadFile(filename);
	}
        
	/** Destructor. Frees all memory */
	virtual ~URDF(void)
	{
	    freeMemory();
	}
        
	/** Clear all datastructures. The parser can be used for a new document */
	virtual void clear(void);

	/** Parse the content of the specified file */
	virtual bool loadFile(const char *filename);

	/** Parse the content of the specified file */
	virtual bool loadFile(FILE *file);

	/** Parse the content of the specified buffer */
	virtual bool loadString(const char *data);

	/** Parse the content of the specified stream */
	virtual bool loadStream(std::istream &is);

	/** Print the parsed datastructure */
	virtual void print(std::ostream &out = std::cout) const;
        
	/** Check if the links form a cycle */
	bool containsCycle(unsigned int index) const;
	
	/** Simple checks to make sure the parsed values are correct */
	void sanityCheck(void) const;
	
	/** Returns the robot name */
	const std::string& getRobotName(void) const;
	
	/** Returns the number of individual robot parts that connect to the environment */
	unsigned int getDisjointPartCount(void) const;
	
	/** Return a robot part that connects to the environment */
	Link* getDisjointPart(unsigned int index) const;
	
	/** Check if a specific link connects to the environment */
	bool isRoot(const Link* link) const;
	
	/** Retrieve a link by its name */
	Link* getLink(const std::string &name) const;

	/** Get the list of all links. The array is sorted alphabetically by name. */
	void getLinks(std::vector<Link*> &links) const;
	
	/** Retrieve an actuator by its name */
	Actuator* getActuator(const std::string &name) const;

	/** Get the list of all actuators. The array is sorted alphabetically by name. */
	void getActuators(std::vector<Actuator*> &actuators) const;

	/** Retrieve a transmission by its name */
	Transmission* getTransmission(const std::string &name) const;	
	
	/** Get the list of all transmissions. The array is sorted alphabetically by name. */	
	void getTransmissions(std::vector<Transmission*> &transmissions) const;

	/** Retrieve a frame by its name */
	Frame* getFrame(const std::string &name) const;	

	/** Get the list of all frames. The array is sorted alphabetically by name. */
        void getFrames(std::vector<Frame*> &frames) const;
	
	/** Get the list of all group names. The array is sorted alphabetically. */
	void getGroupNames(std::vector<std::string> &groups) const;

	/** Retrieve a group by its name */
	Group* getGroup(const std::string &name) const;

	/** Get the list of all groups. The array is sorted alphabetically by name. */
	void getGroups(std::vector<Group*> &groups) const;
	
	/** Get the data that was defined at top level */
	const Data& getData(void) const;
        
    protected:
        
	/** Free the memory allocated in this class */
	void freeMemory(void);
	
	/** Clear temporary datastructures used in parsing */
	void clearTemporaryData(void);

	/** Clear the memory used by the XML parser*/
	void clearDocs(void);

	
	
	/** Add the path to a given filename as a known path */
	virtual void  addPath(const char *filename);

	/** Find a file using the set of known paths */
	virtual char* findFile(const char *filename);

	

	/** Add default constants like M_PI */
	virtual void defaultConstants(void);
        
	/** Parse the URDF document */
	virtual bool parse(const TiXmlNode *node);

	/** Parse the <link> tag */
	void loadLink(const TiXmlNode *node);
	/** Parse the <sensor> tag */
	void loadSensor(const TiXmlNode *node);
	/** Parse the <frame> tag */
	void loadFrame(const TiXmlNode *node);
	/** Parse the <actuator> tag */
	void loadActuator(const TiXmlNode *node);
	/** Parse the <transmission> tag */
	void loadTransmission(const TiXmlNode *node);
	/** Parse the <joint> tag */
	void loadJoint(const TiXmlNode *node, const std::string &defaultName, Link::Joint *joint);
	/** Parse the <geometry> tag */
	void loadGeometry(const TiXmlNode *node, const std::string &defaultName, Link::Geometry *geometry);
	/** Parse the <collision> tag */
	void loadCollision(const TiXmlNode *node, const std::string &defaultName, Link::Collision *collision);
	/** Parse the <visual> tag */
	void loadVisual(const TiXmlNode *node, const std::string &defaultName, Link::Visual *visual);
	/** Parse the <inertial> tag */
	void loadInertial(const TiXmlNode *node, const std::string &defaultName, Link::Inertial *inertial);
	/** Parse the <data> tag */
	void loadData(const TiXmlNode *node, Data *data);
	
	/** Parse a list of string expressions and convert them to doubles (this relies on the loaded constants) */
	unsigned int loadDoubleValues(const TiXmlNode *node, unsigned int count, double *vals);
	/** Parse a list of string expressions and convert them to doubles (this relies on the loaded constants) */
	unsigned int loadDoubleValues(const std::string& data, unsigned int count, double *vals, const TiXmlNode *node = NULL);
	/** Parse a list of strings and convert them to booleans */
	unsigned int loadBoolValues  (const TiXmlNode *node, unsigned int count, bool *vals);
	/** Parse a list of string expressions and convert them to doubles (this relies on the loaded constants) */
	unsigned int loadBoolValues(const std::string& data, unsigned int count, bool *vals, const TiXmlNode *node = NULL);
	

	/** Depending on whether the node is a clone or not, get the proper list of children and attibutes for this tag */
	void getChildrenAndAttributes(const TiXmlNode* node, std::vector<const TiXmlNode*> &children, std::vector<const TiXmlAttribute*> &attributes) const;	

	/** Extract and return the value of the name attribute from a list of attributes */
	std::string extractName(std::vector<const TiXmlAttribute*> &attributes, const std::string &defaultName) const;
	

	/** Output a message letting the user know that a specific node was ignored */
	virtual void ignoreNode(const TiXmlNode* node);
	
	/** Replace include declarations with the indicated file */
	void replaceIncludes(TiXmlElement *elem);	

	/** Print the error location, if known */
	void errorLocation(const TiXmlNode* node = NULL) const;
	
	/** Compute the easy-access pointers inside the parsed datastructures */
	void linkDatastructure(void);

	/** The name of the file where the parsing started */
	std::string                          m_source;
	
	/** The list of paths the parser knows about when it sees an <include> directive */
	std::vector<std::string>             m_paths;
        
	/** The robot name */
	std::string                          m_name;

	/** Contains the list of parsed links and sensors */
	std::map<std::string, Link*>         m_links;

	/** Contains the list of parsed groups */
	std::map<std::string, Group*>        m_groups;

	/** Contains the list of parsed actuators */
	std::map<std::string, Actuator*>     m_actuators;

	/** Contains the list of parsed transmissions */
	std::map<std::string, Transmission*> m_transmissions;

	/** Contains the list of parsed frames */
	std::map<std::string, Frame*>        m_frames;
	
	/** Contains information specified in <data> tags at the top level */
	Data                                 m_data;
        
	/** Additional datastructure containing a list links that are connected to the environment */
	std::vector<Link*>                   m_linkRoots; 
	
    private:
        
	/** Temporary datastructure for keeping track link collision components */
	std::map<std::string, Link::Collision*> m_collision;
	/** Temporary datastructure for keeping track link joints */
	std::map<std::string, Link::Joint*>     m_joints;
	/** Temporary datastructure for keeping track link inertial components */
	std::map<std::string, Link::Inertial*>  m_inertial;
	/** Temporary datastructure for keeping track link visual components */
	std::map<std::string, Link::Visual*>    m_visual;
	/** Temporary datastructure for keeping track link geometry */
	std::map<std::string, Link::Geometry*>  m_geoms;
	
	
	/* temporary storage for information during parsing; should not be used elsewhere */
	
	/** The list of constants (temporary) */
	std::map<std::string, std::string>      m_constants; 
	/** The list of templates (temporary) */
	std::map<std::string, const TiXmlNode*> m_templates;
	/** List of nodes to be processed after loading the constants and templates (temporary) */
	std::vector<const TiXmlNode*>           m_stage2;
	/** List of loaded documents. The documents are not cleared after parse since <data> tags may contain pointers to XML datastructures from these documents */
	std::vector<TiXmlDocument*>             m_docs;
	
	/** Approximate location in parsed file, if known (used for error messages) */
	std::string                             m_location;
    };
    
} // namespace robot_desc

#endif

