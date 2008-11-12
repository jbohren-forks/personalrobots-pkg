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

#ifndef URDF_PARSER_
#define URDF_PARSER_

#include <tinyxml/tinyxml.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>

/** 
    @mainpage 
    
    @htmlinclude ../../manifest.html
    
    Universal Robot Description Format (URDF) parser. The URDF class
    is able to load a robot description from multiple files, string
    buffers, streams and produce a datastructure similar to the XML
    structure. Expression evaluation and file inclusion are performed
    if needed. Checks are performed to make sure the document is
    correct: values that are inconsistend are reported, typos in the
    names, unknown tags, etc. For more documentation, see the URDF
    description. */

/** Namespace that contains the URDF parser */
namespace robot_desc
{
    /** This class contains a complete parser for URDF documents.
	Datastructures that resemble the ones specified in the XML
	document are instantiated, easy-access pointers are computed
	(to access a datastructure from another). Checks are made to
	make sure the parsed data is correct.  Since actuators,
	transmissions and controllers are potentially robot specific,
	their data is kept as XML elements.	
     */
    class URDF
    {
    public:
        
	/** This class encapsulates data that can be attached to various tags in the format */
	class Map
	{
	public:
            
	    Map(void)
	    {
	    }
            
	    virtual ~Map(void)
	    {
	    }
            
	    /** Get the set of flags used for the processed <map> tags. Each <map> tag has one flag. Multiple tags may
		have the same flag. */
	    void getMapTagFlags(std::vector<std::string> &flags) const;
	    
	    /** Given a specific flag, retrieve the names of the <map> tags that have the given flag */
	    void getMapTagNames(const std::string &flag, std::vector<std::string> &names) const;
	    
	    /** Given a name and a flag, retrieve the defined map (string, string) */
	    std::map<std::string, std::string> getMapTagValues(const std::string &flag, const std::string &name) const;

	    /** Given a name and a flag, retrieve the defined map (string, XML) */
	    std::map<std::string, const TiXmlElement*> getMapTagXML(const std::string &flag, const std::string &name) const;
	    
	    /** Check whether a key under empty flag and empty name has been defined */
	    bool hasDefault(const std::string &key) const;

	    /** Retrieve the value of a key (string) under empty flag and empty name has been defined */
	    std::string getDefaultValue(const std::string &key) const;
	    
	    /** Retrieve the value of a key (XML) under empty flag and empty name has been defined */
	    const TiXmlElement* getDefaultXML(const std::string &key) const;
	    
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
            
	    /** Add a string element to the map */
	    void add(const std::string &flag, const std::string &name, const std::string &key, const std::string &value);
	    
	    /** Add an XML element to the map */
	    void add(const std::string &flag, const std::string &name, const std::string &key, const TiXmlElement *value);
            
	    Map& operator=(const Map &rhs)
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
        	
	/** This class defines link instances. It contains instances of collision, visual, inertial and joint descriptions. */
	struct Link
	{
	    
	    /** Class for link geometry instances */
	    struct Geometry
	    {
		struct Shape
		{
		    virtual ~Shape(void)
		    {
		    }		    
		};
		
		struct Sphere : public Shape
		{
		    Sphere(void) : Shape()
		    {
			radius = 0.0;
		    }
		    
		    double radius;
		};
		
		struct Box : public Shape
		{
		    Box(void) : Shape()
		    {	
			size[0] = size[1] = size[2] = 0.0;
		    }
		    
		    double size[3];
		};
		
		struct Cylinder : public Shape
		{
		    Cylinder(void) : Shape()
		    {	
			length = radius = 0.0;
		    }
		    
		    double length, radius;
		};		

		struct Mesh : public Shape
		{
		    Mesh(void) : Shape()
		    {
			scale[0] = scale[1] = scale[2] = 1.0;
		    }
		    
		    std::string filename;
		    double      scale[3];
		};		

		Geometry(void)
		{
		    type = UNKNOWN;
		    shape = NULL;
		    isSet["name"] = false;
		    isSet["size"] = false;
		    isSet["length"] = false;
		    isSet["radius"] = false;
		    isSet["filename"] = false;
		    isSet["scale"] = false;
		}
                
		virtual ~Geometry(void)
		{
		    if (shape)
			delete shape;
		}
                
		virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
                
		enum
		    {
			UNKNOWN, SPHERE, BOX, CYLINDER, MESH
		    }                       type;
		std::string                 name;
		Shape                      *shape;
		Map                         data;
		std::map<std::string, bool> isSet;
	    };
	    

	    /** Class for link joint instances (connects a link to its parent) */
	    struct Joint
	    {
		Joint(void)
		{
		    axis[0] = axis[1] = axis[2] = 0.0;
		    anchor[0] = anchor[1] = anchor[2] = 0.0;
		    damping = 0.0;
		    friction = 0.0;
		    limit[0] = limit[1] = 0.0;
		    safetyLength[0] = safetyLength[1] = 0.0;
		    velocityLimit = 0.0;
		    effortLimit = 0.0;
		    type = UNKNOWN;
		    isSet["name"] = false;
		    isSet["type"] = false;
		    isSet["axis"] = false;
		    isSet["anchor"] = false;
		    isSet["damping"] = false;
		    isSet["friction"] = false;
		    isSet["limit"] = false;
		    isSet["safetyLengthMin"] = false;
		    isSet["safetyLengthMax"] = false;
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
		double                      axis[3];         // vector describing the axis of rotation: (x,y,z)
		double                      anchor[3];       // point about which the axis defines the rotation: (x,y,z)
		double                      damping;         // damping coefficient
		double                      friction;        // friction coefficient
		double                      limit[2];        // the joint limits: (min, max)
		double                      safetyLength[2]; // the joint limits: (min, max)
		double                      effortLimit;
		double                      velocityLimit;
		std::string                 calibration;
		Map                         data;
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
		    geometry = NULL;
		    isSet["name"] = false;
		    isSet["verbose"] = false;
		    isSet["xyz"] = false;
		    isSet["rpy"] = false;
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
		Geometry                   *geometry;
		Map                         data;    
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
		/** Ixx Ixy Ixz Iyy Iyz Izz */
		double                      inertia[6]; 
		double                      com[3];
		Map                         data;
		std::map<std::string, bool> isSet;
	    };
	    
	    /** Class for link visual component instances */
	    struct Visual
	    {
		Visual(void)
		{
		    xyz[0] = xyz[1] = xyz[2] = 0.0;
		    rpy[0] = rpy[1] = rpy[2] = 0.0;
		    geometry = NULL;
		    isSet["name"] = false;
		    isSet["xyz"] = false;
		    isSet["rpy"] = false;
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
		Geometry                   *geometry;
		Map                         data;
		std::map<std::string, bool> isSet;
	    };
            
	    Link(void)
	    {
		parent = NULL;
		xyz[0] = xyz[1] = xyz[2] = 0.0;
		rpy[0] = rpy[1] = rpy[2] = 0.0;
		inertial  = NULL;
		visual    = NULL;
		collision = NULL;
		joint     = NULL;
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
	    Map                         data;
	    
	    std::vector<Group*>         groups;
	    std::vector<bool>           inGroup;

	    std::map<std::string, bool> isSet;
	};
	
	/** Class for defining sensors. This is basically a link with a few extra parameters */
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
	
	/** Class for defining frames. A frame is attached to a link,
	    as its parent or as its child. Frames do not have
	    geometric or inertial properties. They are used to define
	    coordinate frames. */
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
	    Map                         data;

	    std::vector<Group*>         groups;
	    std::map<std::string, bool> isSet;
	};
	
	/** A class that represents a link chain */
	struct Chain
	{
	    Chain(void)
	    {
	    }
	    
	    virtual ~Chain(void)
	    {
	    }
	    
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;
	    
	    std::string        name;
	    std::string        root;
	    std::string        tip;
	    std::vector<Link*> links;
	};
		
	/** A class that represents groups of links or frames (or both) */
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
	    	
	    /** Check if the group is empty */
	    bool empty(void) const;
    
	    virtual void print(std::ostream &out = std::cout, std::string indent = "") const;

	    std::string              name;
	    std::vector<std::string> flags;
	    std::vector<std::string> linkNames;
	    std::vector<Link*>       links;
	    std::vector<Link*>       linkRoots;
	    std::vector<std::string> frameNames;
	    std::vector<Frame*>      frames;
	    Map                      data;
	};
	
	/** Constructor. If a filename if specified as argument, that file is parsed. */
	URDF(const char *filename = NULL)
	{   
	    m_paths.push_back("");
	    m_errorCount = 0;
	    m_rememberUnknownTags = false;
	    m_verbose = true;
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
        
	/** Set the verbosity. Default is true */
	void setVerbose(bool verbose);

	/** If set to true, unknown tags are stored in a list instead
	    of presenting error messages. Default is false. */
	void rememberUnknownTags(bool remember);
	
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

	/** Retrieve a link by its joint name (connecting to the link's parent) */
	Link* getJointLink(const std::string &name) const;

	/** Get the list of all links. The array is sorted alphabetically by name. */
	void getLinks(std::vector<Link*> &links) const;
	
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

	/** Retrieve a chain by its name */
	Chain* getChain(const std::string &name) const;
	
	/** Get the list of all chains. The array is sorted alphabetically by name. */
	void getChains(std::vector<Chain*> &chains) const;

	/** Get the data that was defined at top level */
	const Map& getMap(void) const;
	
	/** Get the defined resource path for this document */
	const std::string& getResourceLocation(void) const;
	
	/** Try to evaluate the constant as a double value */
	double getConstantValue(const std::string &name, bool *error = NULL) const;

	/** Try to retrieve the constant as a string value */
	std::string getConstantString(const std::string &name, bool *error = NULL) const;
	
	/** Get the list of actuator tags */
	void getActuators(std::vector<const TiXmlElement*> &actuators) const;

	/** Get the list of transmission tags */
	void getTransmissions(std::vector<const TiXmlElement*> &transmissions) const;

	/** Get the list of controller tags */
	void getControllers(std::vector<const TiXmlElement*> &controllers) const;
	
	/** Get the list of unknown tags. Only available if
	    rememberUnknownTags(true) was called before parsing. */
	void getUnknownTags(std::vector<const TiXmlNode*> &unknownTags) const;
	
	/** Check if the links form a cycle */
	bool containsCycle(unsigned int index) const;
	
	/** Simple checks to make sure the parsed values are correct */
	void sanityCheck(void) const;
	
	/** Return the number of encountered errors */
	unsigned int getErrorCount(void) const;
	
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
	void loadMap(const TiXmlNode *node, Map *data);
	
	/** Parse a list of string expressions and convert them to doubles (this relies on the loaded constants) */
	unsigned int loadDoubleValues(const TiXmlNode *node, unsigned int count, double *vals, const char *attrName = NULL, bool warn = false);
	/** Parse a list of string expressions and convert them to doubles (this relies on the loaded constants) */
	unsigned int loadDoubleValues(const std::string& data, unsigned int count, double *vals, const TiXmlNode *node = NULL);
	/** Parse a list of strings and convert them to booleans */
	unsigned int loadBoolValues  (const TiXmlNode *node, unsigned int count, bool *vals, const char *attrName = NULL, bool warn = false);
	/** Parse a list of string expressions and convert them to doubles (this relies on the loaded constants) */
	unsigned int loadBoolValues(const std::string& data, unsigned int count, bool *vals, const TiXmlNode *node = NULL);
	

	/** Depending on whether the node is a clone or not, get the proper list of children and attibutes for this tag */
	void getChildrenAndAttributes(const TiXmlNode* node, std::vector<const TiXmlNode*> &children, std::vector<const TiXmlAttribute*> &attributes) const;	

	/** Extract and return the value of the name attribute from a list of attributes */
	std::string extractName(std::vector<const TiXmlAttribute*> &attributes, const std::string &defaultName) const;
	

	/** Behaviour for unknown nodes. Default is to output a message letting the user know that a specific node is unknown */
	virtual void unknownNode(const TiXmlNode* node);
	
	/** Replace include declarations with the indicated file */
	bool replaceIncludes(TiXmlElement *elem);	

	/** Print the error location, if known */
	void errorLocation(const TiXmlNode* node = NULL) const;
	
	/** Output an error message */
	void errorMessage(const std::string& msg) const;

	/** Compute the easy-access pointers inside the parsed datastructures */
	virtual void linkDatastructure(void);

	/** The name of the file where the parsing started */
	std::string                             m_source;

	/** A string that defines the base path for resources pointed to from the loaded document */
	std::string                             m_resourceLocation;
		
	/** The list of paths the parser knows about when it sees an <include> directive */
	std::vector<std::string>                m_paths;
        
	/** The robot name */
	std::string                             m_name;

	/** Contains the list of parsed links and sensors */
	std::map<std::string, Link*>            m_links;

	/** Contains the list of parsed groups */
	std::map<std::string, Group*>           m_groups;

	/** Contains the list of parsed frames */
	std::map<std::string, Frame*>           m_frames;

	/** The list of parsed chains */
	std::map<std::string, Chain*>           m_chains;
	
	/** Contains information specified in <data> tags at the top level */
	Map                                     m_data;
        
	/** Additional datastructure containing a list links that are connected to the environment */
	std::vector<Link*>                      m_linkRoots; 
	
	/** The list of constants */
	std::map<std::string, std::string>      m_constants; 
	
	/** The list of constant blocks */
	std::map<std::string, const TiXmlNode*> m_constBlocks;

	/** The list of actuator tags */
	std::vector<const TiXmlElement*>        m_actuators;

	/** The list of controller tags */
	std::vector<const TiXmlElement*>        m_controllers;

	/** The list of transmission tags */
	std::vector<const TiXmlElement*>        m_transmissions;

	/** If this is set to true, unknown tags will be simply added
	    to a list, no error messages related to unknown tags will
	    be presented. */
	bool                                    m_rememberUnknownTags;	

	/** The list of unknown tags (maintained only if rememberUnknownTags(true) was previosuly called) */
	std::vector<const TiXmlNode*>           m_unknownTags;

	/** Verbosity flag */
	bool                                    m_verbose;
	
	/** Counter for errors */
	mutable unsigned int                    m_errorCount;
	
    private:
	
	/* temporary storage for information during parsing; should not be used elsewhere */

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
	
	
	/** List of nodes to be processed after loading the constants and templates (temporary) */
	std::vector<const TiXmlNode*>           m_stage2;
	/** List of loaded documents. The documents are not cleared after parse since <data> tags may contain pointers to XML datastructures from these documents */
	std::vector<TiXmlDocument*>             m_docs;
	
	/** Approximate location in parsed file, if known (used for error messages) */
	std::string                             m_location;
    };
    
} // namespace robot_desc

#endif

