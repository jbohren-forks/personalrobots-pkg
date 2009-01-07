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

#ifndef KINEMATIC_ROBOT_MODEL_
#define KINEMATIC_ROBOT_MODEL_

#include <urdf/URDF.h>
#include <LinearMath/btTransform.h>

#include <iostream>
#include <vector>
#include <string>
#include <cassert>

/** @htmlinclude ../../manifest.html

    @mainpage
    
    A class describing a kinematic robot model loaded from URDF */

/** Main namespace */
namespace planning_models
{
    
    /** Definition of a kinematic model */
    class KinematicModel
    {
    public:
	
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
	    
	    /** x, y, z */
	    double size[3]; 
	};
	
	/** Forward definition of a joint */
	class Joint;
	
	/** Forward definition of a link */
	class Link;
	
	/** Forward definition of a robot */
	class Robot;
	
	/** A joint from the robot. Contains the transform applied by the joint type */
	class Joint
	{
	    friend class KinematicModel;	    
	public:
	    Joint(void)
	    {
		usedParams = 0;
		varTrans.setIdentity();
		before = after = NULL;
		owner = NULL;
	    }
	    
	    virtual ~Joint(void)
	    {
		if (after)
		    delete after;
	    }


	    /** Name of the joint */
	    std::string       name;
	    
	    /** The model that owns this joint */
	    KinematicModel   *owner;

	    /** the links that this joint connects */	    
	    Link             *before;
	    Link             *after;
	    
	    /** The range of indices in the parameter vector that
		needed to access information about the position of this
		joint */
	    unsigned int      usedParams;
	    
	    /** Bitvector identifying which groups this joint is part of */
	    std::vector<bool> inGroup;
	    
	    /** the local transform (computed by forward kinematics) */
	    btTransform       varTrans;

	    /* compute the parameter names from this joint */
	    unsigned int computeParameterNames(unsigned int pos);

	    /** Update varTrans if this joint is part of the group indicated by groupID
	     *  and recompute globalTrans using varTrans */
	    const double* computeTransform(const double *params, int groupID = -1);
	    
	    /** Update the value of VarTrans using the information from params */
	    virtual void updateVariableTransform(const double *params) = 0;

	    /** Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot) = 0;
	};

	/** A fixed joint */
	class FixedJoint : public Joint
	{
	public:
	    
	    /** Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	};

	/** A planar joint */
	class PlanarJoint : public Joint
	{
	public:
	    
	    PlanarJoint(void) : Joint()
	    {
		usedParams = 3;
	    }
	    
	    /** Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	};

	/** A floating joint */
	class FloatingJoint : public Joint
	{
	public:
	    
	    FloatingJoint(void) : Joint()
	    {
		usedParams = 7;
	    }

	    /** Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	};

	/** A prismatic joint */
	class PrismaticJoint : public Joint
	{
	public:
	    
	    PrismaticJoint(void) : Joint(), axis(0.0, 0.0, 0.0)
	    {
		limit[0] = limit[1] = 0.0;
		usedParams = 1;
	    }
	    
	    /** Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	    
	    btVector3 axis;
	    double    limit[2];
	};
	
	/** A revolute joint */
	class RevoluteJoint : public Joint
	{
	public:
	    
	    RevoluteJoint(void) : Joint(), axis(0.0, 0.0, 0.0), anchor(0.0, 0.0, 0.0)
	    {
		limit[0] = limit[1] = 0.0;
		usedParams = 1;
	    }
	    
	    /** Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	    
	    btVector3 axis;
	    btVector3 anchor;
	    double    limit[2];
	};
	
	
	/** A link from the robot. Contains the constant transform applied to the link and its geometry */
	class Link
	{
	    friend class KinematicModel;
	public:

	    Link(void)
	    {
		before = NULL;
		shape  = NULL;
		owner  = NULL;
		constTrans.setIdentity();
		constGeomTrans.setIdentity();
		globalTransFwd.setIdentity();
		globalTrans.setIdentity();		
	    }
	    
	    virtual ~Link(void)
	    {
		if (shape)
		    delete shape;
		for (unsigned int i = 0 ; i < after.size() ; ++i)
		    delete after[i];
	    }

	    /** Name of the link */
	    std::string         name;

	    /** The model that owns this link */
	    KinematicModel     *owner;

	    /** Joint that connects this link to the parent link */
	    Joint              *before;
	    
	    /** List of descending joints (each connects to a child link) */
	    std::vector<Joint*> after;
	    
	    /** The constant transform applied to the link (local) */
	    btTransform         constTrans;
	    
	    /** The constant transform applied to the collision geometry of the link (local) */
	    btTransform         constGeomTrans;
	    
	    /** The geometry of the link */
	    Shape              *shape;
	    
	    /* ----------------- Computed data -------------------*/
	    
	    /** The global transform this link forwards (computed by forward kinematics) */
	    btTransform         globalTransFwd;

	    /** The global transform for this link (computed by forward kinematics) */
	    btTransform         globalTrans;

	    /* compute the parameter names from this link */
	    unsigned int computeParameterNames(unsigned int pos);

	    /** recompute globalTrans */
	    const double* computeTransform(const double *params, int groupID = -1);

	    /** Extract the information needed by the joint given the URDF description */
	    void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	    
	};
	
	/** A robot structure */
	class Robot
	{
	public:

	    Robot(KinematicModel *model)
	    {
		owner = model;	  
		chain = NULL;
		stateDimension = 0;
	    }
	    
	    virtual ~Robot(void)
	    {
		if (chain)
		    delete chain;
	    }
	    
	    void computeTransforms(const double *params, int groupID = -1);
	    
	    /** The model that owns this robot */
	    KinematicModel     *owner;

	    /** List of links in the robot */
	    std::vector<Link*>  links;

	    /** List of joints in the robot */
	    std::vector<Joint*> joints;
	    
	    /** The first joint in the robot -- the root */
	    Joint              *chain;	    

	    /** Number of parameters needed to define the joints */
	    unsigned int        stateDimension;
	    
	    /** The bounding box for the set of parameters describing the
	     *  joints.  This array contains 2 * stateDimension elements:
	     *  positions 2*i and 2*i+1 define the minimum and maximum
	     *  values for the parameter. If both minimum and maximum are
	     *  set to 0, the parameter is unbounded. */
	    std::vector<double> stateBounds;
	    
	    /** The list of index values where floating joints
		start. These joints need special attention in motion
		planning, so the indices are provided here for
		convenience. */
	    std::vector<int>    floatingJoints;

	    /** The list of index values where planar joints
		start. These joints need special attention in motion
		planning, so the indices are provided here for
		convenience. */
	    std::vector<int>    planarJoints;
	    
	    /* For each group, we have a list of index values in the
	     * computed state information that describe the components of
	     * the state space the group corresponds to */
	    std::vector< std::vector<unsigned int> > groupStateIndexList;

	    /* The roots of every group within this robot */
	    std::vector<std::vector<Joint*> >        groupChainStart;
	    
	};
	
	/** A class that can hold the named parameters of this planning model */
	class StateParams
	{
	public:

	    StateParams(KinematicModel *model)
	    {
		assert(model->isBuilt());
		m_owner = model;
		m_dim = m_owner->stateDimension > 0 ? m_owner->stateDimension : 0;
		m_params = m_dim > 0 ? new double[m_dim] : NULL;
		m_pos = m_owner->parameterNames;
	    }
	    
	    virtual ~StateParams(void)
	    {
		if (m_params)
		    delete[] m_params;
	    }
	    
	    /** Given the name of a joint, set the values of the parameters describing the joint */
	    void setParams(const double *params, const std::string &name);
	    
	    /** Set all the parameters to a given value */
	    void setAll(const double value);
	    
	    /** Set the parameters for a given group */
	    void setParams(const double *params, int groupID = -1);
	    
	    /** Return the current value of the params */
	    const double* getParams(void) const;
	    
	    /** Get the offset for the parameter of a joint in a given group */
	    int getPos(const std::string &name, int groupID = -1) const;

	    /** Copy the parameters for a given group to a destination address */
	    void copyParams(double *params, int groupID = -1) const;
	    
	    /** Copy the parameters describen a given joint */
	    void copyParams(double *params, const std::string &name) const;
	    
	    /** Print the data from the state to screen */
	    void print(std::ostream &out = std::cout);
	    
	protected:
	    
	    KinematicModel                      *m_owner;
	    unsigned int                         m_dim;
	    double                              *m_params;
	    std::map<std::string, unsigned int>  m_pos;
	};
	
	
	KinematicModel(void)
	{
	    rootTransform.setIdentity();
	    stateDimension = 0;
	    m_ignoreSensors = false;
	    m_verbose = false;	    
	    m_built = false;
	}
	
	virtual ~KinematicModel(void)
	{
	    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
		delete m_robots[i];
	}
	
	virtual void build(const robot_desc::URDF &model, bool ignoreSensors = false);
	bool         isBuilt(void) const;
	StateParams* newStateParams(void);
	
	void         setVerbose(bool verbose);	
	

	unsigned int getRobotCount(void) const;
	Robot*       getRobot(unsigned int index) const;

	void         getGroups(std::vector<std::string> &groups) const;
	int          getGroupID(const std::string &group) const;

	/** Return the group name as it was in the original URDF document */
	std::string  getURDFGroup(const std::string &group) const;
	
	Link*        getLink(const std::string &link) const;
	void         getLinks(std::vector<Link*> &links) const;

	Joint*       getJoint(const std::string &joint) const;
	void         getJoints(std::vector<Joint*> &joints) const;

	/** Get the names of the joints in a specific group */
	void         getJointsInGroup(std::vector<std::string> &names, int groupID = -1) const;

	void defaultState(void);
	void computeTransforms(const double *params, int groupID = -1);

	void printModelInfo(std::ostream &out = std::cout) const;
	void printLinkPoses(std::ostream &out = std::cout) const;
	
	/** The name of the model */
	std::string         name;
	    
	/** A transform that is applied to the entire model */
	btTransform         rootTransform;
	
	/** Cumulative list of floating joints */
	std::vector<int>    floatingJoints;

	/** Cumulative list of planar joints */
	std::vector<int>    planarJoints;
	
	/** Cumulative state dimension */
	unsigned int        stateDimension;
	
	/** Cumulative state bounds */
	std::vector<double> stateBounds;
	
	/** A map defining the parameter names in the complete state */
	std::map<std::string, unsigned int>      parameterNames;
	
	/** Cumulative index list */
	std::vector< std::vector<unsigned int> > groupStateIndexList;

	/** Cumulative list of group roots */
	std::vector< std::vector<Joint*> >       groupChainStart;

    protected:
	
	std::vector<Robot*>               m_robots;
	std::vector<std::string>          m_groups;
	std::map<std::string, int>        m_groupsMap;	       
	std::map<std::string, Link*>      m_linkMap;	
	std::map<std::string, Joint*>     m_jointMap;
	bool                              m_ignoreSensors;
	bool                              m_verbose;    
	bool                              m_built;
	
    private:
	
	/** Build the needed datastructure for a joint */
	void buildChainJ(Robot *robot, Link  *parent, Joint *joint, const robot_desc::URDF::Link *urdfLink, const robot_desc::URDF &model);

	/** Build the needed datastructure for a link */
	void buildChainL(Robot *robot, Joint *parent, Link  *link,  const robot_desc::URDF::Link *urdfLink, const robot_desc::URDF &model);

	/** Construct the list of groups the model knows about (the ones marked with the 'plan' attribute) */
	void constructGroupList(const robot_desc::URDF &model);
	
	/* compute the parameter names  */
	void computeParameterNames(void);

	/** Allocate a joint of appropriate type, depending on the loaded link */
	Joint* createJoint(const robot_desc::URDF::Link* urdfLink);
	
    };

}

#endif
