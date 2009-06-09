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

#ifndef PLANNING_MODELS_KINEMATIC_ROBOT_MODEL_
#define PLANNING_MODELS_KINEMATIC_ROBOT_MODEL_

#include "planning_models/shapes.h"
#include "planning_models/output.h"
#include <urdf/URDF.h>
#include <LinearMath/btTransform.h>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cassert>

/** Describing a kinematic robot model loaded from URDF. Visual geometry is ignored */

/** Main namespace */
namespace planning_models
{
    
    /** Definition of a kinematic model. This class is not thread
	safe, however multiple instances can be created */
    class KinematicModel
    {
    public:
	
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
	    
	    /** The robot that owns this joint */
	    Robot            *owner;

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

	    /* Compute the parameter names from this joint; the
	       purpose is to create a map that relates the name of a
	       joint in the loaded description to the position in a
	       vector of parameters */
	    unsigned int computeParameterNames(unsigned int pos);

	    /** Update varTrans if this joint is part of the group indicated by groupID
	     *  and recompute globalTrans using varTrans */
	    const double* computeTransform(const double *params, int groupID);
	    
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
	        usedParams = 3; // (x, y, theta)
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
	        usedParams = 7; // vector: (x, y, z)  quaternion: (x, y, z, w)
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
		continuous = false;
		usedParams = 1;
	    }
	    
	    /** Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	    
	    btVector3 axis;
	    btVector3 anchor;
	    double    limit[2];
	    bool      continuous;
	};
	
	/** Class defining bodies that can be attached to robot
	    links. This is useful when handling objects picked up by
	    the robot. */
	class AttachedBody
	{
	public:
	    
	    AttachedBody(void)
	    {
		attachTrans.setIdentity();
		shape = NULL;
	    }
	    
	    ~AttachedBody(void)
	    {
		if (shape)
		    delete shape;
	    }
	    
	    /** The constant transform applied to the link (needs to be specified by user) */
	    btTransform         attachTrans;

	    /** The geometry of the attached body */
	    shapes::Shape*      shape;

	    /** The global transform for this link (computed by forward kinematics) */
	    btTransform         globalTrans;
	    
	    /** recompute globalTrans */
	    void computeTransform(btTransform &parentTrans);
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
		for (unsigned int i = 0 ; i < attachedBodies.size() ; ++i)
		    delete attachedBodies[i];
	    }

	    /** Name of the link */
	    std::string                name;

	    /** The model that owns this link */
	    Robot                     *owner;

	    /** Joint that connects this link to the parent link */
	    Joint                     *before;
	    
	    /** List of descending joints (each connects to a child link) */
	    std::vector<Joint*>        after;
	    
	    /** The constant transform applied to the link (local) */
	    btTransform                constTrans;
	    
	    /** The constant transform applied to the collision geometry of the link (local) */
	    btTransform                constGeomTrans;
	    
	    /** The geometry of the link */
	    shapes::Shape             *shape;
	    
	    /** Attached bodies */
	    std::vector<AttachedBody*> attachedBodies;	    
	    
	    /* ----------------- Computed data -------------------*/
	    
	    /** The global transform this link forwards (computed by forward kinematics) */
	    btTransform                globalTransFwd;

	    /** The global transform for this link (computed by forward kinematics) */
	    btTransform                globalTrans;

	    /* compute the parameter names from this link */
	    unsigned int computeParameterNames(unsigned int pos);

	    /** recompute globalTrans */
	    const double* computeTransform(const double *params, int groupID);

	    /** Extract the information needed by the joint given the URDF description */
	    void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	    
	};
	
	/** A robot structure */
	struct Robot
	{
	    Robot(KinematicModel *model)
	    {
		owner = model;	  
		chain = NULL;
		stateDimension = 0;
		rootTransform.setIdentity();
	    }
	    
	    virtual ~Robot(void)
	    {
		if (chain)
		    delete chain;
	    }
	    
	    /** Add transforms to the rootTransform such that the robot is in its planar/floating link frame.
	     *  Such a transform is needed only if the root joint of the robot is planar or floating */
	    void reduceToRobotFrame(void);

	    /** The model that owns this robot */
	    KinematicModel     *owner;
	    
	    /** A transform that is applied to the entire robot */
	    btTransform         rootTransform;

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

	/** State information */
	struct ModelInfo
	{
	    /** Cumulative list of floating joints */
	    std::vector<int>    floatingJoints;
	    
	    /** Cumulative list of planar joints */
	    std::vector<int>    planarJoints;
	    
	    /** Cumulative state dimension */
	    unsigned int        stateDimension;
	    
	    /** Cumulative state bounds */
	    std::vector<double> stateBounds;
	    
	    /** A map defining the parameter names in the complete state */
	    std::map<std::string, unsigned int>      parameterIndex;
	    std::map<unsigned int, std::string>      parameterName;
	    
	    /** Cumulative index list */
	    std::vector< std::vector<unsigned int> > groupStateIndexList;
	    
	    /** Cumulative list of group roots */
	    std::vector< std::vector<Joint*> >       groupChainStart;

	    /** True if this model has been set in the robot frame */
	    bool                                     inRobotFrame;
	};
	
	/** A class that can hold the named parameters of this planning model */
	class StateParams
	{
	public:

	    StateParams(KinematicModel *model) : m_owner(model), m_mi(model->getModelInfo())
	    {
		assert(model->isBuilt());
		m_params = m_mi.stateDimension > 0 ? new double[m_mi.stateDimension] : NULL;
		setAll(0);
		reset();
	    }
	    
	    virtual ~StateParams(void)
	    {
		if (m_params)
		    delete[] m_params;
	    }
	    
	    /** Mark all values as unseen */
	    void reset(void);

	    /** Mark all values in a group as unseen */
	    void resetGroup(int groupID);

	    /** Mark all values in a group as unseen */
	    void resetGroup(const std::string &group);

	    /** Set all the parameters to a given value */
	    void setAll(const double value);

	    /** Set all the parameters from a group to a given value */
	    void setAllInGroup(const double value, const std::string &group);
	    
	    /** Set all the parameters from a group to a given value */
	    void setAllInGroup(const double value, int groupID);

	    /** Set all planar & floating joints to 0, so that the robot is in its own frame */
	    void setInRobotFrame(void);
	    
	    /** Set the parameters for the complete robot. */
	    bool setParams(const std::vector<double> &params);

	    /** Set the parameters for the complete robot. */
	    bool setParams(const double *params);

	    /** Set the parameters for a given group. Return true if
		any change was observed in either of the set
		values. */
	    bool setParamsGroup(const std::vector<double> &params, const std::string &group);

	    /** Set the parameters for a given group. Return true if
		any change was observed in either of the set
		values. */
	    bool setParamsGroup(const std::vector<double> &params, int groupID);
	    
	    /** Set the parameters for a given group. Return true if
		any change was observed in either of the set
		values. */
	    bool setParamsGroup(const double *params, const std::string &group);

	    /** Set the parameters for a given group. Return true if
		any change was observed in either of the set
		values. */
	    bool setParamsGroup(const double *params, int groupID);

	    /** Given the name of a joint, set the values of the
		parameters describing the joint. Return true if any
		change was observed in the set value */
	    bool setParamsJoint(const double *params, const std::string &name);

	    /** Given the name of a joint, set the values of the
		parameters describing the joint. Return true if any
		change was observed in the set value */
	    bool setParamsJoint(const std::vector<double> &params, const std::string &name);

	    /** Given the name of a joint, get the values of the
		parameters describing the joint. */
	    const double* getParamsJoint(const std::string &name) const;
	    
	    /** Return the current value of the params */
	    const double* getParams(void) const;
	    
	    /** Get the offset for the parameter of a joint in a given group */
	    int getJointIndexInGroup(const std::string &name, const std::string &group) const;

	    /** Get the offset for the parameter of a joint in a given group */
	    int getJointIndexInGroup(const std::string &name, int groupID) const;

	    /** Copy the parameters for a given group to a destination address */
	    void copyParamsGroup(std::vector<double> &params, const std::string &group) const;

	    /** Copy the parameters for a given group to a destination address */
	    void copyParamsGroup(std::vector<double> &params, int groupID) const;

	    /** Copy the parameters for a given group to a destination address */
	    void copyParamsGroup(double *params, const std::string &group) const;

	    /** Copy the parameters for a given group to a destination address */
	    void copyParamsGroup(double *params, int groupID) const;
	    
	    /** Copy all parameters to a destination address */
	    void copyParams(double *params) const;

	    /** Copy all parameters to a destination address */
	    void copyParams(std::vector<double> &params) const;
	    
	    /** Copy the parameters describen a given joint */
	    void copyParamsJoint(double *params, const std::string &name) const;
	    
	    /** Check if all params in a group were seen */
	    bool seenAllGroup(const std::string &group) const;

	    /** Check if all params in a group were seen */
	    bool seenAllGroup(int groupID) const;

	    /** Check if all params were seen */
	    bool seenAll(void) const;
	    
	    /** Print the data from the state to screen */
	    void print(std::ostream &out = std::cout) const;

	    /** Print the missing joint names */
	    void missing(int groupID = -1, std::ostream &out = std::cout);
	    
	protected:
	    
	    KinematicModel                      *m_owner;
	    msg::Interface                       m_msg;
	    ModelInfo                           &m_mi;
	    double                              *m_params;
	    std::map<unsigned int, bool>         m_seen;
	};
	
	
	KinematicModel(void)
	{
	    m_mi.stateDimension = 0;
	    m_mi.inRobotFrame = false;
	    m_ignoreSensors = false;
	    m_verbose = false;	    
	    m_built = false;
	}
	
	virtual ~KinematicModel(void)
	{
	    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
		delete m_robots[i];
	}
	
	void         build(const std::string &description, const std::map< std::string, std::vector<std::string> > &groups, bool ignoreSensors = false);
	virtual void build(const robot_desc::URDF &model, const std::map< std::string, std::vector<std::string> > &groups, bool ignoreSensors = false);
	bool         isBuilt(void) const;
	StateParams* newStateParams(void);
	
	void         setVerbose(bool verbose);	
	
	const std::string& getModelName(void) const;
	
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
	void         getJointsInGroup(std::vector<std::string> &names, int groupID) const;
	/** Get the names of the joints in a specific group */
	void         getJointsInGroup(std::vector<std::string> &names, const std::string &name) const;

	/** Returns the dimension of the group (as a state, not number of joints) */
	unsigned int getGroupDimension(int groupID) const;
	
	/** Returns the dimension of the group (as a state, not number of joints) */
	unsigned int getGroupDimension(const std::string &name) const;

	/** Bring the robot to a default state */
	void defaultState(void);

	/** Apply the transforms to a group, based on the params */
	void computeTransformsGroup(const double *params, int groupID);

	/** Apply the transforms to the entire robot, based on the params */
	void computeTransforms(const double *params);
	
	/** Add thansforms to the rootTransform such that the robot is in its planar/floating link frame */
	void reduceToRobotFrame(void);

	/** Provide interface to a lock. Use carefully! */
	void lock(void)
	{
	    m_lock.lock();
	}
	
	/** Provide interface to a lock. Use carefully! */
	void unlock(void)
	{
	    m_lock.unlock();
	}
	
	void printModelInfo(std::ostream &out = std::cout);
	void printLinkPoses(std::ostream &out = std::cout) const;
	
	ModelInfo&       getModelInfo(void);
	const ModelInfo& getModelInfo(void) const;
	
    protected:
	
	/** The name of the model */
	std::string                       m_name;
	ModelInfo                         m_mi;
	
	std::vector<Robot*>               m_robots;
	std::vector<std::string>          m_groups;
	std::map<std::string, int>        m_groupsMap;	       
	std::map<std::string, Link*>      m_linkMap;	
	std::map<std::string, Joint*>     m_jointMap;
	bool                              m_ignoreSensors;
	bool                              m_verbose;    
	bool                              m_built;

	boost::mutex                      m_lock;
	msg::Interface                    m_msg;
	
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
