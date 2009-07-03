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

#include <geometric_shapes/shapes.h>
#include "planning_models/output.h"

#include <urdf/URDF.h>
#include <LinearMath/btTransform.h>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>


/** \brief Main namespace */
namespace planning_models
{
 
    class StateParams;
    
    /** \brief Definition of a kinematic model. This class is not thread
	safe, however multiple instances can be created */
    class KinematicModel
    {
    public:
	
	/** \brief Forward definition of a joint */
	class Joint;
	
	/** \brief Forward definition of a link */
	class Link;
	
	/** \brief Forward definition of a robot */
	class Robot;
	
	/** \brief A joint from the robot. Contains the transform applied by the joint type */
	class Joint
	{
	    friend class KinematicModel;	
	    friend class Link;
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


	    /** \brief Name of the joint */
	    std::string       name;
	    
	    /** \brief The robot that owns this joint */
	    Robot            *owner;

	    /** \brief the links that this joint connects */	    
	    Link             *before;
	    Link             *after;
	    
	    /** \brief The range of indices in the parameter vector that
		needed to access information about the position of this
		joint */
	    unsigned int      usedParams;
	    
	    /** \brief Bitvector identifying which groups this joint is part of */
	    std::vector<bool> inGroup;

	protected:

	    /** \brief the local transform (computed by forward kinematics) */
	    btTransform       varTrans;

	    /* Compute the parameter names from this joint; the
	       purpose is to create a map that relates the name of a
	       joint in the loaded description to the position in a
	       vector of parameters */
	    unsigned int computeParameterNames(unsigned int pos);

	    /** \brief Update varTrans if this joint is part of the group indicated by groupID
	     *  and recompute globalTrans using varTrans */
	    const double* computeTransform(const double *params, int groupID);
	    
	    /** \brief Update the value of VarTrans using the information from params */
	    virtual void updateVariableTransform(const double *params) = 0;

	    /** \brief Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot) = 0;
	};

	/** \brief A fixed joint */
	class FixedJoint : public Joint
	{
	protected:
	    
	    /** \brief Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** \brief Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	};

	/** \brief A planar joint */
	class PlanarJoint : public Joint
	{
	public:
	    
	    PlanarJoint(void) : Joint()
	    {
	        usedParams = 3; // (x, y, theta)
	    }

	protected:
	    /** \brief Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** \brief Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	};

	/** \brief A floating joint */
	class FloatingJoint : public Joint
	{
	public:
	    
	    FloatingJoint(void) : Joint()
	    {
	        usedParams = 7; // vector: (x, y, z)  quaternion: (x, y, z, w)
	    }

	protected:

	    /** \brief Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** \brief Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	};

	/** \brief A prismatic joint */
	class PrismaticJoint : public Joint
	{
	public:
	    
	    PrismaticJoint(void) : Joint(), axis(0.0, 0.0, 0.0)
	    {
		limit[0] = limit[1] = 0.0;
		usedParams = 1;
	    }

	    btVector3 axis;
	    double    limit[2];

	protected:
	    /** \brief Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** \brief Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);
	    
	};
	
	/** \brief A revolute joint */
	class RevoluteJoint : public Joint
	{
	public:
	    
	    RevoluteJoint(void) : Joint(), axis(0.0, 0.0, 0.0), anchor(0.0, 0.0, 0.0)
	    {
		limit[0] = limit[1] = 0.0;
		continuous = false;
		usedParams = 1;
	    }
	    	    
	    btVector3 axis;
	    btVector3 anchor;
	    double    limit[2];
	    bool      continuous;

	protected:

	    /** \brief Update the value of varTrans using the information from params */
	    virtual void updateVariableTransform(const double *params);

	    /** \brief Extract the information needed by the joint given the URDF description */
	    virtual void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot);

	};
	
	/** \brief Class defining bodies that can be attached to robot
	    links. This is useful when handling objects picked up by
	    the robot. */
	class AttachedBody
	{
	    friend class KinematicModel;
	    friend class Link;
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
	    
	    /** \brief The constant transform applied to the link (needs to be specified by user) */
	    btTransform         attachTrans;

	    /** \brief The geometry of the attached body */
	    shapes::Shape*      shape;

	    /** \brief The global transform for this link (computed by forward kinematics) */
	    btTransform         globalTrans;
	    
	protected:
	    /** \brief recompute globalTrans */
	    void computeTransform(btTransform &parentTrans);
	};
	
	
	/** \brief A link from the robot. Contains the constant transform applied to the link and its geometry */
	class Link
	{
	    friend class KinematicModel;
	    friend class Joint;
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

	    /** \brief Name of the link */
	    std::string                name;

	    /** \brief The model that owns this link */
	    Robot                     *owner;

	    /** \brief Joint that connects this link to the parent link */
	    Joint                     *before;
	    
	    /** \brief List of descending joints (each connects to a child link) */
	    std::vector<Joint*>        after;
	    
	    /** \brief The constant transform applied to the link (local) */
	    btTransform                constTrans;
	    
	    /** \brief The constant transform applied to the collision geometry of the link (local) */
	    btTransform                constGeomTrans;
	    
	    /** \brief The geometry of the link */
	    shapes::Shape             *shape;
	    
	    /** \brief Attached bodies */
	    std::vector<AttachedBody*> attachedBodies;	    
	    
	    /* ----------------- Computed data -------------------*/
	    
	    /** \brief The global transform this link forwards (computed by forward kinematics) */
	    btTransform                globalTransFwd;

	    /** \brief The global transform for this link (computed by forward kinematics) */
	    btTransform                globalTrans;

	protected:

	    /* compute the parameter names from this link */
	    unsigned int computeParameterNames(unsigned int pos);

	    /** \brief recompute globalTrans */
	    const double* computeTransform(const double *params, int groupID);

	    /** \brief Extract the information needed by the joint given the URDF description */
	    void extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot, const robot_desc::URDF &model);
	    
	};
	
	/** \brief A robot structure */
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
	    
	    /** \brief The model that owns this robot */
	    KinematicModel     *owner;
	    
	    /** \brief A transform that is applied to the entire robot */
	    btTransform         rootTransform;

	    /** \brief List of links in the robot */
	    std::vector<Link*>  links;

	    /** \brief List of joints in the robot */
	    std::vector<Joint*> joints;
	    
	    /** \brief The first joint in the robot -- the root */
	    Joint              *chain;	    

	    /** \brief Number of parameters needed to define the joints */
	    unsigned int        stateDimension;
	    
	    /** \brief The bounding box for the set of parameters describing the
	     *  joints.  This array contains 2 * stateDimension elements:
	     *  positions 2*i and 2*i+1 define the minimum and maximum
	     *  values for the parameter. If both minimum and maximum are
	     *  set to 0, the parameter is unbounded. */
	    std::vector<double> stateBounds;
	    
	    /** \brief The list of index values where floating joints
		start. These joints need special attention in motion
		planning, so the indices are provided here for
		convenience. */
	    std::vector<int>    floatingJoints;

	    /** \brief The list of index values where planar joints
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

	/** \brief State information */
	struct ModelInfo
	{
	    /** \brief Cumulative list of floating joints */
	    std::vector<int>    floatingJoints;
	    
	    /** \brief Cumulative list of planar joints */
	    std::vector<int>    planarJoints;
	    
	    /** \brief Cumulative state dimension */
	    unsigned int        stateDimension;
	    
	    /** \brief Cumulative state bounds */
	    std::vector<double> stateBounds;
	    
	    /** \brief A map defining the parameter names in the complete state */
	    std::map<std::string, unsigned int>      parameterIndex;
	    std::map<unsigned int, std::string>      parameterName;
	    
	    /** \brief Cumulative index list */
	    std::vector< std::vector<unsigned int> > groupStateIndexList;
	    
	    /** \brief Cumulative list of group roots */
	    std::vector< std::vector<Joint*> >       groupChainStart;

	    /** \brief True if this model has been set in the robot frame */
	    bool                                     inRobotFrame;
	};
	

	KinematicModel(void)
	{
	    m_mi.stateDimension = 0;
	    m_mi.inRobotFrame = false;
	    m_verbose = false;	    
	    m_built = false;
	}
	
	virtual ~KinematicModel(void)
	{
	    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
		delete m_robots[i];
	}
	
	void         build(const std::string &description, const std::map< std::string, std::vector<std::string> > &groups);
	virtual void build(const robot_desc::URDF &model, const std::map< std::string, std::vector<std::string> > &groups);
	bool         isBuilt(void) const;
	StateParams* newStateParams(void);
	
	void         setVerbose(bool verbose);	
	

	/** \brief General the model name **/
	const std::string& getModelName(void) const;
	
	/** \brief Get the number of robots in the model */
	unsigned int getRobotCount(void) const;

	/** \brief Get the datastructure associated to a specific robot */
	Robot*       getRobot(unsigned int index) const;

	/** \brief Get the array of planning groups, indexed by their group ID */
	void         getGroups(std::vector<std::string> &groups) const;
	
	/** \brief Get the number of groups */
	unsigned int getGroupCount(void) const;
	
	/** \brief Get the group ID of a group */
	int          getGroupID(const std::string &group) const;
	
	/** \brief Get a link by its name */
	Link*        getLink(const std::string &link) const;

	/** \brief Get the array of links, in no particular order */
	void         getLinks(std::vector<Link*> &links) const;

	/** \brief Get a joint by its name */
	Joint*       getJoint(const std::string &joint) const;

	/** \brief Get the array of joints, ordered in the same way as in the state vector */
	void         getJoints(std::vector<Joint*> &joints) const;

	/** \brief Get the names of the joints in a specific group. Only joints with paramteres are returned and the order is the 
	    same as in the group state */
	void         getJointsInGroup(std::vector<std::string> &names, int groupID) const;
	
	/** \brief Get the names of the joints in a specific group. Only joints with parameters are returned and the order is the 
	    same as in the group state */
	void         getJointsInGroup(std::vector<std::string> &names, const std::string &name) const;

	/** \brief Return the index of a joint in the complete state vector */
	int getJointIndex(const std::string &name) const;

	/** \brief Get the index for the parameter of a joint in a given group */
	int getJointIndexInGroup(const std::string &name, const std::string &group) const;

	/** \brief Get the index for the parameter of a joint in a given group */
	int getJointIndexInGroup(const std::string &name, int groupID) const;

	/** \brief Returns the dimension of the group (as a state, not number of joints) */
	unsigned int getGroupDimension(int groupID) const;
	
	/** \brief Returns the dimension of the group (as a state, not number of joints) */
	unsigned int getGroupDimension(const std::string &name) const;

	/** \brief Bring the robot to a default state */
	void defaultState(void);

	/** \brief Apply the transforms to a group, based on the params */
	void computeTransformsGroup(const double *params, int groupID);

	/** \brief Apply the transforms to the entire robot, based on the params */
	void computeTransforms(const double *params);
	
	/** \brief Add transforms to the rootTransform such that the robot is in its planar/floating link frame.
	 *  Such a transform is needed only if the root joint of the robot is planar or floating */
	void reduceToRobotFrame(void);

	/** \brief Provide interface to a lock. Use carefully! */
	void lock(void)
	{
	    m_lock.lock();
	}
	
	/** \brief Provide interface to a lock. Use carefully! */
	void unlock(void)
	{
	    m_lock.unlock();
	}
	
	void printModelInfo(std::ostream &out = std::cout);
	void printLinkPoses(std::ostream &out = std::cout) const;
	
	ModelInfo&       getModelInfo(void);
	const ModelInfo& getModelInfo(void) const;
	
    protected:
	
	/** \brief The name of the model */
	std::string                                       m_name;
	ModelInfo                                         m_mi;
	
	std::vector<Robot*>                               m_robots;
	std::vector<std::string>                          m_groups;
	std::map<std::string, int>                        m_groupsMap;
	std::map< std::string, std::vector<std::string> > m_groupContent;
	
	std::map<std::string, Link*>                      m_linkMap;	
	std::map<std::string, Joint*>                     m_jointMap;
	std::map< std::string, std::map<int, int> >       m_jointIndexGroup;
	

	bool                                              m_verbose;    
	bool                                              m_built;

	boost::mutex                                      m_lock;
	msg::Interface                                    m_msg;
	
    private:
	

	/** \brief Build the needed datastructure for a joint */
	void buildChainJ(Robot *robot, Link  *parent, Joint *joint, const robot_desc::URDF::Link *urdfLink, const robot_desc::URDF &model);

	/** \brief Build the needed datastructure for a link */
	void buildChainL(Robot *robot, Joint *parent, Link  *link,  const robot_desc::URDF::Link *urdfLink, const robot_desc::URDF &model);

	/** \brief Construct the list of groups the model knows about (the ones marked with the 'plan' attribute) */
	void constructGroupList();
	
	/* compute the parameter names  */
	void computeParameterNames(void);

	/** \brief Get the index for the parameter of a joint in a given group */
	int getJointIndexInGroupSlow(const std::string &name, int groupID) const;
	
	/** \brief Allocate a joint of appropriate type, depending on the loaded link */
	Joint* createJoint(const robot_desc::URDF::Link* urdfLink);
	
    };

}

#endif
