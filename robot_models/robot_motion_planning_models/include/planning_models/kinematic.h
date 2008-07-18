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

#ifndef KINEMATIC_ROBOT_MODEL_
#define KINEMATIC_ROBOT_MODEL_

#include <urdf/URDF.h>
#include <libTF/Pose3D.h>
#include <vector>
#include <string>

/** @htmlinclude ../../manifest.html

    A class describing a kinematic robot model loaded from URDF */

namespace robot_models
{
    
    
    class KinematicModel
    {
    public:
	
	
	/** Possible geometry elements. These are assumed to be centered at origin (similar to ODE) */
	struct Geometry
	{
	    Geometry(void)
	    {
		type = UNKNOWN;
		size[0] = size[1] = size[2] = 0.0;
	    }
	    
	    enum 
		{
		    UNKNOWN, BOX, CYLINDER, SPHERE
		}  type;
	    double size[3];
	};
	
	struct Joint;
	struct Link;	
	
	/** A joint from the robot. Contains the transform applied by the joint type */
	struct Joint
	{
	    Joint(void)
	    {
		before = after = NULL;
		usedParamStart = usedParamEnd = 0;
		axis[0] = axis[1] = axis[2] = 0.0;
		anchor[0] = anchor[1] = anchor[2] = 0.0;
		limit[0] = limit[1] = 0.0;
		type = UNKNOWN;
	    }
	    
	    ~Joint(void)
	    {
		if (after)
		    delete after;
	    }
	    
	    /* the links that this joint connects */	    
	    Link         *before;
	    Link         *after;
	    
	    /* the range of indices in the parameter vector that
	       needed to access information about the position of this
	       joint */
	    unsigned int  usedParamStart;
	    unsigned int  usedParamEnd;
	    bool          active;
	    
	    /* relevant joint information */
	    enum
		{
		    UNKNOWN, FIXED, REVOLUTE, PRISMATIC, FLOATING
		}         type;
	    double        axis[3];
	    double        anchor[3];
	    double        limit[2];
	    
	    /* ----------------- Computed data -------------------*/
	    
	    /* the local transform (computed by forward kinematics) */
	    libTF::Pose3D varTrans;
	    libTF::Pose3D globalTrans;
	    
	    void computeTransform(const double *params);
	    
	};
	
	/** A link from the robot. Contains the constant transform applied to the link and its geometry */
	struct Link
	{
	    Link(void)
	    {
		before = NULL;
		geom = new Geometry();
	    }
	    
	    ~Link(void)
	    {
		if (geom)
		    delete geom;
		for (unsigned int i = 0 ; i < after.size() ; ++i)
		    delete after[i];
	    }
	    
	    /* joint that connects this link to the parent link */
	    Joint              *before;
	    
	    /* list of descending joints (each connects to a child link) */
	    std::vector<Joint*> after;
	    
	    /* the constant transform applied to the link (local) */
	    libTF::Pose3D       constTrans;
	    
	    /* the constant transform applied to the collision geometry of the link (local) */
	    libTF::Pose3D       constGeomTrans;
	    
	    /* the geometry of the link */
	    Geometry           *geom;
	    
	    /* ----------------- Computed data -------------------*/
	    
	    /* the global transform for this link (computed by forward kinematics) */
	    libTF::Pose3D       globalTrans;
	    
	    void computeTransform(const double *params);	
	};
	
	/** A robot structure */
	struct Robot
	{
	    Robot(void)
	    {
		chain = NULL;
		stateDimension = 0;
	    }
	    
	    virtual ~Robot(void)
	    {
		if (chain)
		    delete chain;
	    }
	    
	    void computeTransforms(const double *params);
	    
	    /** List of links in the robot */
	    std::vector<Link*>  links;
	    
	    /** List of leaf links (have no child links) */
	    std::vector<Link*>  leafs;
	    
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
	    
	    /** Group of links corresponding to this robot (if any) */
	    std::string         tag;
	    
	};
	
	KinematicModel(void)
	{
	    m_verbose = false;
	}
	
	virtual ~KinematicModel(void)
	{
	    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
		delete m_robots[i];
	}
	
	virtual void build(robot_desc::URDF &model, const char *group = NULL);
	
	unsigned int getRobotCount(void) const;
	Robot* getRobot(unsigned int index) const;
	
    protected:
	
	std::vector<Robot*> m_robots;
	bool                m_verbose;    
	
    private:
	
	void buildChain(Robot *robot, Link  *parent, Joint *joint, robot_desc::URDF::Link *urdfLink);
	void buildChain(Robot *robot, Joint *parent, Link  *link,  robot_desc::URDF::Link *urdfLink);
	
    };

}

#endif
