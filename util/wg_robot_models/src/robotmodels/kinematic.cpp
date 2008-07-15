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

#include <robotmodels/kinematic.h>
#include <cstdio>

void KinematicModel::Robot::computeTransforms(const double *params)
{
    chain->computeTransform(params);
}

// we can optimize things here... (when we use identity transforms, for example)
void KinematicModel::Joint::computeTransform(const double *params)
{
    switch (type)
    {
    case Joint::REVOLUTE:
	varTrans.setAxisAngle(axis, params[usedParamStart]);
	break;
    case Joint::PRISMATIC:
	{
	    double p  = params[usedParamStart];
	    double dx = axis[0] * p;
	    double dy = axis[1] * p;
	    double dz = axis[2] * p;
	    varTrans.setPosition(dx, dy, dz);
	}
	break;
    case Joint::FLOATING:
	varTrans.setPosition(params[usedParamStart], params[usedParamStart + 1], params[usedParamStart + 2]);
	break;
    default:
	break;
    }
    if (before)
    {
	globalTrans = before->globalTrans;
	globalTrans.multiplyPose(varTrans);
    }
    else
	globalTrans = varTrans;
    after->computeTransform(params);
}

void KinematicModel::Link::computeTransform(const double *params)
{
    globalTrans = before->globalTrans;
    globalTrans.multiplyPose(constTrans);
    for (unsigned int i = 0 ; i < after.size() ; ++i)
	after[i]->computeTransform(params);
    globalTrans.multiplyPose(constGeomTrans);
}

void KinematicModel::build(URDF &model, const char *group)
{
    if (group)
    {
	URDF::Group *g = model.getGroup(group);
	for (unsigned int i = 0 ; i < g->linkRoots.size() ; ++i)
	{
	    URDF::Link *link = g->linkRoots[i];
	    Robot *rb = new Robot();
	    rb->tag = g->name;
	    rb->chain = new Joint();
	    buildChain(rb, NULL, rb->chain, link);
	    m_robots.push_back(rb);
	}
    }
    else
    {
	for (unsigned int i = 0 ; i < model.getDisjointPartCount() ; ++i)
	{
	    URDF::Link *link = model.getDisjointPart(i);
	    Robot *rb = new Robot();
	    rb->chain = new Joint();
	    buildChain(rb, NULL, rb->chain, link);
	    m_robots.push_back(rb);
	}
    }
}
    
unsigned int KinematicModel::getRobotCount(void) const
{
    return m_robots.size();
}

KinematicModel::Robot* KinematicModel::getRobot(unsigned int index) const
{
    return m_robots[index];
}

void KinematicModel::buildChain(Robot *robot, Link *parent, Joint* joint, URDF::Link* urdfLink)
{
    joint->usedParamStart = robot->stateDimension;
    joint->before = parent;
    joint->after  = new Link();
    
    /* copy relevant data */
    joint->limit[0] = urdfLink->joint->limit[0];
    joint->limit[1] = urdfLink->joint->limit[1];
    joint->axis[0] = urdfLink->joint->axis[0];
    joint->axis[1] = urdfLink->joint->axis[1];
    joint->axis[2] = urdfLink->joint->axis[2];
    joint->anchor[0] = urdfLink->joint->anchor[0];
    joint->anchor[1] = urdfLink->joint->anchor[1];
    joint->anchor[2] = urdfLink->joint->anchor[2];
    switch (urdfLink->joint->type)
    {
    case URDF::Link::Joint::FLOATING:
	joint->type = Joint::FLOATING;
	joint->usedParamEnd = joint->usedParamStart + 3;
	robot->stateBounds.insert(robot->stateBounds.end(), 6, 0.0);
	break;
    case URDF::Link::Joint::FIXED:
	joint->type = Joint::FIXED; 
	joint->usedParamEnd = joint->usedParamStart;
	break;
    case URDF::Link::Joint::REVOLUTE:
	joint->type = Joint::REVOLUTE;
	joint->usedParamEnd = joint->usedParamStart + 1;
	robot->stateBounds.push_back(joint->limit[0]);
	robot->stateBounds.push_back(joint->limit[1]);
	break;
    case URDF::Link::Joint::PRISMATIC:
	joint->type = Joint::PRISMATIC;
	joint->usedParamEnd = joint->usedParamStart + 1;
	robot->stateBounds.push_back(joint->limit[0]);
	robot->stateBounds.push_back(joint->limit[1]);
	break;
    default:
	joint->type = Joint::UNKNOWN; 
	joint->usedParamEnd = joint->usedParamStart;
	break;
    }
    joint->active = joint->usedParamEnd > joint->usedParamStart;
    robot->stateDimension = joint->usedParamEnd;	    
    buildChain(robot, joint, joint->after, urdfLink);
}

void KinematicModel::buildChain(Robot *robot, Joint *parent, Link* link, URDF::Link* urdfLink)
{
    link->before = parent;
    robot->links.push_back(link);
    
    if (m_verbose)
	printf("Created link '%s'\n", urdfLink->name.c_str());
    
    /* copy relevant data */ 
    switch (urdfLink->collision->geometry->type)
    {
    case URDF::Link::Geometry::UNKNOWN:
	link->geom->type = Geometry::UNKNOWN; break;
    case URDF::Link::Geometry::BOX:
	link->geom->type = Geometry::BOX; break;
    case URDF::Link::Geometry::SPHERE:
	link->geom->type = Geometry::SPHERE; break;
    case URDF::Link::Geometry::CYLINDER:
	link->geom->type = Geometry::CYLINDER; break;
    default:
	break;
    }
    
    /* copy the geometry of the link */
    link->geom->size[0] = urdfLink->collision->geometry->size[0];
    link->geom->size[1] = urdfLink->collision->geometry->size[1];
    link->geom->size[2] = urdfLink->collision->geometry->size[2];
    
    /* compute the constant transform for this link */
    double *xyz = urdfLink->xyz;
    double *rpy = urdfLink->rpy;
    link->constTrans.setFromEuler(xyz[0], xyz[1], xyz[2], rpy[2], rpy[1], rpy[0]);	    
    
    xyz = urdfLink->collision->xyz;
    rpy = urdfLink->collision->rpy;
    link->constGeomTrans.setFromEuler(xyz[0], xyz[1], xyz[2], rpy[2], rpy[1], rpy[0]);	    
    
    for (unsigned int i = 0 ; i < urdfLink->children.size() ; ++i)
    {
	/* if building from a group of links, make sure we do not exit the group */
	if (!robot->tag.empty())
	{
	    bool found = false;
	    for (unsigned int k = 0 ; k < urdfLink->children[i]->groups.size() ; ++k)
		if (urdfLink->children[i]->groups[k]->name == robot->tag)
		{
		    found = true;
		    break;
		}
	    if (!found)
		continue;
	}
	if (m_verbose)
	    printf("Connecting to link '%s'\n", urdfLink->children[i]->name.c_str());
	Joint *newJoint = new Joint();
	buildChain(robot, link, newJoint, urdfLink->children[i]);
	link->after.push_back(newJoint);
    }

    if (link->after.size() == 0)
	robot->leafs.push_back(link);
}
