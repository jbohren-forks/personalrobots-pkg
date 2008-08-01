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

#include <planning_models/kinematic.h>
#include <cmath>

void planning_models::KinematicModel::Robot::computeTransforms(const double *params, int groupID)
{
    if (groupID >= 0)
	chain->computeTransform(params, groupID);
    else
    {
	chain->globalTrans = owner->rootTransform;
	chain->computeTransform(params, groupID);
    }
}

void planning_models::KinematicModel::computeTransforms(const double *params, int groupID)
{
    if (groupID >= 0)
	for (unsigned int i = 0 ; i < groupChainStart[groupID].size() ; ++i)
	{
	    groupChainStart[groupID][i]->computeTransform(params, groupID);
	    params += groupStateIndexList[groupID].size();
	}    
    else
	for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
	{
	    m_robots[i]->chain->globalTrans = rootTransform;
	    m_robots[i]->chain->computeTransform(params, groupID);
	    params += m_robots[i]->stateDimension;	    
	} 
}

// we can optimize things here... (when we use identity transforms, for example)
void planning_models::KinematicModel::Joint::computeTransform(const double *params, int groupID)
{
    if (groupID < 0 || inGroup[groupID])
	switch (type)
	{
	case Joint::REVOLUTE:
	    varTrans.setAxisAngle(axis, params[0]);
	    break;
	case Joint::PRISMATIC:
	    {
		double p  = params[0];
		double dx = axis[0] * p;
		double dy = axis[1] * p;
		double dz = axis[2] * p;
		varTrans.setPosition(dx, dy, dz);
	    }
	    break;
	case Joint::PLANAR:
	    varTrans.setPosition(params[0], params[1], 0.0);
	    varTrans.setAxisAngle(0.0, 0.0, 1.0, params[2]);
	    break;
	case Joint::FLOATING:
	    varTrans.setPosition(params[0], params[1], params[2]);
	    varTrans.setQuaternion(params[3], params[4], params[5], params[6]);
	    break;
	default:
	    break;
	}

    if (before)
	globalTrans = before->globalTrans; // otherwise, the caller initialized globalTrans already
    globalTrans.multiplyPose(varTrans);
    after->computeTransform(params + usedParams, groupID);
}

void planning_models::KinematicModel::Link::computeTransform(const double *params, int groupID)
{
    globalTrans = before->globalTrans;
    globalTrans.multiplyPose(constTrans);
    for (unsigned int i = 0 ; i < after.size() ; ++i)
	after[i]->computeTransform(params, groupID);
    globalTrans.multiplyPose(constGeomTrans);
}

void planning_models::KinematicModel::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

void planning_models::KinematicModel::constructGroupList(robot_desc::URDF &model)
{
    std::string rname = model.getRobotName();
    std::vector<std::string> allGroups;
    model.getGroupNames(allGroups);
    m_groups.clear();
    for (unsigned int i = 0 ; i < allGroups.size() ; ++i)
	if (model.getGroup(allGroups[i])->hasFlag("plan"))
	    m_groups.push_back(rname + "::" + allGroups[i]);
    m_groupsMap.clear();
    for (unsigned int i = 0 ; i < m_groups.size() ; ++i)
	m_groupsMap[m_groups[i]] = i;
}

void planning_models::KinematicModel::build(robot_desc::URDF &model, const char *group)
{
    if (m_built)
    {
	fprintf(stderr, "Model has already been built!\n");
	return;
    }
    
    m_built = true;

    /* construct a map for the available groups */
    constructGroupList(model);
    groupStateIndexList.resize(m_groups.size());
    groupChainStart.resize(m_groups.size());
    
    if (group)
    {
	robot_desc::URDF::Group *g = model.getGroup(group);
	if (g)
	{
	    if (g->hasFlag("plan"))
		for (unsigned int i = 0 ; i < g->linkRoots.size() ; ++i)
		{
		    robot_desc::URDF::Link *link = g->linkRoots[i];
		    Robot *rb = new Robot(this);
		    rb->groupStateIndexList.resize(m_groups.size());
		    rb->groupChainStart.resize(m_groups.size());
		    rb->tag = g->name;
		    rb->chain = new Joint();
		    buildChainJ(rb, NULL, rb->chain, link, model);
		    m_robots.push_back(rb);
		}
	    else
		fprintf(stderr, "Group '%s' is not marked for planning ('plan' flag).\n", group);
	}
	else
	    fprintf(stderr, "Group '%s' not found.\n", group);
    }
    else
    {
	for (unsigned int i = 0 ; i < model.getDisjointPartCount() ; ++i)
	{
	    robot_desc::URDF::Link *link = model.getDisjointPart(i);
	    Robot *rb = new Robot(this);
	    rb->groupStateIndexList.resize(m_groups.size());
	    rb->groupChainStart.resize(m_groups.size());
	    rb->chain = new Joint();
	    buildChainJ(rb, NULL, rb->chain, link, model);
	    m_robots.push_back(rb);
	}
    }

    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
    {
	/* copy state bounds */
	stateBounds.insert(stateBounds.end(), m_robots[i]->stateBounds.begin(), m_robots[i]->stateBounds.end());
	
	/* copy floating joints*/
	for (unsigned int j = 0 ; j < m_robots[i]->floatingJoints.size() ; ++j)
	    floatingJoints.push_back(stateDimension + m_robots[i]->floatingJoints[j]);

	/* copy planar joints*/
	for (unsigned int j = 0 ; j < m_robots[i]->planarJoints.size() ; ++j)
	    planarJoints.push_back(stateDimension + m_robots[i]->planarJoints[j]);
	
	/* copy group roots */
	for (unsigned int j = 0 ; j < m_robots[i]->groupChainStart.size() ; ++j)
	    groupChainStart[j].insert(groupChainStart[j].end(), m_robots[i]->groupChainStart[j].begin(), m_robots[i]->groupChainStart[j].end());
	
	/* copy state index list */
	for (unsigned int j = 0 ; j < m_robots[i]->groupStateIndexList.size() ; ++j)
	    for (unsigned int k = 0 ; k < m_robots[i]->groupStateIndexList[j].size() ; ++k)
		groupStateIndexList[j].push_back(stateDimension + m_robots[i]->groupStateIndexList[j][k]);
	
	stateDimension += m_robots[i]->stateDimension;
    }
}

int planning_models::KinematicModel::getGroupID(const std::string &group) const
{
    std::map<std::string, int>::const_iterator pos = m_groupsMap.find(group);
    return pos == m_groupsMap.end() ? -1 : pos->second;
}

void planning_models::KinematicModel::getGroups(std::vector<std::string> &groups) const
{
    groups = m_groups;
}

unsigned int planning_models::KinematicModel::getRobotCount(void) const
{
    return m_robots.size();
}

planning_models::KinematicModel::Robot* planning_models::KinematicModel::getRobot(unsigned int index) const
{
    return m_robots[index];
}

void planning_models::KinematicModel::buildChainJ(Robot *robot, Link *parent, Joint* joint, robot_desc::URDF::Link* urdfLink, robot_desc::URDF &model)
{
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
    if (!urdfLink->joint->isSet["limit"] && urdfLink->joint->type == robot_desc::URDF::Link::Joint::REVOLUTE)
    {
	joint->limit[0] = -M_PI;
	joint->limit[1] =  M_PI;
    }

    switch (urdfLink->joint->type)
    {
    case robot_desc::URDF::Link::Joint::FLOATING:
	joint->type = Joint::FLOATING;
	joint->usedParams = 7;
	robot->stateBounds.insert(robot->stateBounds.end(), 14, 0.0);
	robot->floatingJoints.push_back(robot->stateDimension);
	break;
    case robot_desc::URDF::Link::Joint::PLANAR:
	joint->type = Joint::PLANAR;
	joint->usedParams = 3;
	robot->stateBounds.insert(robot->stateBounds.end(), 4, 0.0);
	robot->stateBounds.push_back(-M_PI);
	robot->stateBounds.push_back(M_PI);
	robot->planarJoints.push_back(robot->stateDimension);
	break;
    case robot_desc::URDF::Link::Joint::FIXED:
	joint->type = Joint::FIXED; 
	joint->usedParams = 0;
	break;
    case robot_desc::URDF::Link::Joint::REVOLUTE:
	joint->type = Joint::REVOLUTE;
	joint->usedParams = 1;
	robot->stateBounds.push_back(joint->limit[0]);
	robot->stateBounds.push_back(joint->limit[1]);
	break;
    case robot_desc::URDF::Link::Joint::PRISMATIC:
	joint->type = Joint::PRISMATIC;
	joint->usedParams = 1;
	robot->stateBounds.push_back(joint->limit[0]);
	robot->stateBounds.push_back(joint->limit[1]);
	break;
    default:
	joint->type = Joint::UNKNOWN; 
	joint->usedParams = 0;
	break;
    }
    
    /** construct the inGroup bitvector */
    std::vector<std::string> gnames;
    model.getGroupNames(gnames);
    for (unsigned int i = 0 ; i < gnames.size() ; ++i)
	if (model.getGroup(gnames[i])->hasFlag("plan"))
	    joint->inGroup.push_back(urdfLink->inGroup[i]);
    
    for (unsigned int i = 0 ; i < joint->inGroup.size() ; ++i)
	if (joint->inGroup[i])
	    for (unsigned int j = 0 ; j < joint->usedParams ; ++j)
		robot->groupStateIndexList[i].push_back(j + robot->stateDimension);
    
    for (unsigned int k = 0 ; k < urdfLink->groups.size() ; ++k)
	if (urdfLink->groups[k]->isRoot(urdfLink))
	{
	    std::string gname = model.getRobotName() + "::" + urdfLink->groups[k]->name;
	    if (m_groupsMap.find(gname) != m_groupsMap.end())
		robot->groupChainStart[m_groupsMap[gname]].push_back(joint);
	}
    
    if (m_verbose && joint->usedParams > 0)
    {
	printf("Joint '%s' connects link '%s' to link '%s' and uses state coordinates: ",
	       urdfLink->joint->name.c_str(), urdfLink->parentName.c_str(), urdfLink->name.c_str());
	for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
	    printf("%d ", i + robot->stateDimension);
	printf("\n");
    }
    
    robot->stateDimension += joint->usedParams;
    buildChainL(robot, joint, joint->after, urdfLink, model);
}

void planning_models::KinematicModel::buildChainL(Robot *robot, Joint *parent, Link* link, robot_desc::URDF::Link* urdfLink, robot_desc::URDF &model)
{
    link->name = urdfLink->name;
    link->before = parent;
    robot->links.push_back(link);
    
    /* copy relevant data */ 
    switch (urdfLink->collision->geometry->type)
    {
    case robot_desc::URDF::Link::Geometry::UNKNOWN:
	link->geom->type = Geometry::UNKNOWN; break;
    case robot_desc::URDF::Link::Geometry::BOX:
	link->geom->type = Geometry::BOX; break;
    case robot_desc::URDF::Link::Geometry::SPHERE:
	link->geom->type = Geometry::SPHERE; break;
    case robot_desc::URDF::Link::Geometry::CYLINDER:
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
	Joint *newJoint = new Joint();
	buildChainJ(robot, link, newJoint, urdfLink->children[i], model);
	link->after.push_back(newJoint);
    }

    if (link->after.size() == 0)
	robot->leafs.push_back(link);
}

void planning_models::KinematicModel::printModelInfo(FILE *out) const
{   
    fprintf(out, "Number of robots = %d\n", getRobotCount());
    fprintf(out, "Complete model state dimension = %d\n", stateDimension);

    fprintf(out, "State bounds: ");
    for (unsigned int i = 0 ; i < stateDimension ; ++i)
	fprintf(out, "[%f, %f] ", stateBounds[2 * i], stateBounds[2 * i + 1]);
    fprintf(out, "\n");
    
    fprintf(out, "Floating joints at: ");    
    for (unsigned int i = 0 ; i < floatingJoints.size() ; ++i)
	fprintf(out, "%d ", floatingJoints[i]);
    fprintf(out, "\n");

    fprintf(out, "Planar joints at: ");    
    for (unsigned int i = 0 ; i < planarJoints.size() ; ++i)
	fprintf(out, "%d ", planarJoints[i]);
    fprintf(out, "\n");

    fprintf(out, "Available groups: ");    
    std::vector<std::string> l;    
    getGroups(l);
    for (unsigned int i = 0 ; i < l.size() ; ++i)
	fprintf(out, "%s ", l[i].c_str());
    fprintf(out, "\n");
    
    for (unsigned int i = 0 ; i < l.size() ; ++i)
    {
	int gid = getGroupID(l[i]);
	fprintf(out, "Group %s has %d roots\n", l[i].c_str(), groupChainStart[gid].size());
	fprintf(out, "The state components for this group are: ");
	for (unsigned int j = 0 ; j < groupStateIndexList[gid].size() ; ++j)
	    fprintf(out, "%d ", groupStateIndexList[gid][j]);
	fprintf(out, "\n");
    }    
}
