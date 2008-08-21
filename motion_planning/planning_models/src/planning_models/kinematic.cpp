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
#include <algorithm>
#include <cmath>

/** Operator for sorting objects by name */
template<typename T>
struct SortByName
{
    bool operator()(const T *a, const T *b) const
    {
	return a->name < b->name;
    }
};
    
void planning_models::KinematicModel::Robot::computeTransforms(const double *params, int groupID)
{
    chain->computeTransform(params, groupID);
}

void planning_models::KinematicModel::computeTransforms(const double *params, int groupID)
{
    if (groupID >= 0)
    {
	for (unsigned int i = 0 ; i < groupChainStart[groupID].size() ; ++i)
	{
	    Joint *start = groupChainStart[groupID][i];
	    params = start->computeTransform(params, groupID);
	}
    }
    else
    {  
	for (unsigned int i = 0 ; i < m_robots.size(); ++i)
	{
	    Joint *start =  m_robots[i]->chain;
	    params = start->computeTransform(params, groupID);
	}
    }
}

const double* planning_models::KinematicModel::Joint::computeTransform(const double *params, int groupID)
{
    unsigned int used = 0;
    
    if (groupID < 0 || inGroup[groupID])
    {
	updateVariableTransform(params);
	used = usedParams;
    }
    
    return after->computeTransform(params + used, groupID);
}

const double* planning_models::KinematicModel::Link::computeTransform(const double *params, int groupID)
{
    globalTransFwd = before->before ? before->before->globalTransFwd : owner->rootTransform;
    globalTransFwd.multiplyPose(constTrans);
    globalTransFwd.multiplyPose(before->varTrans);
    
    for (unsigned int i = 0 ; i < after.size() ; ++i)
	params = after[i]->computeTransform(params, groupID);
    
    globalTrans = globalTransFwd;
    globalTrans.multiplyPose(constGeomTrans);
    
    return params;
}

void planning_models::KinematicModel::FixedJoint::updateVariableTransform(const double *params)
{
    // the joint remains identity
}

void planning_models::KinematicModel::FixedJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    // we need no data
}

void planning_models::KinematicModel::PlanarJoint::updateVariableTransform(const double *params)
{
    varTrans.setPosition(params[0], params[1], 0.0);
    varTrans.setAxisAngle(0.0, 0.0, 1.0, params[2]);
}

void planning_models::KinematicModel::PlanarJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    robot->stateBounds.insert(robot->stateBounds.end(), 4, 0.0);
    robot->stateBounds.push_back(-M_PI);
    robot->stateBounds.push_back(M_PI);
    robot->planarJoints.push_back(robot->stateDimension);
}

void planning_models::KinematicModel::FloatingJoint::updateVariableTransform(const double *params)
{
    varTrans.setPosition(params[0], params[1], params[2]);
    varTrans.setQuaternion(params[3], params[4], params[5], params[6]);
}

void planning_models::KinematicModel::FloatingJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    robot->stateBounds.insert(robot->stateBounds.end(), 14, 0.0);
    robot->floatingJoints.push_back(robot->stateDimension);
}

void planning_models::KinematicModel::PrismaticJoint::updateVariableTransform(const double *params)
{
    double p  = params[0];
    double dx = axis[0] * p;
    double dy = axis[1] * p;
    double dz = axis[2] * p;
    varTrans.setPosition(dx, dy, dz);
}

void planning_models::KinematicModel::PrismaticJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    axis[0]  = urdfLink->joint->axis[0];
    axis[1]  = urdfLink->joint->axis[1];
    axis[2]  = urdfLink->joint->axis[2];
    limit[0] = urdfLink->joint->limit[0];
    limit[1] = urdfLink->joint->limit[1];
    
    robot->stateBounds.push_back(limit[0]);
    robot->stateBounds.push_back(limit[1]);
}

void planning_models::KinematicModel::RevoluteJoint::updateVariableTransform(const double *params)
{
    varTrans.setAxisAngle(axis, params[0]);
    // anchor is ignored here; the rotation is assumed to be about
    // (0,0,0) but it should be about anchor; when this exists in the
    // transforms library, it should be switched here too
}

void planning_models::KinematicModel::RevoluteJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    axis[0] = urdfLink->joint->axis[0];
    axis[1] = urdfLink->joint->axis[1];
    axis[2] = urdfLink->joint->axis[2];
    anchor[0] = urdfLink->joint->anchor[0];
    anchor[1] = urdfLink->joint->anchor[1];
    anchor[2] = urdfLink->joint->anchor[2];
    
    // if the joint is continuous, we set the limit to [-Pi, Pi]
    if (urdfLink->joint->isSet["limit"])
    {
	limit[0] = urdfLink->joint->limit[0];
	limit[1] = urdfLink->joint->limit[1];
    }
    else
    {
	limit[0] = -M_PI;
	limit[1] =  M_PI;
    }
    
    robot->stateBounds.push_back(limit[0]);
    robot->stateBounds.push_back(limit[1]);
}

void planning_models::KinematicModel::Link::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    /* compute the geometry for this link */
    switch (urdfLink->collision->geometry->type)
    {
    case robot_desc::URDF::Link::Geometry::BOX:
	{
	    Box          *box  = new Box();	    
	    const double *size = static_cast<const robot_desc::URDF::Link::Geometry::Box*>(urdfLink->collision->geometry->shape)->size;
	    box->size[0] = size[0];
	    box->size[1] = size[1];
	    box->size[2] = size[2];
	    shape        = box;
	}
	break;
    case robot_desc::URDF::Link::Geometry::SPHERE:
	{
	    Sphere *sphere = new Sphere();
	    sphere->radius = static_cast<const robot_desc::URDF::Link::Geometry::Sphere*>(urdfLink->collision->geometry->shape)->radius;
	    shape          = sphere;
	}
	break;
    case robot_desc::URDF::Link::Geometry::CYLINDER:
	{
	    Cylinder *cylinder = new Cylinder();
	    cylinder->length = static_cast<const robot_desc::URDF::Link::Geometry::Cylinder*>(urdfLink->collision->geometry->shape)->length;
	    cylinder->radius = static_cast<const robot_desc::URDF::Link::Geometry::Cylinder*>(urdfLink->collision->geometry->shape)->radius;
	    shape            = cylinder;
	}	
	break;
    default:
	break;
    }
    
    /* compute the constant transform for this link */
    const double *xyz = urdfLink->xyz;
    const double *rpy = urdfLink->rpy;
    constTrans.setFromEuler(xyz[0], xyz[1], xyz[2], rpy[2], rpy[1], rpy[0]);	    
    
    xyz = urdfLink->collision->xyz;
    rpy = urdfLink->collision->rpy;
    constGeomTrans.setFromEuler(xyz[0], xyz[1], xyz[2], rpy[2], rpy[1], rpy[0]);    
}

void planning_models::KinematicModel::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

void planning_models::KinematicModel::constructGroupList(const robot_desc::URDF &model)
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

void planning_models::KinematicModel::build(const robot_desc::URDF &model, bool ignoreSensors)
{
    if (m_built)
    {
	std::cerr << "Model has already been built!" << std::endl;
	return;
    }
    
    m_built = true;
    m_ignoreSensors = ignoreSensors;
    
    /* construct a map for the available groups */
    constructGroupList(model);
    groupStateIndexList.resize(m_groups.size());
    groupChainStart.resize(m_groups.size());
    
    for (unsigned int i = 0 ; i < model.getDisjointPartCount() ; ++i)
    {
	const robot_desc::URDF::Link *link = model.getDisjointPart(i);
	if (link->canSense() && m_ignoreSensors)
	    continue;
	Robot                  *rb   = new Robot(this);
	rb->name = model.getRobotName();
	rb->groupStateIndexList.resize(m_groups.size());
	rb->groupChainStart.resize(m_groups.size());
	rb->chain = createJoint(link);
	buildChainJ(rb, NULL, rb->chain, link, model);
	m_robots.push_back(rb);
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
	
	for (unsigned int j = 0 ; j < m_robots[i]->links.size() ; ++j)
	    m_linkMap[m_robots[i]->links[j]->name] = m_robots[i]->links[j];
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

void planning_models::KinematicModel::getLinks(std::vector<Link*> &links) const
{
    std::vector<Link*> localLinks;
    for (std::map<std::string, Link*>::const_iterator it = m_linkMap.begin() ; it != m_linkMap.end() ; ++it)
	localLinks.push_back(it->second);
    std::sort(localLinks.begin(), localLinks.end(), SortByName<Link>());
    links.insert(links.end(), localLinks.begin(), localLinks.end());
}

planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &link) const
{
    std::map<std::string, Link*>::const_iterator pos = m_linkMap.find(link);
    return pos == m_linkMap.end() ? NULL : pos->second;
}

void planning_models::KinematicModel::buildChainJ(Robot *robot, Link *parent, Joint* joint, const robot_desc::URDF::Link* urdfLink, const robot_desc::URDF &model)
{
    joint->before = parent;
    joint->after  = new Link();
    joint->name   = urdfLink->joint->name;
    joint->owner  = robot->owner;
    
    joint->extractInformation(urdfLink, robot);
    
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
	    std::string gname = robot->name + "::" + urdfLink->groups[k]->name;
	    if (m_groupsMap.find(gname) != m_groupsMap.end())
		robot->groupChainStart[m_groupsMap[gname]].push_back(joint);
	}
    
    if (m_verbose && joint->usedParams > 0)
    {
	std::cout << "Joint '" << urdfLink->joint->name << "' connects link '" << urdfLink->parentName << "' to link '" << 
	    urdfLink->name << "' and uses state coordinates: ";
	for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
	    std::cout << i + robot->stateDimension << " ";
	std::cout << std::endl;
    }
    
    robot->stateDimension += joint->usedParams;
    buildChainL(robot, joint, joint->after, urdfLink, model);
}

void planning_models::KinematicModel::buildChainL(Robot *robot, Joint *parent, Link* link, const robot_desc::URDF::Link* urdfLink, const robot_desc::URDF &model)
{
    link->name   = urdfLink->name;
    link->before = parent;
    link->owner  = robot->owner;
    robot->links.push_back(link);
    
    link->extractInformation(urdfLink, robot);
    
    for (unsigned int i = 0 ; i < urdfLink->children.size() ; ++i)
    {
	if (urdfLink->children[i]->canSense() && m_ignoreSensors)
	    continue;
	Joint *newJoint = createJoint(urdfLink->children[i]);
	buildChainJ(robot, link, newJoint, urdfLink->children[i], model);
	link->after.push_back(newJoint);
    }
    
    if (link->after.size() == 0)
	robot->leafs.push_back(link);
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::createJoint(const robot_desc::URDF::Link* urdfLink)
{
    Joint *newJoint = NULL;
    switch (urdfLink->joint->type)
    {
    case robot_desc::URDF::Link::Joint::FIXED:
	newJoint = new FixedJoint();
	break;	    
    case robot_desc::URDF::Link::Joint::FLOATING:
	newJoint = new FloatingJoint();
	break;	    
    case robot_desc::URDF::Link::Joint::PLANAR:
	newJoint = new PlanarJoint();
	break;	    
    case robot_desc::URDF::Link::Joint::PRISMATIC:
	newJoint = new PrismaticJoint();
	break;
    case robot_desc::URDF::Link::Joint::REVOLUTE:
	newJoint = new RevoluteJoint();
	break;
    default:
	std::cerr << "Unknown joint type " << urdfLink->joint->type << std::endl;
	break;
    }  
    return newJoint;
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) const
{   
    out << "Number of robots = " << getRobotCount() << std::endl;
    out << "Complete model state dimension = " << stateDimension << std::endl;
    
    out << "State bounds: ";
    for (unsigned int i = 0 ; i < stateDimension ; ++i)
	out << "[" << stateBounds[2 * i] << ", " << stateBounds[2 * i + 1] << "] ";
    out << std::endl;
    
    out << "Floating joints at: ";
    for (unsigned int i = 0 ; i < floatingJoints.size() ; ++i)
	out << floatingJoints[i] << " ";
    out << std::endl;
    
    out << "Planar joints at: ";
    for (unsigned int i = 0 ; i < planarJoints.size() ; ++i)
	out << planarJoints[i] << " ";
    out << std::endl;
    
    out << "Available groups: ";
    std::vector<std::string> l;
    getGroups(l);
    for (unsigned int i = 0 ; i < l.size() ; ++i)
	out << l[i] << " ";
    out << std::endl;
    
    for (unsigned int i = 0 ; i < l.size() ; ++i)
    {
	int gid = getGroupID(l[i]);
	out << "Group " << l[i] << " has " << groupChainStart[gid].size() << " roots" << std::endl;
	out << "The state components for this group are: ";
	for (unsigned int j = 0 ; j < groupStateIndexList[gid].size() ; ++j)
	    out << groupStateIndexList[gid][j] << " ";
	out << std::endl;
    }
}

void planning_models::KinematicModel::printLinkPoses(std::ostream &out) const
{
    std::vector<Link*> links;
    getLinks(links);
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	out << links[i]->name << std::endl;
	out << links[i]->globalTrans << std::endl;
    }
}
