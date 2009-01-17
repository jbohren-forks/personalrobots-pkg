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
    

void planning_models::KinematicModel::defaultState(void)
{
    /* The default state of the robot. Place each value at 0.0, if
       within bounds. Otherwise, select middle point. */
    double params[stateDimension];
    for (unsigned int i = 0 ; i < stateDimension ; ++i)
	if (stateBounds[2 * i] <= 0.0 && stateBounds[2 * i + 1] >= 0.0)
	    params[i] = 0.0;
	else
	    params[i] = (stateBounds[2 * i] + stateBounds[2 * i + 1]) / 2.0;
    
    computeTransforms(params);
}

void planning_models::KinematicModel::computeTransforms(const double *params, int groupID)
{
    assert(m_built);
    
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

void planning_models::KinematicModel::Robot::computeTransforms(const double *params, int groupID)
{
    chain->computeTransform(params, groupID);
}

void planning_models::KinematicModel::computeParameterNames(void)
{
    parameterNames.clear();
    
    for (unsigned int i = 0 ; i < m_robots.size(); ++i)
    {
	Joint *start =  m_robots[i]->chain;
	start->computeParameterNames(0);
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
    globalTransFwd  = before->before ? before->before->globalTransFwd : owner->rootTransform;
    globalTransFwd *= constTrans;
    globalTransFwd *= before->varTrans;
    
    for (unsigned int i = 0 ; i < after.size() ; ++i)
	params = after[i]->computeTransform(params, groupID);
    
    globalTrans.mult(globalTransFwd, constGeomTrans);
    
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
    // this will need to be updated to deal with 'ground plane'; right now all code assumes we move on the ground;
    // moving on a ramp will cause problems
    btVector3 newOrigin(btScalar(params[0]), btScalar(params[1]), btScalar(0.0));
    varTrans.setOrigin(newOrigin);
    btVector3 newAxis(btScalar(0.0), btScalar(0.0), btScalar(1.0));
    btQuaternion newQuat(newAxis, btScalar(params[2]));    
    varTrans.setRotation(newQuat);
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
    varTrans.setOrigin(btVector3(btScalar(params[0]), btScalar(params[1]), btScalar(params[2])));
    varTrans.setRotation(btQuaternion(btScalar(params[3]), btScalar(params[4]), btScalar(params[5]), btScalar(params[6])));    
}

void planning_models::KinematicModel::FloatingJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    robot->stateBounds.insert(robot->stateBounds.end(), 14, 0.0);
    robot->floatingJoints.push_back(robot->stateDimension);
}

void planning_models::KinematicModel::PrismaticJoint::updateVariableTransform(const double *params)
{
    btVector3 newOrigin = axis;
    newOrigin *= btScalar(params[0]);
    varTrans.setOrigin(newOrigin);
}

void planning_models::KinematicModel::PrismaticJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    axis.setX(btScalar(urdfLink->joint->axis[0]));
    axis.setY(btScalar(urdfLink->joint->axis[1]));
    axis.setZ(btScalar(urdfLink->joint->axis[2]));
        
    limit[0] = urdfLink->joint->limit[0] + urdfLink->joint->safetyLength[0];
    limit[1] = urdfLink->joint->limit[1] - urdfLink->joint->safetyLength[1];
    
    robot->stateBounds.push_back(limit[0]);
    robot->stateBounds.push_back(limit[1]);
}

void planning_models::KinematicModel::RevoluteJoint::updateVariableTransform(const double *params)
{
    btQuaternion newQuat(axis,  btScalar(params[0]));
    btQuaternion noRotation(btScalar(0.0), btScalar(0.0), btScalar(0.0), btScalar(1.0));
    
    btTransform  part1(noRotation, anchor);
    btTransform  part2(newQuat);
    btTransform  part3(noRotation, -anchor);
    
    varTrans.mult(part1, part2);
    varTrans *= part3;
}

void planning_models::KinematicModel::RevoluteJoint::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot)
{
    axis.setX(urdfLink->joint->axis[0]);
    axis.setY(urdfLink->joint->axis[1]);
    axis.setZ(urdfLink->joint->axis[2]);
    anchor.setX(urdfLink->joint->anchor[0]);
    anchor.setY(urdfLink->joint->anchor[1]);
    anchor.setZ(urdfLink->joint->anchor[2]);
    
    // if the joint is continuous, we set the limit to [-Pi, Pi]
    if (urdfLink->joint->isSet["limit"])
    {
	limit[0] = urdfLink->joint->limit[0] + urdfLink->joint->safetyLength[0];
	limit[1] = urdfLink->joint->limit[1] - urdfLink->joint->safetyLength[1];
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
    constTrans.setOrigin(btVector3(btScalar(xyz[0]), btScalar(xyz[1]), btScalar(xyz[2])));
    constTrans.setRotation(btQuaternion(btScalar(rpy[2]), btScalar(rpy[1]), btScalar(rpy[0])));  
    
    xyz = urdfLink->collision->xyz;
    rpy = urdfLink->collision->rpy;
    constGeomTrans.setOrigin(btVector3(btScalar(xyz[0]), btScalar(xyz[1]), btScalar(xyz[2])));
    constGeomTrans.setRotation(btQuaternion(btScalar(rpy[2]), btScalar(rpy[1]), btScalar(rpy[0])));  
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
	if (model.getGroup(allGroups[i])->hasFlag("planning") && !model.getGroup(allGroups[i])->links.empty())
	    m_groups.push_back(rname + "::" + allGroups[i]);
    m_groupsMap.clear();
    for (unsigned int i = 0 ; i < m_groups.size() ; ++i)
	m_groupsMap[m_groups[i]] = i;
}

unsigned int planning_models::KinematicModel::Joint::computeParameterNames(unsigned int pos)
{
    if (usedParams > 0)
    {
        owner->parameterNames[name] = pos;
	for (unsigned int i = 0 ; i < usedParams ; ++i)
	    owner->parameterValues[pos + i] = name;
    }
    return after->computeParameterNames(pos + usedParams);
}

unsigned int planning_models::KinematicModel::Link::computeParameterNames(unsigned int pos)
{
    for (unsigned int i = 0 ; i < after.size() ; ++i)
	pos = after[i]->computeParameterNames(pos);
    return pos;
}

bool planning_models::KinematicModel::isBuilt(void) const
{
    return m_built;
}

planning_models::KinematicModel::StateParams* planning_models::KinematicModel::newStateParams(void)
{
    return new StateParams(this);
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
    name = model.getRobotName();

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

	for (unsigned int j = 0 ; j < m_robots[i]->joints.size() ; ++j)
	    m_jointMap[m_robots[i]->joints[j]->name] = m_robots[i]->joints[j];
    }
    
    computeParameterNames();
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

std::string planning_models::KinematicModel::getURDFGroup(const std::string &group) const
{
    std::string urdfGroup = group;
    urdfGroup.erase(0, urdfGroup.find_last_of(":") + 1);
    return urdfGroup;
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


planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &joint) const
{
    std::map<std::string, Joint*>::const_iterator pos = m_jointMap.find(joint);
    return pos == m_jointMap.end() ? NULL : pos->second;
}

void planning_models::KinematicModel::getJoints(std::vector<Joint*> &joints) const
{
    std::vector<Joint*> localJoints;
    for (std::map<std::string, Joint*>::const_iterator it = m_jointMap.begin() ; it != m_jointMap.end() ; ++it)
	localJoints.push_back(it->second);
    std::sort(localJoints.begin(), localJoints.end(), SortByName<Joint>());
    joints.insert(joints.end(), localJoints.begin(), localJoints.end());  
}

void planning_models::KinematicModel::getJointsInGroup(std::vector<std::string> &names, int groupID) const
{
    std::vector<Joint*> joints;
    getJoints(joints);
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
	if (groupID < 0 || joints[i]->inGroup[groupID])
	    names.push_back(joints[i]->name);
}

void planning_models::KinematicModel::buildChainJ(Robot *robot, Link *parent, Joint* joint, const robot_desc::URDF::Link* urdfLink, const robot_desc::URDF &model)
{
    joint->before = parent;
    joint->after  = new Link();
    joint->name   = urdfLink->joint->name;
    joint->owner  = robot->owner;
    robot->joints.push_back(joint);

    joint->extractInformation(urdfLink, robot);
    
    /** construct the inGroup bitvector */
    std::vector<std::string> gnames;
    model.getGroupNames(gnames);
    for (unsigned int i = 0 ; i < gnames.size() ; ++i)
	if (model.getGroup(gnames[i])->hasFlag("planning"))
	    joint->inGroup.push_back(urdfLink->inGroup[i]);
    
    for (unsigned int i = 0 ; i < joint->inGroup.size() ; ++i)
	if (joint->inGroup[i])
	    for (unsigned int j = 0 ; j < joint->usedParams ; ++j)
		robot->groupStateIndexList[i].push_back(j + robot->stateDimension);
    
    for (unsigned int k = 0 ; k < urdfLink->groups.size() ; ++k)
	if (urdfLink->groups[k]->isRoot(urdfLink))
	{
	    std::string gname = name + "::" + urdfLink->groups[k]->name;
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

void planning_models::KinematicModel::StateParams::setParams(const double *params, const std::string &name)
{
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	unsigned int pos = m_pos[name];
	for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
	    m_params[pos + i] = params[i];
    }
    else
	std::cerr << "Unknown joint: '" << name << "'" << std::endl;
}
 
void planning_models::KinematicModel::StateParams::setParams(const double *params, int groupID)
{
    if (groupID < 0)
    {  
	for (unsigned int i = 0 ; i < m_dim ; ++i)
	    m_params[i] = params[i];
    }
    else
    {
	for (unsigned int i = 0 ; i < m_owner->groupStateIndexList[groupID].size() ; ++i)
	    m_params[m_owner->groupStateIndexList[groupID][i]] = params[i];
    }
}

void planning_models::KinematicModel::StateParams::setAll(const double value)
{
    for (unsigned int i = 0 ; i < m_dim ; ++i)
	m_params[i] = value;
}

const double* planning_models::KinematicModel::StateParams::getParams(void) const
{
    return m_params;
}

int planning_models::KinematicModel::StateParams::getPos(const std::string &name, int groupID) const
{
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	std::map<std::string, unsigned int>::const_iterator it = m_pos.find(name);
	if (it != m_pos.end())
	{
	    unsigned int pos = it->second; // position in the complete state vector
	    if (groupID < 0)
		return pos;
	    
	    for (unsigned int i = 0 ; i < m_owner->groupStateIndexList[groupID].size() ; ++i)
		if (m_owner->groupStateIndexList[groupID][i] == pos)
		    return i;
	    std::cerr << "Joint '" << name << "' is not in group " << groupID << std::endl; 
	}
    }
    std::cerr << "Unknown joint: '" << name << "'" << std::endl; 
    return -1;
}

void planning_models::KinematicModel::StateParams::copyParams(double *params, const std::string &name) const
{
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	std::map<std::string, unsigned int>::const_iterator it = m_pos.find(name);
	if (it != m_pos.end())
	{
	    unsigned int pos = it->second;
	    for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
		params[i] = m_params[pos + i];
	    return;
	}
    }
    std::cerr << "Unknown joint: '" << name << "'" << std::endl;
}

void planning_models::KinematicModel::StateParams::copyParams(double *params, int groupID) const
{
    if (groupID < 0)
    {  
	for (unsigned int i = 0 ; i < m_dim ; ++i)
	    params[i] = m_params[i];
    }
    else
    {
	for (unsigned int i = 0 ; i < m_owner->groupStateIndexList[groupID].size() ; ++i)
	    params[i] = m_params[m_owner->groupStateIndexList[groupID][i]];
    }
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) 
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
    for (unsigned int i = 0 ; i < floatingJoints.size() ; ++i)
	out << parameterValues[floatingJoints[i]] << " ";
    out << std::endl;
    
    out << "Planar joints at: ";
    for (unsigned int i = 0 ; i < planarJoints.size() ; ++i)
	out << planarJoints[i] << " ";
    for (unsigned int i = 0 ; i < planarJoints.size() ; ++i)
	out << parameterValues[planarJoints[i]] << " ";
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
	for (unsigned int j = 0 ; j < groupStateIndexList[gid].size() ; ++j)
	    out << parameterValues[groupStateIndexList[gid][j]] << " ";
	out << std::endl;
    }
}

void planning_models::KinematicModel::printLinkPoses(std::ostream &out) const
{
    std::vector<Link*> links;
    getLinks(links);
    for (unsigned int i = 0 ; i < links.size() ; ++i)
	out << links[i]->name << std::endl;
}

void planning_models::KinematicModel::StateParams::print(std::ostream &out)
{
    out << std::endl;
    for (std::map<std::string, unsigned int>::const_iterator it = m_pos.begin() ; it != m_pos.end() ; ++it)
    {
	Joint* joint = m_owner->getJoint(it->first);
	if (joint)
	{
	    for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
		out << it->first << ": " << m_params[it->second + i] << std::endl;
	}
    }
    out << std::endl;
    for (unsigned int i = 0; i < m_dim ; ++i)
	out << m_params[i] << " ";
    out << std::endl;
}
