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
    
const std::string& planning_models::KinematicModel::getModelName(void) const
{
    return m_name;
}

const planning_models::KinematicModel::ModelInfo& planning_models::KinematicModel::getModelInfo(void) const
{
    return m_mi;
}

planning_models::KinematicModel::ModelInfo& planning_models::KinematicModel::getModelInfo(void)
{
    return m_mi;
}

void planning_models::KinematicModel::defaultState(void)
{
    /* The default state of the robot. Place each value at 0.0, if
       within bounds. Otherwise, select middle point. */
    double params[m_mi.stateDimension];
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	if (m_mi.stateBounds[2 * i] <= 0.0 && m_mi.stateBounds[2 * i + 1] >= 0.0)
	    params[i] = 0.0;
	else
	    params[i] = (m_mi.stateBounds[2 * i] + m_mi.stateBounds[2 * i + 1]) / 2.0;
    m_lastTransformGroup = -2;
    computeTransformsGroup(params, -1);
}

void planning_models::KinematicModel::computeTransforms(const double *params)
{
    computeTransformsGroup(params, -1);
}

void planning_models::KinematicModel::computeTransformsGroup(const double *params, int groupID)
{
    assert(m_built);
    
    const unsigned int gdim = getGroupDimension(groupID);
    if (m_lastTransformGroup == groupID)
    {
	bool same = true;
	for (unsigned int i = 0 ; i < gdim ; ++i)
	    if (params[i] != m_lastTransformParams[i])
	    {
		same = false;
		break;
	    }
	if (same)
	    return;
    }
    m_lastTransformGroup = groupID;
    memcpy(m_lastTransformParams, params,  gdim * sizeof(double));
    
    if (groupID >= 0)
    {
	for (unsigned int i = 0 ; i < m_mi.groupChainStart[groupID].size() ; ++i)
	{
	    Joint *start = m_mi.groupChainStart[groupID][i];
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

void planning_models::KinematicModel::computeParameterNames(void)
{
    m_mi.parameterIndex.clear();
    unsigned int pos = 0;
    for (unsigned int i = 0 ; i < m_robots.size(); ++i)
    {
	Joint *start =  m_robots[i]->chain;
	start->computeParameterNames(pos);
	pos += m_robots[i]->stateDimension;	
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
    
    for (unsigned int i = 0 ; i < attachedBodies.size() ; ++i)
	attachedBodies[i]->computeTransform(globalTrans);
    
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
        owner->owner->getModelInfo().parameterIndex[name] = pos;
	for (unsigned int i = 0 ; i < usedParams ; ++i)
	    owner->owner->getModelInfo().parameterName[pos + i] = name;
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
	
bool planning_models::KinematicModel::Robot::reduceToRobotFrame(void)
{
    if (floatingJoints.size() + planarJoints.size() == 1)
    {
	if (planarJoints.size())
	    rootTransform *= joints[planarJoints[0]]->after->constTrans.inverse();
	else
	    rootTransform *= joints[floatingJoints[0]]->after->constTrans.inverse();
	return true;
    }
    else
	return false;    
}

bool planning_models::KinematicModel::reduceToRobotFrame(void)
{
    bool result = true;
    for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
	result = result && m_robots[i]->reduceToRobotFrame();
    defaultState();
    return result;
}

void planning_models::KinematicModel::build(const std::string &description, bool ignoreSensors)
{	    
    robot_desc::URDF *file = new robot_desc::URDF();
    file->loadString(description.c_str());
    build(*file, ignoreSensors);
    delete file;
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
    m_name = model.getRobotName();

    /* construct a map for the available groups */
    constructGroupList(model);
    m_mi.groupStateIndexList.resize(m_groups.size());
    m_mi.groupChainStart.resize(m_groups.size());
    
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
	m_mi.stateBounds.insert(m_mi.stateBounds.end(), m_robots[i]->stateBounds.begin(), m_robots[i]->stateBounds.end());
	
	/* copy floating joints*/
	for (unsigned int j = 0 ; j < m_robots[i]->floatingJoints.size() ; ++j)
	    m_mi.floatingJoints.push_back(m_mi.stateDimension + m_robots[i]->floatingJoints[j]);

	/* copy planar joints*/
	for (unsigned int j = 0 ; j < m_robots[i]->planarJoints.size() ; ++j)
	    m_mi.planarJoints.push_back(m_mi.stateDimension + m_robots[i]->planarJoints[j]);
	
	/* copy group roots */
	for (unsigned int j = 0 ; j < m_robots[i]->groupChainStart.size() ; ++j)
	    for (unsigned int k = 0 ; k < m_robots[i]->groupChainStart[j].size() ; ++k)
		m_mi.groupChainStart[j].push_back(m_robots[i]->groupChainStart[j][k]);
	
	/* copy state index list */
	for (unsigned int j = 0 ; j < m_robots[i]->groupStateIndexList.size() ; ++j)
	    for (unsigned int k = 0 ; k < m_robots[i]->groupStateIndexList[j].size() ; ++k)
		m_mi.groupStateIndexList[j].push_back(m_mi.stateDimension + m_robots[i]->groupStateIndexList[j][k]);
	
	m_mi.stateDimension += m_robots[i]->stateDimension;
	
	for (unsigned int j = 0 ; j < m_robots[i]->links.size() ; ++j)
	    m_linkMap[m_robots[i]->links[j]->name] = m_robots[i]->links[j];

	for (unsigned int j = 0 ; j < m_robots[i]->joints.size() ; ++j)
	    m_jointMap[m_robots[i]->joints[j]->name] = m_robots[i]->joints[j];
    }

    m_lastTransformParams = new double[m_mi.stateDimension];
    
    computeParameterNames();
    defaultState();
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

unsigned int planning_models::KinematicModel::getGroupDimension(int groupID) const
{
    return groupID >= 0 ? m_mi.groupStateIndexList[groupID].size() : m_mi.stateDimension;
}

unsigned int planning_models::KinematicModel::getGroupDimension(const std::string &group) const
{
    return getGroupDimension(getGroupID(group));
}

void planning_models::KinematicModel::getJointsInGroup(std::vector<std::string> &names, const std::string &group) const
{
    getJointsInGroup(names, getGroupID(group));
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
    joint->owner  = robot;
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
	    std::string gname = m_name + "::" + urdfLink->groups[k]->name;
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
    link->owner  = robot;
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

void planning_models::KinematicModel::AttachedBody::computeTransform(btTransform &parentTrans)
{
    globalTrans = attachTrans * parentTrans;
}

void planning_models::KinematicModel::StateParams::reset(void)
{
    resetGroup(-1);
}

void planning_models::KinematicModel::StateParams::resetGroup(const std::string &group)
{
    resetGroup(m_owner->getGroupID(group));
}

void planning_models::KinematicModel::StateParams::resetGroup(int groupID)
{
    if (groupID < 0)
    { 
	for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	    m_seen[i] = false;
    }
    else
    {
	for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	{
	    unsigned int j = m_mi.groupStateIndexList[groupID][i];
	    m_seen[j] = false;
	}
    }
}

bool planning_models::KinematicModel::StateParams::seenAll(void) const
{
    return seenAllGroup(-1);
}

bool planning_models::KinematicModel::StateParams::seenAllGroup(const std::string &group) const
{
    return seenAllGroup(m_owner->getGroupID(group));
}

bool planning_models::KinematicModel::StateParams::seenAllGroup(int groupID) const
{
    if (groupID < 0)
    {
	for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	{
	    std::map<unsigned int, bool>::const_iterator it = m_seen.find(i);
	    if (!it->second)
		return false;
	}
    }
    else
    {
	for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	{
	    unsigned int j = m_mi.groupStateIndexList[groupID][i];
	    std::map<unsigned int, bool>::const_iterator it = m_seen.find(j);
	    if (!it->second)
		return false;
	}
    }
    return true;
}

void planning_models::KinematicModel::StateParams::missing(int groupID, std::ostream &out)
{
    if (groupID < 0)
    {
	for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	    if (!m_seen[i])
		out << m_mi.parameterName[i] << " ";
    }
    else
    {
	for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	{
	    unsigned int j = m_mi.groupStateIndexList[groupID][i];
	    if (!m_seen[j])
		out << m_mi.parameterName[j] << " ";
	}
    }
}

const double* planning_models::KinematicModel::StateParams::getParamsJoint(const std::string &name) const
{
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
	if (it != m_mi.parameterIndex.end())
	    return m_params + it->second;
	else
	    return NULL;
    }
    else
	return NULL;    
}

bool planning_models::KinematicModel::StateParams::setParamsJoint(const std::vector<double> &params, const std::string &name)
{
    bool result = false;
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	double *dparams = new double[joint->usedParams];
	result = setParamsJoint(dparams, name);
	delete[] dparams;
    }
    return result;
}

bool planning_models::KinematicModel::StateParams::setParamsJoint(const double *params, const std::string &name)
{
    bool result = false;
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	unsigned int pos = m_mi.parameterIndex[name];
	for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
	{
	    unsigned int pos_i = pos + i;
	    if (m_params[pos_i] != params[i] || !m_seen[pos_i])
	    {
		m_params[pos_i] = params[i];
		m_seen[pos_i] = true;		
		result = true;
	    }
	}
    }
    else
	std::cerr << "Unknown joint: '" << name << "'" << std::endl;
    return result;
}

bool planning_models::KinematicModel::StateParams::setParams(const std::vector<double> &params)
{
    return setParamsGroup(params, -1);
}

bool planning_models::KinematicModel::StateParams::setParams(const double *params)
{
    return setParamsGroup(params, -1);
}

bool planning_models::KinematicModel::StateParams::setParamsGroup(const std::vector<double> &params, const std::string &group)
{
    return setParamsGroup(params, m_owner->getGroupID(group));
}

bool planning_models::KinematicModel::StateParams::setParamsGroup(const std::vector<double> &params, int groupID)
{
    double *dparams = new double[m_owner->getGroupDimension(groupID)];
    for (unsigned int i = 0 ; i < params.size() ; ++i)
	dparams[i] = params[i];
    bool result = setParamsGroup(dparams, groupID);
    delete[] dparams;
    return result;
}

bool planning_models::KinematicModel::StateParams::setParamsGroup(const double *params, const std::string &group)
{
    return setParamsGroup(params, m_owner->getGroupID(group));
}

bool planning_models::KinematicModel::StateParams::setParamsGroup(const double *params, int groupID)
{
    bool result = false;
    if (groupID < 0)
    {  
	for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	    if (m_params[i] != params[i] || !m_seen[i])
	    {
		m_params[i] = params[i];
		m_seen[i] = true;
		result = true;
	    }
    }
    else
    {
	for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	{
	    unsigned int j = m_mi.groupStateIndexList[groupID][i];
	    if (m_params[j] != params[i] || !m_seen[j])
	    {
		m_params[j] = params[i];
		m_seen[j] = true;
		result = true;
	    }
	}
    }
    return result;
}

void planning_models::KinematicModel::StateParams::setAllInGroup(const double value, const std::string &group)
{
    setAllInGroup(value, m_owner->getGroupID(group));
}

void planning_models::KinematicModel::StateParams::setAllInGroup(const double value, int groupID)
{
    if (groupID < 0)
    {
	for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	{
	    m_params[i] = value;
	    m_seen[i] = true;
	}    
    }
    else
    {
	for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	{
	    unsigned int j = m_mi.groupStateIndexList[groupID][i];
	    m_params[j] = value;
	    m_seen[j] = true;
	}	
    }
}

void planning_models::KinematicModel::StateParams::setAll(const double value)
{
    setAllInGroup(value, -1);
}

void planning_models::KinematicModel::StateParams::setInRobotFrame(void)
{
    for (unsigned int j = 0 ; j < m_mi.floatingJoints.size() ; ++j)
    {
	double vals[7] = {0, 0, 0, 0, 0, 0, 1};
	setParamsJoint(vals, m_mi.parameterName[m_mi.floatingJoints[j]]);
    }
    
    for (unsigned int j = 0 ; j < m_mi.planarJoints.size() ; ++j)
    {
	double vals[3] = {0, 0, 0};
	setParamsJoint(vals, m_mi.parameterName[m_mi.planarJoints[j]]);
    }
}

const double* planning_models::KinematicModel::StateParams::getParams(void) const
{
    return m_params;
}

int planning_models::KinematicModel::StateParams::getJointIndexInGroup(const std::string &name, const std::string &group) const
{
    return getJointIndexInGroup(name, m_owner->getGroupID(group));
}

int planning_models::KinematicModel::StateParams::getJointIndexInGroup(const std::string &name, int groupID) const
{
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
	if (it != m_mi.parameterIndex.end())
	{
	    unsigned int pos = it->second; // position in the complete state vector
	    if (groupID < 0)
		return pos;
	    
	    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
		if (m_mi.groupStateIndexList[groupID][i] == pos)
		    return i;
	    std::cerr << "Joint '" << name << "' is not in group " << groupID << std::endl; 
	}
    }
    std::cerr << "Unknown joint: '" << name << "'" << std::endl; 
    return -1;
}

void planning_models::KinematicModel::StateParams::copyParamsJoint(double *params, const std::string &name) const
{
    Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
	if (it != m_mi.parameterIndex.end())
	{
	    unsigned int pos = it->second;
	    for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
		params[i] = m_params[pos + i];
	    return;
	}
    }
    std::cerr << "Unknown joint: '" << name << "'" << std::endl;
}

void planning_models::KinematicModel::StateParams::copyParams(double *params) const
{
    copyParamsGroup(params, -1);
}

void planning_models::KinematicModel::StateParams::copyParams(std::vector<double> &params) const
{
    copyParamsGroup(params, -1);
}

void planning_models::KinematicModel::StateParams::copyParamsGroup(double *params, const std::string &group) const
{
    copyParamsGroup(params, m_owner->getGroupID(group));
}

void planning_models::KinematicModel::StateParams::copyParamsGroup(std::vector<double> &params, const std::string &group) const
{
    copyParamsGroup(params, m_owner->getGroupID(group));
}

void planning_models::KinematicModel::StateParams::copyParamsGroup(std::vector<double> &params, int groupID) const
{ 
    unsigned int dim = m_owner->getGroupDimension(groupID);
    double *dparams = new double[dim];
    copyParamsGroup(dparams, groupID);
    params.resize(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	params[i] = dparams[i];
    delete[] dparams;
}

void planning_models::KinematicModel::StateParams::copyParamsGroup(double *params, int groupID) const
{
    if (groupID < 0)
    {  
	for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	    params[i] = m_params[i];
    }
    else
    {
	for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	    params[i] = m_params[m_mi.groupStateIndexList[groupID][i]];
    }
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) 
{   
    out << "Number of robots = " << getRobotCount() << std::endl;
    out << "Complete model state dimension = " << m_mi.stateDimension << std::endl;
    
    std::ios_base::fmtflags old_flags = out.flags();    
    out.setf(std::ios::fixed, std::ios::floatfield);
    std::streamsize old_prec = out.precision();
    out.precision(5);
    out << "State bounds: ";
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	out << "[" << m_mi.stateBounds[2 * i] << ", " << m_mi.stateBounds[2 * i + 1] << "] ";
    out << std::endl;
    out.precision(old_prec);    
    out.flags(old_flags);
    
    out << "Parameter index:" << std::endl;
    for (std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.begin() ; 
	 it != m_mi.parameterIndex.end() ; ++it)
	out << it->first << " = " << it->second << std::endl;

    out << "Parameter name:" << std::endl;
    for (std::map<unsigned int, std::string>::const_iterator it = m_mi.parameterName.begin() ; 
	 it != m_mi.parameterName.end() ; ++it)
	out << it->first << " = " << it->second << std::endl;
    
    out << "Floating joints at: ";
    for (unsigned int i = 0 ; i < m_mi.floatingJoints.size() ; ++i)
	out << m_mi.floatingJoints[i] << " ";
    for (unsigned int i = 0 ; i < m_mi.floatingJoints.size() ; ++i)
	out << m_mi.parameterName[m_mi.floatingJoints[i]] << " ";
    out << std::endl;
    
    out << "Planar joints at: ";
    for (unsigned int i = 0 ; i < m_mi.planarJoints.size() ; ++i)
	out << m_mi.planarJoints[i] << " ";
    for (unsigned int i = 0 ; i < m_mi.planarJoints.size() ; ++i)
	out << m_mi.parameterName[m_mi.planarJoints[i]] << " ";
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
	out << "Group " << l[i] << " with ID " << gid << " has " << m_mi.groupChainStart[gid].size() << " roots: ";
	for (unsigned int j = 0 ; j < m_mi.groupChainStart[gid].size() ; ++j)
	    out << m_mi.groupChainStart[gid][j]->name << " ";
	out << std::endl;
	out << "The state components for this group are: ";
	for (unsigned int j = 0 ; j < m_mi.groupStateIndexList[gid].size() ; ++j)
	    out << m_mi.groupStateIndexList[gid][j] << " ";
	for (unsigned int j = 0 ; j < m_mi.groupStateIndexList[gid].size() ; ++j)
	    out << m_mi.parameterName[m_mi.groupStateIndexList[gid][j]] << " ";
	out << std::endl;
    }
}

void planning_models::KinematicModel::printLinkPoses(std::ostream &out) const
{
    out << "Link poses:" << std::endl;
    std::vector<Link*> links;
    getLinks(links);
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	out << links[i]->name << std::endl;
	const btVector3 &v = links[i]->globalTrans.getOrigin();
	out << "  origin: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
	const btQuaternion &q = links[i]->globalTrans.getRotation();
	out << "  quaternion: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
	out << std::endl;
    }    
}

void planning_models::KinematicModel::StateParams::print(std::ostream &out) const
{
    out << std::endl;
    for (std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.begin() ; it != m_mi.parameterIndex.end() ; ++it)
    {
	Joint* joint = m_owner->getJoint(it->first);
	if (joint)
	{
	    out << it->first;
	    std::map<unsigned int, bool>::const_iterator sit = m_seen.find(it->second);
	    if (!sit->second)
		out << "[ *** UNSEEN *** ]";
	    out << ": ";
	    for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
		out << m_params[it->second + i] << std::endl;
	}
    }
    out << std::endl;
    for (unsigned int i = 0; i < m_mi.stateDimension ; ++i)
	out << m_params[i] << " ";
    out << std::endl;
}
