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
#include <planning_models/kinematic_state_params.h>
#include <cassert>
#include <algorithm>
#include <sstream>
#include <cmath>

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
    StateParams sp(this);
    sp.defaultState();
    computeTransforms(sp.getParams());
}

void planning_models::KinematicModel::computeTransforms(const double *params)
{
    for (unsigned int i = 0 ; i < m_robots.size(); ++i)
    {
	Joint *start =  m_robots[i]->chain;
	params = start->computeTransform(params, -1);
    }
}

void planning_models::KinematicModel::computeTransformsGroup(const double *params, int groupID)
{
    assert(m_built);
    assert(groupID >= 0 && groupID < (int)m_groups.size());
    
    for (unsigned int i = 0 ; i < m_mi.groupChainStart[groupID].size() ; ++i)
    {
	Joint *start = m_mi.groupChainStart[groupID][i];
	params = start->computeTransform(params, groupID);
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
    btVector3 newOrigin((btScalar)params[0], (btScalar)params[1], btScalar(0.0));
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
	continuous = false;       
    }
    else
    {
	continuous = true;
	limit[0] = -M_PI;
	limit[1] =  M_PI;
    }
    
    robot->stateBounds.push_back(limit[0]);
    robot->stateBounds.push_back(limit[1]);
}

void planning_models::KinematicModel::Link::extractInformation(const robot_desc::URDF::Link *urdfLink, Robot *robot, const robot_desc::URDF &model)
{
    /* compute the geometry for this link */
    switch (urdfLink->collision->geometry->type)
    {
    case robot_desc::URDF::Link::Geometry::BOX:
	{
	    shapes::Box  *box  = new shapes::Box();	    
	    const double *size = static_cast<const robot_desc::URDF::Link::Geometry::Box*>(urdfLink->collision->geometry->shape)->size;
	    box->size[0] = size[0];
	    box->size[1] = size[1];
	    box->size[2] = size[2];
	    shape        = box;
	}
	break;
    case robot_desc::URDF::Link::Geometry::SPHERE:
	{
	    shapes::Sphere *sphere = new shapes::Sphere();
	    sphere->radius         = static_cast<const robot_desc::URDF::Link::Geometry::Sphere*>(urdfLink->collision->geometry->shape)->radius;
	    shape                  = sphere;
	}
	break;
    case robot_desc::URDF::Link::Geometry::CYLINDER:
	{
	    shapes::Cylinder *cylinder = new shapes::Cylinder();
	    cylinder->length           = static_cast<const robot_desc::URDF::Link::Geometry::Cylinder*>(urdfLink->collision->geometry->shape)->length;
	    cylinder->radius           = static_cast<const robot_desc::URDF::Link::Geometry::Cylinder*>(urdfLink->collision->geometry->shape)->radius;
	    shape                      = cylinder;
	}
	break;
    case robot_desc::URDF::Link::Geometry::MESH:
	{
	    std::string filename = model.getResourceLocation() + "/" + static_cast<const robot_desc::URDF::Link::Geometry::Mesh*>(urdfLink->collision->geometry->shape)->filename;
	    if (filename.rfind(".stl") == std::string::npos)
		filename += ".stl";
	    std::cout << "Loading '" << filename << "'" << std::endl;	    
	    shapes::Mesh *mesh = shapes::create_mesh_from_binary_stl(filename.c_str());
	    shape              = mesh;
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

void planning_models::KinematicModel::constructGroupList(void)
{
    m_groups.clear();
    m_groupsMap.clear();

    for (std::map< std::string, std::vector<std::string> >::const_iterator it = m_groupContent.begin() ; it != m_groupContent.end() ; ++it)
	m_groups.push_back(it->first);
    
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

planning_models::StateParams* planning_models::KinematicModel::newStateParams(void)
{
    return new StateParams(this);
}

void planning_models::KinematicModel::reduceToRobotFrame(void)
{
    if (!m_mi.inRobotFrame)
    {
	for (unsigned int i = 0 ; i < m_robots.size() ; ++i)
	    if (dynamic_cast<PlanarJoint*>(m_robots[i]->chain) || dynamic_cast<FloatingJoint*>(m_robots[i]->chain))
		m_robots[i]->rootTransform *= m_robots[i]->chain->after->constTrans.inverse();
	
	m_mi.inRobotFrame = true;
    }
    defaultState();
}

void planning_models::KinematicModel::build(const std::string &description, const std::map< std::string, std::vector<std::string> > &groups)
{	    
    robot_desc::URDF *file = new robot_desc::URDF();
    file->loadString(description.c_str());
    build(*file, groups);
    delete file;
}

void planning_models::KinematicModel::build(const robot_desc::URDF &model, const std::map< std::string, std::vector<std::string> > &groups)
{
    if (m_built)
    {
	m_msg.error("Model has already been built!");
	return;
    }
    
    m_built = true;
    m_name = model.getRobotName();

    /* construct a map for the available groups */
    m_groupContent = groups;
    
    constructGroupList();
    m_mi.groupStateIndexList.resize(m_groups.size());
    m_mi.groupChainStart.resize(m_groups.size());
    
    for (unsigned int i = 0 ; i < model.getDisjointPartCount() ; ++i)
    {
	const robot_desc::URDF::Link *link = model.getDisjointPart(i);
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

    computeParameterNames();
    
    // compute the index of every joint in every group
    for (std::map<std::string, Joint*>::const_iterator it = m_jointMap.begin() ; it != m_jointMap.end() ; ++it)
	if (it->second->usedParams > 0)
	    for (int i = 0 ; i < (int)m_groups.size() ; ++i)
		m_jointIndexGroup[it->first][i] = getJointIndexInGroupSlow(it->first, i);
    
    defaultState();
}

int planning_models::KinematicModel::getGroupID(const std::string &group) const
{
    std::map<std::string, int>::const_iterator pos = m_groupsMap.find(group);
    if (pos == m_groupsMap.end())
    {
	m_msg.error("Group '" + group + "' not found");
	return -1;
    }
    else
	return pos->second;
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
    for (std::map<std::string, Link*>::const_iterator it = m_linkMap.begin() ; it != m_linkMap.end() ; ++it)
	links.push_back(it->second);
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
    std::vector<Joint*> jn(m_mi.stateDimension, NULL);
    for (std::map<std::string, Joint*>::const_iterator it = m_jointMap.begin() ; it != m_jointMap.end() ; ++it)
    {
	if (it->second->usedParams == 0)
	    continue;
	std::map<std::string, unsigned int>::const_iterator p = m_mi.parameterIndex.find(it->first);
	assert(p != m_mi.parameterIndex.end());
	jn[p->second] = it->second;
    }
    joints.clear();
    for (unsigned int i = 0 ; i < jn.size() ; ++i)
	if (jn[i])
	    joints.push_back(jn[i]);
}

unsigned int planning_models::KinematicModel::getGroupDimension(int groupID) const
{
    assert(groupID >= 0 && groupID < (int)m_groups.size());
    return m_mi.groupStateIndexList[groupID].size();
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
    assert(groupID >= 0 && groupID < (int)m_groups.size());
    std::vector<std::string> nm;
    for (std::map<std::string, Joint*>::const_iterator it = m_jointMap.begin() ; it != m_jointMap.end() ; ++it)
	if (it->second->inGroup[groupID] && it->second->usedParams > 0)
	    nm.push_back(it->first);
    names.resize(nm.size());
    for (unsigned int i = 0 ; i < nm.size() ; ++i)
	names[getJointIndexInGroup(nm[i], groupID)] = nm[i];
}

void planning_models::KinematicModel::buildChainJ(Robot *robot, Link *parent, Joint* joint, const robot_desc::URDF::Link* urdfLink, const robot_desc::URDF &model)
{
    joint->before = parent;
    joint->after  = new Link();
    joint->name   = urdfLink->joint->name;
    joint->owner  = robot;
    robot->joints.push_back(joint);

    joint->extractInformation(urdfLink, robot);
    
    // construct the inGroup bitvector 
    joint->inGroup.resize(m_groups.size());
    for (unsigned int i = 0 ; i < m_groups.size() ; ++i)
    {
	bool in = false;
	for (unsigned int j = 0 ; j < m_groupContent[m_groups[i]].size() ; ++j)
	    if (m_groupContent[m_groups[i]][j] == urdfLink->name)
	    {
		in = true;
		break;
	    }
	joint->inGroup[i] = in;
    }
    
    // for each group, keep track of the indices from the robot state that correspond to it
    for (unsigned int i = 0 ; i < joint->inGroup.size() ; ++i)
	if (joint->inGroup[i])
	    for (unsigned int j = 0 ; j < joint->usedParams ; ++j)
		robot->groupStateIndexList[i].push_back(j + robot->stateDimension);
    
    // check if the current link has parents in this group
    // if it does not, it is a root link, so we keep track of it
    for (unsigned int k = 0 ; k < urdfLink->groups.size() ; ++k)
	if (urdfLink->groups[k]->isRoot(urdfLink))
	{
	    std::string gname = urdfLink->groups[k]->name;
	    if (m_groupsMap.find(gname) != m_groupsMap.end())
		robot->groupChainStart[m_groupsMap[gname]].push_back(joint);
	}
    
    if (m_verbose && joint->usedParams > 0)
    {
	m_msg.message("Joint '" + joint->name + "' connects link '" + urdfLink->parentName + "' to link '" + urdfLink->name + "' and uses state coordinates: ");
	std::stringstream ss;
	for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
	    ss << i + robot->stateDimension << " ";
	m_msg.message(ss.str());
    }

    // update the robot state dimension with the joint we just saw
    robot->stateDimension += joint->usedParams;
    buildChainL(robot, joint, joint->after, urdfLink, model);
}

void planning_models::KinematicModel::buildChainL(Robot *robot, Joint *parent, Link* link, const robot_desc::URDF::Link* urdfLink, const robot_desc::URDF &model)
{
    link->name   = urdfLink->name;
    link->before = parent;
    link->owner  = robot;
    robot->links.push_back(link);
    
    link->extractInformation(urdfLink, robot, model);
    if (link->shape == NULL)
	m_msg.error("Unable to construct shape for link '%s'", link->name.c_str());
    
    for (unsigned int i = 0 ; i < urdfLink->children.size() ; ++i)
    {
	Joint *newJoint = createJoint(urdfLink->children[i]);
	buildChainJ(robot, link, newJoint, urdfLink->children[i], model);
	link->after.push_back(newJoint);
    }
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::createJoint(const robot_desc::URDF::Link* urdfLink)
{
    Joint *newJoint = NULL;
    std::stringstream ss;
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
	ss << urdfLink->joint->type;
	m_msg.error("Unknown joint type " + ss.str());
	break;
    }  
    return newJoint;
}

unsigned int planning_models::KinematicModel::getGroupCount(void) const
{
    return m_groups.size();
}

void planning_models::KinematicModel::AttachedBody::computeTransform(btTransform &parentTrans)
{
    globalTrans = attachTrans * parentTrans;
}

int planning_models::KinematicModel::getJointIndex(const std::string &name) const
{
    std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
    if (it != m_mi.parameterIndex.end())
	return it->second;
    m_msg.error("Joint " + name + " not found");
    return -1;
}

int planning_models::KinematicModel::getJointIndexInGroup(const std::string &name, const std::string &group) const
{
    return getJointIndexInGroup(name, getGroupID(group));
}

int planning_models::KinematicModel::getJointIndexInGroup(const std::string &name, int groupID) const
{
    std::map< std::string, std::map<int, int> >::const_iterator ij = m_jointIndexGroup.find(name);
    if (ij != m_jointIndexGroup.end())
    {
	std::map<int, int>::const_iterator ig = ij->second.find(groupID);
	if (ig != ij->second.end())
	    return ig->second;
	else
	{
	    if (groupID >= 0 && groupID < (int)m_groups.size())
		m_msg.error("There is no group with ID %d", groupID);
	    else
		m_msg.error("Joint " + name + " is not in group " + m_groups[groupID]);
	}
    }
    else
	m_msg.error("Joint " + name + " not found");
    return -1;
}

int planning_models::KinematicModel::getJointIndexInGroupSlow(const std::string &name, int groupID) const
{
    std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
    unsigned int pos = it->second; // position in the complete state vector
	
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	if (m_mi.groupStateIndexList[groupID][i] == pos)
	    return i;
    return -1;
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
