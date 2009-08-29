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

#include <planning_models/kinematic_model.h>
#include <resource_retriever/retriever.h>
#include <ogre_tools/stl_loader.h>
#include <ros/console.h>
#include <cmath>

/* ------------------------ KinematicModel ------------------------ */

planning_models::KinematicModel::KinematicModel(const urdf::Model &model, const std::map< std::string, std::vector<std::string> > &groups)
{    
    dimension_ = 0;
    if (model.getRoot())
    {
	root_ = buildRecursive(NULL, model.getRoot().get());
	buildGroups(groups);
    }
    else
    {
	root_ = NULL;
	ROS_WARN("No root found");
    }
}

planning_models::KinematicModel::~KinematicModel(void)
{
    for (std::map<std::string, JointGroup*>::iterator it = groupMap_.begin() ; it != groupMap_.end() ; ++it)
	delete it->second;
    if (root_)
	delete root_;
}

const std::string& planning_models::KinematicModel::getName(void) const
{
    return modelName_;
}

unsigned int planning_models::KinematicModel::getDimension(void) const
{
    return dimension_;
}

const btTransform& planning_models::KinematicModel::getRootTransform(void) const
{
    return rootTransform_;
}

void planning_models::KinematicModel::setRootTransform(const btTransform &transform)
{
    rootTransform_ = transform;
}

void planning_models::KinematicModel::lock(void)
{
    lock_.lock();
}

void planning_models::KinematicModel::unlock(void)
{
    lock_.unlock();
}

void planning_models::KinematicModel::buildGroups(const std::map< std::string, std::vector<std::string> > &groups)
{
    for (std::map< std::string, std::vector<std::string> >::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
    {
	std::vector<Joint*> jointv;
	for (unsigned int i = 0 ; i < it->second.size() ; ++i)
	{
	    std::map<std::string, Joint*>::iterator p = jointMap_.find(it->second[i]);
	    if (p == jointMap_.end())
	    {
		ROS_ERROR("Unknown joint '%s'. Not adding to group '%s'", it->second[i].c_str(), it->first.c_str());
		jointv.clear();
		break;
	    }
	    else
		jointv.push_back(p->second);
	}
	if (jointv.empty())
	    ROS_ERROR("Skipping group '%s'", it->first.c_str());
	else
	{
	    ROS_DEBUG("Adding group '%s'", it->first.c_str());
	    groupMap_[it->first] = new JointGroup(this, it->first, jointv);
	}
    }
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::buildRecursive(Link *parent, const urdf::Link *link)
{
    Joint *joint = constructJoint(link->parent_joint.get());
    joint->stateIndex = dimension_;
    jointMap_[joint->name] = joint;
    jointList_.push_back(joint);
    jointIndex_.push_back(dimension_);
    dimension_ += joint->usedParams;
    joint->before = parent;
    joint->after = constructLink(link);
    linkMap_[joint->after->name] = joint->after;
    joint->after->before = joint;
    
    for (unsigned int i = 0 ; link->child_links.size() ; ++i)
	joint->after->after.push_back(buildRecursive(joint->after, link->child_links[i].get()));
    
    return joint;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::constructJoint(const urdf::Joint *urdfJoint)
{
    planning_models::KinematicModel::Joint *result = NULL;
    
    ROS_ASSERT(urdfJoint);
    
    switch (urdfJoint->type)
    {
    case urdf::Joint::REVOLUTE:
	{
	    ROS_ASSERT(urdfJoint->safety);
	    RevoluteJoint *j = new RevoluteJoint(this);
	    j->hiLimit = urdfJoint->safety->soft_upper_limit;
	    j->lowLimit = urdfJoint->safety->soft_lower_limit;
	    j->continuous = false;
	    j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
	    result = j;
	}
	break;
    case urdf::Joint::CONTINUOUS:
	{
	    RevoluteJoint *j = new RevoluteJoint(this);
	    j->hiLimit = M_PI;
	    j->lowLimit = -M_PI;
	    j->continuous = true;
	    j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
	    result = j;
	}
	break;
    case urdf::Joint::PRISMATIC:
	{
	    ROS_ASSERT(urdfJoint->safety);
	    PrismaticJoint *j = new PrismaticJoint(this);
	    j->hiLimit = urdfJoint->safety->soft_upper_limit;
	    j->lowLimit = urdfJoint->safety->soft_lower_limit;
	    j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
	    result = j;
	}
	break;
    case urdf::Joint::FLOATING:
	result = new FloatingJoint(this);
	break;
    case urdf::Joint::PLANAR:
	result = new PlanarJoint(this);
	break;
    case urdf::Joint::FIXED:
	result = new FixedJoint(this);
	break;
    default:
	ROS_ERROR("Unknown joint type: %d", (int)urdfJoint->type);
	break;
    }
    
    if (result)
	result->name = urdfJoint->name;
    
    return result;
}

namespace planning_models
{
    static inline btTransform urdfPose2btTransform(const urdf::Pose &pose)
    {
	return btTransform(btQuaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
			   btVector3(pose.position.x, pose.position.y, pose.position.z));
    }
}

planning_models::KinematicModel::Link* planning_models::KinematicModel::constructLink(const urdf::Link *urdfLink)
{
    ROS_ASSERT(urdfLink);
    ROS_ASSERT(urdfLink->collision);

    Link *result = new Link(this);
    result->name = urdfLink->name;

    result->constGeomTrans = urdfPose2btTransform(urdfLink->collision->origin);
    result->constTrans = urdfPose2btTransform(urdfLink->parent_joint->parent_to_joint_origin_transform);
    
    result->shape = constructShape(urdfLink->collision->geometry.get());
    
    return result;
}

shapes::Shape* planning_models::KinematicModel::constructShape(const urdf::Geometry *geom)
{
    ROS_ASSERT(geom);
 
    shapes::Shape *result = NULL;
    switch (geom->type)
    {
    case urdf::Geometry::SPHERE:
	result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
	break;	
    case urdf::Geometry::BOX:
	{
	    urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
	    result = new shapes::Box(dim.x, dim.y, dim.z);
	}
    case urdf::Geometry::CYLINDER:
	result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
				      dynamic_cast<const urdf::Cylinder*>(geom)->length);
	break;
    case urdf::Geometry::MESH:
	{
	    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
	    if (!mesh->filename.empty())
	    {
		resource_retriever::Retriever retriever;
		resource_retriever::MemoryResource res;
		bool ok = true;
		
		try
		{
		    res = retriever.get(mesh->filename);
		}
		catch (resource_retriever::Exception& e)
		{
		    ROS_ERROR("%s", e.what());
		    ok = false;
		}
		
		if (ok)
		{
		    if (res.size == 0)
			ROS_WARN("Retrieved empty mesh for resource [%s]", mesh->filename.c_str());
		    else
		    {
			ogre_tools::STLLoader loader;
			if (loader.load(res.data.get()))
			{
			    std::vector<btVector3> triangles;
			    for (unsigned int i = 0 ; i < loader.triangles_.size() ; ++i)
			    {
				triangles.push_back(btVector3(loader.triangles_[i].vertices_[0].x, loader.triangles_[i].vertices_[0].y, loader.triangles_[i].vertices_[0].z));
				triangles.push_back(btVector3(loader.triangles_[i].vertices_[1].x, loader.triangles_[i].vertices_[1].y, loader.triangles_[i].vertices_[1].z));
				triangles.push_back(btVector3(loader.triangles_[i].vertices_[2].x, loader.triangles_[i].vertices_[2].y, loader.triangles_[i].vertices_[2].z));
			    }
			    result = shapes::createMeshFromVertices(triangles);
			}
			else
			    ROS_ERROR("Failed to load mesh [%s]", mesh->filename.c_str());
		    }
		}
	    }
	    else
		ROS_WARN("Empty mesh filename");
	}
	
	break;
    default:
	ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
	break;
    }
    
    return result;
}

void planning_models::KinematicModel::computeTransforms(const double *params)
{
    for (unsigned int i = 0  ; i < jointList_.size() ; ++i)
	jointList_[i]->updateVariableTransform(params + jointIndex_[i]);
    
    if (root_)
    {
	std::queue<Link*> links;
	links.push(root_->after);
	while (!links.empty())
	{
	    Link *link = links.front();
	    links.pop();
	    
	    link->computeTransform();
	    for (unsigned int i = 0 ; i < link->after.size() ; ++i)
		links.push(link->after[i]->after);
	}
    }
}

bool planning_models::KinematicModel::hasJoint(const std::string &name) const
{
    return jointMap_.find(name) != jointMap_.end();
}

bool planning_models::KinematicModel::hasLink(const std::string &name) const
{
    return linkMap_.find(name) != linkMap_.end();
}

bool planning_models::KinematicModel::hasGroup(const std::string &name) const
{
    return groupMap_.find(name) != groupMap_.end();
}

const planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &name) const
{
    std::map<std::string, Joint*>::const_iterator it = jointMap_.find(name);
    if (it == jointMap_.end())
    {
	ROS_ERROR("Joint '%s' not found", name.c_str());
	return NULL;
    }
    else
	return it->second;
}

const planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &name) const
{
    std::map<std::string, Link*>::const_iterator it = linkMap_.find(name);
    if (it == linkMap_.end())
    {
	ROS_ERROR("Link '%s' not found", name.c_str());
	return NULL;
    }
    else
	return it->second;
}

const planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::getGroup(const std::string &name) const
{
    std::map<std::string, JointGroup*>::const_iterator it = groupMap_.find(name);
    if (it == groupMap_.end())
    {
	ROS_ERROR("Joint group '%s' not found", name.c_str());
	return NULL;
    }
    else
	return it->second;
}

void planning_models::KinematicModel::getGroups(std::vector<const JointGroup*> &groups) const
{
    groups.clear();
    groups.reserve(groupMap_.size());
    for (std::map<std::string, JointGroup*>::const_iterator it = groupMap_.begin() ; it != groupMap_.end() ; ++it)
	groups.push_back(it->second);
}

void planning_models::KinematicModel::getGroupNames(std::vector<std::string> &groups) const
{
    groups.clear();
    groups.reserve(groupMap_.size());
    for (std::map<std::string, JointGroup*>::const_iterator it = groupMap_.begin() ; it != groupMap_.end() ; ++it)
	groups.push_back(it->second->name);
}

void planning_models::KinematicModel::getLinks(std::vector<const Link*> &links) const
{
    links.clear();
    links.reserve(linkMap_.size());
    for (std::map<std::string, Link*>::const_iterator it = linkMap_.begin() ; it != linkMap_.end() ; ++it)
	links.push_back(it->second);
}

void planning_models::KinematicModel::getLinkNames(std::vector<std::string> &links) const
{
    links.clear();
    links.reserve(linkMap_.size());
    for (std::map<std::string, Link*>::const_iterator it = linkMap_.begin() ; it != linkMap_.end() ; ++it)
	links.push_back(it->second->name);
}

void planning_models::KinematicModel::getJoints(std::vector<const Joint*> &joints) const
{
    joints.clear();
    joints.reserve(jointList_.size());
    for (unsigned int i = 0 ; i < jointList_.size() ; ++i)
	joints.push_back(jointList_[i]);
}

void planning_models::KinematicModel::getJointNames(std::vector<std::string> &joints) const
{
    joints.clear();
    joints.reserve(jointList_.size());
    for (unsigned int i = 0 ; i < jointList_.size() ; ++i)
	joints.push_back(jointList_[i]->name);
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) const
{
}

void planning_models::KinematicModel::printLinkPoses(std::ostream &out) const
{
}
    
/* ------------------------ Joint ------------------------ */

planning_models::KinematicModel::Joint::Joint(KinematicModel *model) : owner(model), usedParams(0), stateIndex(0), before(NULL), after(NULL)
{
    varTrans.setIdentity();
}

planning_models::KinematicModel::Joint::~Joint(void)
{
    if (after)
	delete after;
}

void planning_models::KinematicModel::FixedJoint::updateVariableTransform(const double *params)
{
    // the joint remains identity
}


void planning_models::KinematicModel::PlanarJoint::updateVariableTransform(const double *params)
{
    varTrans.setOrigin(btVector3(params[0], params[1], 0.0));
    varTrans.setRotation(btQuaternion(btVector3(0.0, 0.0, 1.0), params[2]));
}

void planning_models::KinematicModel::FloatingJoint::updateVariableTransform(const double *params)
{
    varTrans.setOrigin(btVector3(params[0], params[1], params[2]));
    varTrans.setRotation(btQuaternion(params[3], params[4], params[5], params[6]));
}

void planning_models::KinematicModel::PrismaticJoint::updateVariableTransform(const double *params)
{
    varTrans.setOrigin(axis * params[0]);
}

void planning_models::KinematicModel::RevoluteJoint::updateVariableTransform(const double *params)
{
    varTrans.setRotation(btQuaternion(axis, params[0]));
}

/* ------------------------ Link ------------------------ */

planning_models::KinematicModel::Link::Link(KinematicModel *model) : owner(model), before(NULL), shape(NULL)
{
    constTrans.setIdentity();
    constGeomTrans.setIdentity();
    globalTransFwd.setIdentity();
    globalTrans.setIdentity();		
}

planning_models::KinematicModel::Link::~Link(void)
{
    if (shape)
	delete shape;
    for (unsigned int i = 0 ; i < after.size() ; ++i)
	delete after[i];
    for (unsigned int i = 0 ; i < attachedBodies.size() ; ++i)
	delete attachedBodies[i];
}


void planning_models::KinematicModel::Link::computeTransform(void)
{    
    globalTransFwd.mult(before->before ? before->before->globalTransFwd : owner->getRootTransform(), constTrans);
    globalTransFwd *= before->varTrans;    
    globalTrans.mult(globalTransFwd, constGeomTrans);
        
    for (unsigned int i = 0 ; i < attachedBodies.size() ; ++i)
	attachedBodies[i]->computeTransform();
}

/* ------------------------ AttachedBody ------------------------ */

planning_models::KinematicModel::AttachedBody::AttachedBody(Link *link) : owner(link), shape(NULL)
{
    attachTrans.setIdentity();
}

planning_models::KinematicModel::AttachedBody::~AttachedBody(void)
{
    if (shape)
	delete shape;
}

void planning_models::KinematicModel::AttachedBody::computeTransform(void)
{
    globalTrans = owner->globalTrans * attachTrans;
}


/* ------------------------ JointGroup ------------------------ */

planning_models::KinematicModel::JointGroup::JointGroup(KinematicModel *model, const std::string& groupName,
							const std::vector<Joint*> &groupJoints) : owner(model)
{
    name = groupName;
    joints = groupJoints;
    jointNames.resize(joints.size());
    jointIndex.resize(joints.size());
    dimension = 0;
    
    std::vector<const Joint*> allJoints;
    owner->getJoints(allJoints);
    
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
    {
	jointNames[i] = joints[i]->name;
	jointIndex[i] = dimension;
	dimension += joints[i]->usedParams;
	jointMap_[jointNames[i]] = i;
	
	unsigned int globalStateIndex = 0;
	bool found = false;
	for (unsigned int j = 0 ; j < allJoints.size() ; ++j)
	    if (allJoints[j]->name == joints[i]->name)
	    {
		for (unsigned int k = 0 ; k < joints[i]->usedParams ; ++k)
		    stateIndex.push_back(globalStateIndex + k);
		found = true;
		break;
	    }
	if (!found)
	    ROS_FATAL("Group joint not in kinematic model");
    }
    
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
    {
	bool found = false;
	Joint *joint = joints[i];
	while (joint->before)
	{
	    joint = joint->before->before;
	    if (hasJoint(joint->name))
	    {
		found = true;
		break;
	    }
	}
	
	if (!found)
	    jointRoots.push_back(joints[i]);
    }
}

planning_models::KinematicModel::JointGroup::~JointGroup(void)
{
}

bool planning_models::KinematicModel::JointGroup::hasJoint(const std::string &joint) const
{
    return jointMap_.find(joint) != jointMap_.end();
}

void planning_models::KinematicModel::JointGroup::computeTransforms(const double *params)
{
    for (unsigned int i = 0  ; i < joints.size() ; ++i)
	joints[i]->updateVariableTransform(params + jointIndex[i]);

    for (unsigned int i = 0 ; i < jointRoots.size() ; ++i)
    {
	std::queue<Link*> links;
	links.push(jointRoots[i]->after);
	
	while (!links.empty())
	{
	    Link *link = links.front();
	    links.pop();
	    
	    link->computeTransform();
	    for (unsigned int i = 0 ; i < link->after.size() ; ++i)
		links.push(link->after[i]->after);
	}
    }
}
