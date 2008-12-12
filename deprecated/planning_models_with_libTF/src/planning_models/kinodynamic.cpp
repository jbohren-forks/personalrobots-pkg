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

#include <planning_models/kinodynamic.h>
#include <algorithm>
#include <cassert>
#include <cmath>

void planning_models::KinodynamicModelODE::build(const robot_desc::URDF &model, bool ignoreSensors)
{
    m_km.build(model, ignoreSensors);
    m_km.defaultState();
    
    std::vector<KinematicModel::Link*> links;
    m_km.getLinks(links);
    
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	ODEPart *part = new ODEPart();
	
	part->body = dBodyCreate(m_world);
	
	robot_desc::URDF::Link *ulink = model.getLink(links[i]->name);
	dMassSetParameters(&part->mass,
			   ulink->inertial->mass,
			   ulink->inertial->com[0], ulink->inertial->com[1], ulink->inertial->com[2],
			   /* urdf gives Ixx Ixy Ixz Iyy Iyz Izz, ODE needs Ixx Iyy Izz Ixy Ixz Iyz */
			   ulink->inertial->inertia[0], ulink->inertial->inertia[3], ulink->inertial->inertia[5],
			   ulink->inertial->inertia[1], ulink->inertial->inertia[2], ulink->inertial->inertia[4]);
	dBodySetMass(part->body, &part->mass);

	part->geom = createODEGeom(m_space, links[i]->shape);

	dGeomSetBody(part->geom, part->body);
	setBodyPosition(part->body, links[i]->globalTrans);
	
	m_namePart[links[i]->name] = part;
	m_parts.push_back(part);
    }
    
    std::vector<KinematicModel::Joint*> joints;
    m_km.getJoints(joints);

    for (unsigned int i = 0 ; i < joints.size() ; ++i)
    {
	if (!joints[i]->before || !joints[i]->after)
	    continue;
	ODEPart *part1 = m_namePart[joints[i]->before->name];
	ODEPart *part2 = m_namePart[joints[i]->after->name];
	assert(part1 && part2);
	
	robot_desc::URDF::Link *ulink = model.getJointLink(joints[i]->name);
	dJointID ojoint;
	
	switch (ulink->joint->type)
	{
	case robot_desc::URDF::Link::Joint::REVOLUTE:
	    ojoint = dJointCreateHinge(m_world, NULL);
	    dJointAttach(ojoint, part1->body, part2->body);
	    if (ulink->joint->isSet["limit"])
	    {
		dJointSetHingeParam(ojoint, dParamLoStop, ulink->joint->limit[0]);
		dJointSetHingeParam(ojoint, dParamHiStop, ulink->joint->limit[1]);
	    }
	    dJointSetHingeAxis(ojoint, ulink->joint->axis[0], ulink->joint->axis[1], ulink->joint->axis[2]);
	    {
		libTF::Pose3D::Position pos = joints[i]->before->globalTransFwd.getPosition();
		dJointSetHingeAnchor(ojoint, pos.x + ulink->joint->anchor[0], pos.y + ulink->joint->anchor[1], pos.z + ulink->joint->anchor[2]);
	    }
	    m_joints.push_back(ojoint);
	    m_nameJoint[joints[i]->name] = ojoint;
	    break;
	case robot_desc::URDF::Link::Joint::PRISMATIC:
	    ojoint = dJointCreateSlider(m_world, NULL);
	    dJointAttach(ojoint, part1->body, part2->body);
	    if (ulink->joint->isSet["limit"])
	    {
		dJointSetSliderParam(ojoint, dParamLoStop, ulink->joint->limit[0]);
		dJointSetSliderParam(ojoint, dParamHiStop, ulink->joint->limit[1]);
	    }
	    dJointSetSliderAxis(ojoint, ulink->joint->axis[0], ulink->joint->axis[1], ulink->joint->axis[2]);
	    m_joints.push_back(ojoint);
	    m_nameJoint[joints[i]->name] = ojoint;
	    break;
	case robot_desc::URDF::Link::Joint::FIXED:
	    ojoint = dJointCreateHinge(m_world, NULL);
	    dJointAttach(ojoint, part1->body, part2->body);
	    dJointSetHingeParam(ojoint, dParamLoStop, 0.0);
	    dJointSetHingeParam(ojoint, dParamHiStop, 0.0);
	    dJointSetHingeAxis(ojoint, 1.0, 0.0, 0.0);
	    dJointSetHingeAnchor(ojoint, 0.0, 0.0, 0.0);
	    break;
	default:
	    break;
	}
    }
}

void planning_models::KinodynamicModelODE::setGeomPosition(dGeomID geom, const libTF::Pose3D &pose)
{
    libTF::Pose3D::Position pos = pose.getPosition();
    dGeomSetPosition(geom, pos.x, pos.y, pos.z);
    libTF::Pose3D::Quaternion quat = pose.getQuaternion();
    dQuaternion q; q[0] = quat.w; q[1] = quat.x; q[2] = quat.y; q[3] = quat.z;
    dGeomSetQuaternion(geom, q);
}

void planning_models::KinodynamicModelODE::setBodyPosition(dBodyID body, const libTF::Pose3D &pose)
{
    libTF::Pose3D::Position pos = pose.getPosition();
    dBodySetPosition(body, pos.x, pos.y, pos.z);
    libTF::Pose3D::Quaternion quat = pose.getQuaternion();
    dQuaternion q; q[0] = quat.w; q[1] = quat.x; q[2] = quat.y; q[3] = quat.z;
    dBodySetQuaternion(body, q);
}

dGeomID planning_models::KinodynamicModelODE::createODEGeom(dSpaceID space, planning_models::KinematicModel::Shape *shape) const
{
    dGeomID g = NULL;
    switch (shape->type)
    {
    case KinematicModel::Shape::SPHERE:
	{
	    g = dCreateSphere(space, static_cast<KinematicModel::Sphere*>(shape)->radius);
	}
	break;
    case KinematicModel::Shape::BOX:
	{
	    const double *size = static_cast<KinematicModel::Box*>(shape)->size;
	    g = dCreateBox(space, size[0], size[1], size[2]);
	}	
	break;
    case KinematicModel::Shape::CYLINDER:
	{
	    g = dCreateCylinder(space, static_cast<KinematicModel::Cylinder*>(shape)->radius,
				static_cast<KinematicModel::Cylinder*>(shape)->length);
	}
	break;
    default:
	break;
    }
    return g;
}
