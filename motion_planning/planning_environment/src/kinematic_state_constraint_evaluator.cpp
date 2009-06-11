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

#include "planning_environment/kinematic_state_constraint_evaluator.h"

bool planning_environment::JointConstraintEvaluator::use(const planning_models::KinematicModel *kmodel, const ros::Message *kc)
{
    const motion_planning_msgs::JointConstraint *jc = dynamic_cast<const motion_planning_msgs::JointConstraint*>(kc);
    if (jc)
	return use(kmodel, *jc);
    else
	return false;
}

bool planning_environment::JointConstraintEvaluator::use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::JointConstraint &jc)
{
    m_kmodel = kmodel;
    m_joint  = kmodel->getJoint(jc.joint_name);
    m_jc     = jc;
    return true;
}
	
bool planning_environment::JointConstraintEvaluator::decide(const double *params, const int groupID) const
{
    const double *val = params + m_kmodel->getJointIndexInGroup(m_joint->name, groupID);
    for (unsigned int i = 0 ; i < m_jc.min.size() ; ++i)
	if (val[i] < m_jc.min[i])
	    return false;
    for (unsigned int i = 0 ; i < m_jc.max.size() ; ++i)
	if (val[i] > m_jc.max[i])
	    return false;
    return true;
}

void planning_environment::JointConstraintEvaluator::clear(void)
{
    m_joint = NULL;
    m_kmodel = NULL;
}

const motion_planning_msgs::JointConstraint& planning_environment::JointConstraintEvaluator::getConstraintMessage(void) const
{
    return m_jc;
}

void planning_environment::JointConstraintEvaluator::print(std::ostream &out) const
{		
    if (m_joint)
    {
	out << "Joint constraint for joint " << m_jc.joint_name << ": " << std::endl;
	out << "  min = ";	    
	for (unsigned int i = 0 ; i < m_jc.min.size() ; ++i)
	    out << m_jc.min[i] << "; ";
	
	out << "  max = ";
	for (unsigned int i = 0 ; i < m_jc.max.size() ; ++i)
	    out << m_jc.max[i] << "; ";
	out << std::endl;
    }
    else
	out << "No constraint" << std::endl;
    
}


bool planning_environment::PoseConstraintEvaluator::use(const planning_models::KinematicModel *kmodel, const ros::Message *kc)
{
    const motion_planning_msgs::PoseConstraint *pc = dynamic_cast<const motion_planning_msgs::PoseConstraint*>(kc);
    if (pc)
	return use(kmodel, *pc);
    else
	return false;
}
	
bool planning_environment::PoseConstraintEvaluator::use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::PoseConstraint &pc)
{
    m_link = kmodel->getLink(pc.link_name);
    m_pc   = pc;
    return true;
}

void planning_environment::PoseConstraintEvaluator::clear(void)
{
    m_link = NULL;
}

bool planning_environment::PoseConstraintEvaluator::decide(const double *, int) const
{
    double dPos, dAng;
    evaluate(&dPos, &dAng);
    
    return decide(dPos, dAng);
}

void planning_environment::PoseConstraintEvaluator::evaluate(double *distPos, double *distAng) const
{
    if (m_link)
    {	
	if (distPos)
	{
	    if (m_pc.type & 0xFF)
	    {		    
		const btVector3 &bodyPos = m_link->globalTrans.getOrigin();
		double dx, dy, dz;			
		switch (m_pc.type & 0xFF)
		{
		case motion_planning_msgs::PoseConstraint::POSITION_XYZ:
		    dx = bodyPos.getX() - m_pc.x;
		    dy = bodyPos.getY() - m_pc.y;
		    dz = bodyPos.getZ() - m_pc.z;
		    *distPos = dx * dx + dy * dy + dz * dz;
		    break;
		case motion_planning_msgs::PoseConstraint::POSITION_XY:
		    dx = bodyPos.getX() - m_pc.x;
		    dy = bodyPos.getY() - m_pc.y;
		    *distPos = dx * dx + dy * dy;
		    break;
		case motion_planning_msgs::PoseConstraint::POSITION_XZ:
		    dx = bodyPos.getX() - m_pc.x;
		    dz = bodyPos.getZ() - m_pc.z;
		    *distPos = dx * dx + dz * dz;
		    break;
		case motion_planning_msgs::PoseConstraint::POSITION_YZ:
		    dy = bodyPos.getY() - m_pc.y;
		    dz = bodyPos.getZ() - m_pc.z;
		    *distPos = dy * dy + dz * dz;
		    break;
		case motion_planning_msgs::PoseConstraint::POSITION_X:
		    dx = bodyPos.getX() - m_pc.x;
		    *distPos = dx * dx;
		    break;
		case motion_planning_msgs::PoseConstraint::POSITION_Y:
		    dy = bodyPos.getY() - m_pc.y;
		    *distPos = dy * dy;
		    break;
		case motion_planning_msgs::PoseConstraint::POSITION_Z:
		    dz = bodyPos.getZ() - m_pc.z;
		    *distPos = dz * dz;
		    break;
		default:
		    *distPos = 0.0;
		}
	    }
	    else
		*distPos = 0.0;
	}
	
	if (distAng)
	{
	    if (m_pc.type & (~0xFF))
	    {
		btScalar yaw, pitch, roll;
		m_link->globalTrans.getBasis().getEulerYPR(yaw, pitch, roll);			
		switch (m_pc.type & (~0xFF))
		{
		case motion_planning_msgs::PoseConstraint::ORIENTATION_RPY:
		    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll)) + 
			fabs(angles::shortest_angular_distance(pitch, m_pc.pitch)) +
			fabs(angles::shortest_angular_distance(yaw, m_pc.yaw));
		    break;
		case motion_planning_msgs::PoseConstraint::ORIENTATION_RP:
		    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll)) + 
			fabs(angles::shortest_angular_distance(pitch, m_pc.pitch));
		    break;
		case motion_planning_msgs::PoseConstraint::ORIENTATION_RY:
		    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll)) + 
			fabs(angles::shortest_angular_distance(yaw, m_pc.yaw));
		    break;
		case motion_planning_msgs::PoseConstraint::ORIENTATION_PY:
		    *distAng = fabs(angles::shortest_angular_distance(pitch, m_pc.pitch)) +
			fabs(angles::shortest_angular_distance(yaw, m_pc.yaw));
		    break;
		case motion_planning_msgs::PoseConstraint::ORIENTATION_R:
		    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll));
		    break;
		case motion_planning_msgs::PoseConstraint::ORIENTATION_P:
		    *distAng = fabs(angles::shortest_angular_distance(pitch, m_pc.pitch));
		    break;
		case motion_planning_msgs::PoseConstraint::ORIENTATION_Y:
		    *distAng = fabs(angles::shortest_angular_distance(yaw, m_pc.yaw));
		    break;
		default:
		    *distAng = 0.0;
		    break;
		}    
	    }
	    else
		*distAng = 0.0;
	}
    }
    else
    {
	if (distPos)
	    *distPos = 0.0;
	if (distAng)
	    *distAng = 0.0;
    }
}

bool planning_environment::PoseConstraintEvaluator::decide(double dPos, double dAng) const
{
    bool v1 = (m_pc.type & 0xFF) ? dPos < m_pc.position_distance : true;
    bool v2 = (m_pc.type & (~0xFF)) ? dAng < m_pc.orientation_distance : true;
    return v1 && v2;
}

const motion_planning_msgs::PoseConstraint& planning_environment::PoseConstraintEvaluator::getConstraintMessage(void) const
{
    return m_pc;
}

void planning_environment::PoseConstraintEvaluator::print(std::ostream &out) const
{
    if (m_link)
    {
	out << "Pose constraint on link '" << m_pc.link_name << "'" << std::endl;
	
	if (m_pc.type & 0xFF)
	{
	    out << "  Desired position: ";
	    switch (m_pc.type & 0xFF)
	    {
	    case motion_planning_msgs::PoseConstraint::POSITION_XYZ:
		out << "x = " << m_pc.x << " ";
		out << "y = " << m_pc.y << " ";
		out << "z = " << m_pc.z << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::POSITION_XY:
		out << "x = " << m_pc.x << " ";
		out << "y = " << m_pc.y << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::POSITION_XZ:
		out << "x = " << m_pc.x << " ";
		out << "z = " << m_pc.z << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::POSITION_YZ:
		out << "y = " << m_pc.y << " ";
		out << "z = " << m_pc.z << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::POSITION_X:
		out << "x = " << m_pc.x << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::POSITION_Y:
		out << "y = " << m_pc.y << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::POSITION_Z:
		out << "z = " << m_pc.z << " ";
		break;
	    default:
		break;
	    }
	    
	    out << " (within distance: " << m_pc.position_distance << ")" << std::endl;
	}
	
	
	if (m_pc.type & (~0xFF))
	{
	    out << "  Desired orientation: ";
	    switch (m_pc.type & (~0xFF))
	    {
	    case motion_planning_msgs::PoseConstraint::ORIENTATION_RPY:
		out << "roll = " << m_pc.roll << " ";
		out << "pitch = " << m_pc.pitch << " ";
		out << "yaw = " << m_pc.yaw << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::ORIENTATION_RP:
		out << "roll = " << m_pc.roll << " ";
		out << "pitch = " << m_pc.pitch << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::ORIENTATION_RY:
		out << "roll = " << m_pc.roll << " ";
		out << "yaw = " << m_pc.yaw << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::ORIENTATION_PY:
		out << "pitch = " << m_pc.pitch << " ";
		out << "yaw = " << m_pc.yaw << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::ORIENTATION_R:
		out << "roll = " << m_pc.roll << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::ORIENTATION_P:
		out << "pitch = " << m_pc.pitch << " ";
		break;
	    case motion_planning_msgs::PoseConstraint::ORIENTATION_Y:
		out << "yaw = " << m_pc.yaw << " ";
		break;
	    default:
		break;
	    }    
	    
	    out << " (within distance: " << m_pc.orientation_distance;
	    if (m_pc.type & 0xFF)
		out << " and with importance " << m_pc.orientation_importance;
	    out << ")" << std::endl;
	}
    }
    else
	out << "No constraint" << std::endl;
}

void planning_environment::KinematicConstraintEvaluatorSet::clear(void)
{
    for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	delete m_kce[i];
    m_kce.clear();	
}
	
bool planning_environment::KinematicConstraintEvaluatorSet::add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::JointConstraint> &jc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < jc.size() ; ++i)
    {
	JointConstraintEvaluator *ev = new JointConstraintEvaluator();
	result = result && ev->use(kmodel, jc[i]);
	m_kce.push_back(ev);
    }
    return result;
}

bool planning_environment::KinematicConstraintEvaluatorSet::add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::PoseConstraint> &kc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < kc.size() ; ++i)
    {
	PoseConstraintEvaluator *ev = new PoseConstraintEvaluator();
	result = result && ev->use(kmodel, kc[i]);
	m_kce.push_back(ev);
    }
    return result;
}

bool planning_environment::KinematicConstraintEvaluatorSet::decide(const double *params, int groupID) const
{
    for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	if (!m_kce[i]->decide(params, groupID))
	    return false;
    return true;
}

void planning_environment::KinematicConstraintEvaluatorSet::print(std::ostream &out) const
{
    out << m_kce.size() << " kinematic constraints" << std::endl;
    for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	m_kce[i]->print(out);
}
