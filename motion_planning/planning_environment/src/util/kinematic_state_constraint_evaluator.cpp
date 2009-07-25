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

#include "planning_environment/util/kinematic_state_constraint_evaluator.h"
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <cassert>

bool planning_environment::KinematicConstraintEvaluator::decide(const double *params) const
{
    return decide(params, -1);
}

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

void planning_environment::JointConstraintEvaluator::use(const planning_models::KinematicModel *kmodel)
{
    if (m_joint)
    {   
	m_kmodel = kmodel;
	m_joint  = kmodel->getJoint(m_jc.joint_name);
    }
}

bool planning_environment::JointConstraintEvaluator::decide(const double *params, const int groupID) const
{
    const double *val = params + (groupID >= 0 ? m_kmodel->getJointIndexInGroup(m_joint->name, groupID) : m_kmodel->getJointIndex(m_joint->name));
    assert(m_jc.value.size() == m_jc.tolerance_below.size() && m_jc.value.size() == m_jc.tolerance_above.size());
    
    for (unsigned int i = 0 ; i < m_jc.value.size() ; ++i)
    {
	double dif = val[i] - m_jc.value[i];
	if (dif > m_jc.tolerance_above[i] || dif < - m_jc.tolerance_below[i])
	    return false;
    }
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
	out << "  value = ";	    
	for (unsigned int i = 0 ; i < m_jc.value.size() ; ++i)
	    out << m_jc.value[i] << "; ";
	out << "  tolerance below = ";	    
	for (unsigned int i = 0 ; i < m_jc.tolerance_below.size() ; ++i)
	    out << m_jc.tolerance_below[i] << "; ";	
	out << "  tolerance above = ";
	for (unsigned int i = 0 ; i < m_jc.tolerance_above.size() ; ++i)
	    out << m_jc.tolerance_above[i] << "; ";
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

    tf::Pose pose;
    tf::poseMsgToTF(m_pc.pose.pose, pose);
    pose.getBasis().getEulerYPR(m_yaw, m_pitch, m_roll);
    m_x = pose.getOrigin().x();
    m_y = pose.getOrigin().y();
    m_z = pose.getOrigin().z();
    
    return true;
}

void planning_environment::PoseConstraintEvaluator::use(const planning_models::KinematicModel *kmodel)
{
    if (m_link)
	m_link = kmodel->getLink(m_pc.link_name);
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
	    *distPos = 0.0;
	    
	    if (m_pc.type & (motion_planning_msgs::PoseConstraint::POSITION_X | motion_planning_msgs::PoseConstraint::POSITION_Y | motion_planning_msgs::PoseConstraint::POSITION_Z))
	    {
		const btVector3 &bodyPos = m_link->globalTrans.getOrigin();
		if (m_pc.type & motion_planning_msgs::PoseConstraint::POSITION_X)
		{
		    double dx = bodyPos.getX() - m_x;
		    if (dx > m_pc.position_tolerance_above.x || -m_pc.position_tolerance_below.x > dx)
			*distPos += dx * dx;
		}
		if (m_pc.type & motion_planning_msgs::PoseConstraint::POSITION_Y)
		{
		    double dy = bodyPos.getX() - m_y;
		    if (dy > m_pc.position_tolerance_above.y || -m_pc.position_tolerance_below.y > dy)
			*distPos += dy * dy;
		}
		if (m_pc.type & motion_planning_msgs::PoseConstraint::POSITION_Z)
		{
		    double dz = bodyPos.getZ() - m_z;
		    if (dz > m_pc.position_tolerance_above.z || -m_pc.position_tolerance_below.z > dz)
			*distPos += dz * dz;
		}
	    }
	}
	
	
	
	if (distAng)
	{
	    *distAng = 0.0;

	    if (m_pc.type & (motion_planning_msgs::PoseConstraint::ORIENTATION_R | motion_planning_msgs::PoseConstraint::ORIENTATION_P | motion_planning_msgs::PoseConstraint::ORIENTATION_Y))
	    {
		btScalar yaw, pitch, roll;
		m_link->globalTrans.getBasis().getEulerYPR(yaw, pitch, roll);

		if (m_pc.type & motion_planning_msgs::PoseConstraint::ORIENTATION_R)
		{
		    double dx = angles::shortest_angular_distance(roll, m_roll);
		    if (dx > m_pc.orientation_tolerance_above.x || -m_pc.orientation_tolerance_below.x > dx)
			*distAng += fabs(dx); 
		}
		if (m_pc.type & motion_planning_msgs::PoseConstraint::ORIENTATION_P)
		{
		    double dy = angles::shortest_angular_distance(pitch, m_pitch);
		    if (dy > m_pc.orientation_tolerance_above.y || -m_pc.orientation_tolerance_below.y > dy)
			*distAng += fabs(dy);
		}
		if (m_pc.type & motion_planning_msgs::PoseConstraint::ORIENTATION_Y)
		{
		    double dz = angles::shortest_angular_distance(yaw, m_yaw);
		    if (dz > m_pc.orientation_tolerance_above.z || -m_pc.orientation_tolerance_below.z > dz)
			*distAng += fabs(dz);
		}
	    }
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
    // the values should actually be 0, for this to be true, but we put a small eps
    bool v1 = (m_pc.type & 0xFF) ? dPos < 1e-12 : true;
    bool v2 = (m_pc.type & (~0xFF)) ? dAng < 1e-12 : true;
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

	if (m_pc.type & motion_planning_msgs::PoseConstraint::POSITION_X)
	{
	    out << "x = " << m_x << " ";
	    out << " with tolerance: above " << m_pc.position_tolerance_above.x << ", below "
		<< m_pc.position_tolerance_below.x << std::endl;
	}	    
	if (m_pc.type & motion_planning_msgs::PoseConstraint::POSITION_Y)
	{
	    out << "y = " << m_y << " ";
	    out << " with tolerance: above " << m_pc.position_tolerance_above.y << ", below "
		<< m_pc.position_tolerance_below.y << std::endl;
	}	
	if (m_pc.type & motion_planning_msgs::PoseConstraint::POSITION_Z)
	{
	    out << "z = " << m_z << " ";
	    out << " with tolerance: above " << m_pc.position_tolerance_above.z << ", below "
		<< m_pc.position_tolerance_below.z  << std::endl;
	}
	
	if (m_pc.type & motion_planning_msgs::PoseConstraint::ORIENTATION_R)
	{
	    out << "roll = " << m_roll << " ";
	    out << " with tolerance: above " << m_pc.position_tolerance_above.x << ", below "
		<< m_pc.position_tolerance_below.x << std::endl;
	}
	
	if (m_pc.type & motion_planning_msgs::PoseConstraint::ORIENTATION_P)
	{
	    out << "pitch = " << m_pitch << " ";
	    out << " with tolerance: above " << m_pc.position_tolerance_above.y << ", below "
		<< m_pc.position_tolerance_below.y << std::endl;
	}
	if (m_pc.type & motion_planning_msgs::PoseConstraint::ORIENTATION_Y)
	{
	    out << "yaw = " << m_yaw << " ";
	    out << " with tolerance: above " << m_pc.position_tolerance_above.z << ", below "
		<< m_pc.position_tolerance_below.z << std::endl;
	    
	}
	if (m_pc.type & (motion_planning_msgs::PoseConstraint::ORIENTATION_R | motion_planning_msgs::PoseConstraint::ORIENTATION_P | motion_planning_msgs::PoseConstraint::ORIENTATION_Y))
	    out << "Orientation importance is " << m_pc.orientation_importance;
    }
    else
	out << "No constraint" << std::endl;
}

void planning_environment::KinematicConstraintEvaluatorSet::clear(void)
{
    for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	delete m_kce[i];
    m_kce.clear();	
    m_jc.clear();
    m_pc.clear();
}
	
bool planning_environment::KinematicConstraintEvaluatorSet::add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::JointConstraint> &jc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < jc.size() ; ++i)
    {
	JointConstraintEvaluator *ev = new JointConstraintEvaluator();
	result = result && ev->use(kmodel, jc[i]);
	m_kce.push_back(ev);
	m_jc.push_back(jc[i]);
    }
    return result;
}

bool planning_environment::KinematicConstraintEvaluatorSet::add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::PoseConstraint> &pc)
{
    bool result = true;
    for (unsigned int i = 0 ; i < pc.size() ; ++i)
    {
	PoseConstraintEvaluator *ev = new PoseConstraintEvaluator();
	result = result && ev->use(kmodel, pc[i]);
	m_kce.push_back(ev);
	m_pc.push_back(pc[i]);
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

bool planning_environment::KinematicConstraintEvaluatorSet::decide(const double *params) const
{
    return decide(params, -1);
}

void planning_environment::KinematicConstraintEvaluatorSet::use(const planning_models::KinematicModel *kmodel)
{
    for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	m_kce[i]->use(kmodel);
}

void planning_environment::KinematicConstraintEvaluatorSet::print(std::ostream &out) const
{
    out << m_kce.size() << " kinematic constraints" << std::endl;
    for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	m_kce[i]->print(out);
}
