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

#ifndef KINEMATIC_PLANNING_RKP_CONSTRAINT_EVALUATOR_
#define KINEMATIC_PLANNING_RKP_CONSTRAINT_EVALUATOR_

#include <planning_models/kinematic.h>
#include <robot_msgs/KinematicConstraints.h>
#include <iostream>
#include <vector>


class KinematicConstraintEvaluator
{
 public:
    
    KinematicConstraintEvaluator(void)
    {
    }

    virtual ~KinematicConstraintEvaluator(void)
    {
    }
    
    virtual void clear(void) = 0;
    virtual bool use(planning_models::KinematicModel *kmodel, const ros::msg *kc) = 0;
    virtual bool decide(void) = 0;
    virtual void print(std::ostream &out = std::cout) const
    {
    }
};


class PoseConstraintEvaluator : public KinematicConstraintEvaluator
{
 public:
    
    PoseConstraintEvaluator(void) : KinematicConstraintEvaluator()
    {
	m_link = NULL;
    }
    
    virtual bool use(planning_models::KinematicModel *kmodel, const ros::msg *kc)
    {
	const robot_msgs::PoseConstraint *pc = dynamic_cast<const robot_msgs::PoseConstraint*>(kc);
	if (pc)
	    return use(kmodel, *pc);
	else
	    return false;
    }
    
    bool use(planning_models::KinematicModel *kmodel, const robot_msgs::PoseConstraint &pc)
    {
	m_link = kmodel->getLink(pc.robot_link);
	m_pc   = pc;
	return true;
    }
    
    virtual void clear(void)
    {
	m_link = NULL;
    }
    
    virtual bool decide(void)
    {
	double dPos, dAng;
	evaluate(&dPos, &dAng);

	return decide(dPos, dAng);
    }
    
    void evaluate(double *distPos, double *distAng)
    {
	if (m_link)
	{	
	    if (distPos)
	    {
		if ((m_pc.type == robot_msgs::PoseConstraint::ONLY_POSITION || m_pc.type == robot_msgs::PoseConstraint::COMPLETE_POSE))
		{		    
		    btVector3 bodyPos = m_link->globalTrans.getOrigin();
		    double dx = bodyPos.getX() - m_pc.pose.position.x;
		    double dy = bodyPos.getY() - m_pc.pose.position.y;
		    double dz = bodyPos.getZ() - m_pc.pose.position.z;
		    *distPos = dx * dx + dy * dy + dz * dz;
		}
		else
		    *distPos = 0.0;
	    }
	    
	    if (distAng)
	    {
		if ((m_pc.type == robot_msgs::PoseConstraint::ONLY_ORIENTATION || m_pc.type == robot_msgs::PoseConstraint::COMPLETE_POSE))
		{
		    btQuaternion quat(m_pc.pose.orientation.x, m_pc.pose.orientation.y, m_pc.pose.orientation.z, m_pc.pose.orientation.w);
		    *distAng = quat.angle(m_link->globalTrans.getRotation());
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
    
    bool decide(double dPos, double dAng)
    {
	bool result = true;
	switch (m_pc.type)
	{
	case robot_msgs::PoseConstraint::ONLY_POSITION:
	    result = dPos < m_pc.position_distance;
	    break;
	    
	case robot_msgs::PoseConstraint::ONLY_ORIENTATION:
	    result = dAng < m_pc.orientation_distance;
	    break;
	    
	case robot_msgs::PoseConstraint::COMPLETE_POSE:		
	    result = (dPos < m_pc.position_distance) && (dAng < m_pc.orientation_distance);
	    break;
	    
	default:
	    break;
	}
	return result;
    }
    
    virtual void print(std::ostream &out = std::cout) const
    {
	if (m_link)
	{
	    out << "Constraint on link '" << m_pc.robot_link << "'" << std::endl;
	    if (m_pc.type !=  robot_msgs::PoseConstraint::ONLY_ORIENTATION)
		out << "  Desired position: " << m_pc.pose.position.x << ", " << m_pc.pose.position.y << ", " << m_pc.pose.position.z
		    << " (within distance: " << m_pc.position_distance << ")" << std::endl;
	    if (m_pc.type !=  robot_msgs::PoseConstraint::ONLY_POSITION)
		out << "  Desired orientation: " << m_pc.pose.orientation.x << ", " << m_pc.pose.orientation.y << ", " << m_pc.pose.orientation.z << ", " << m_pc.pose.orientation.w 
		    << " (within distance: " << m_pc.orientation_distance << ")" << std::endl;
	}
	else
	    out << "No constraint" << std::endl;
    }
    
protected:
    
    robot_msgs::PoseConstraint             m_pc;
    planning_models::KinematicModel::Link *m_link;    
    
};


class KinematicConstraintEvaluatorSet
{
 public:
    
    KinematicConstraintEvaluatorSet(void)
    {
    }

    ~KinematicConstraintEvaluatorSet(void)
    {
	clear();
    }
    
    void clear(void)
    {
	for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	    delete m_kce[i];
	m_kce.clear();	
    }
	      
    bool use(planning_models::KinematicModel *kmodel, const std::vector<robot_msgs::PoseConstraint> &kc)
    {
	clear();
	bool result = true;
	for (unsigned int i = 0 ; i < kc.size() ; ++i)
	{
	    PoseConstraintEvaluator *ev = new PoseConstraintEvaluator();
	    result = result && ev->use(kmodel, kc[i]);
	    m_kce.push_back(ev);
	}
	return result;
    }
    
    bool decide(void)
    {
	for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	    if (!m_kce[i]->decide())
		return false;
	return true;
    }
    
    void print(std::ostream &out = std::cout) const
    {
	out << m_kce.size() << " constraints" << std::endl;
	for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
	    m_kce[i]->print(out);
    }
    
protected:
    
    std::vector<KinematicConstraintEvaluator*> m_kce;
    
};


#endif
