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

/** \Author Ioan Sucan */

#ifndef KINEMATIC_PLANNING_KINEMATIC_CONSTRAINT_EVALUATOR_
#define KINEMATIC_PLANNING_KINEMATIC_CONSTRAINT_EVALUATOR_

#include <planning_models/kinematic.h>
#include <robot_msgs/KinematicConstraint.h>
#include <iostream>
#include <vector>

class KinematicConstraintEvaluator
{
 public:
    
    KinematicConstraintEvaluator(void)
    {
	m_link = NULL;
    }
    
    void use(planning_models::KinematicModel *kmodel, const robot_msgs::KinematicConstraint &kc)
    {
	m_link = kmodel->getLink(kc.robot_link);
	m_kc   = kc;	
    }
    
    void clear(void)
    {
	m_link = NULL;
    }
    
    void evaluate(double *distPos, double *distAng)
    {
	if (m_link)
	{	    
	    switch (m_kc.type)
	    {
	    case robot_msgs::KinematicConstraint::ONLY_POSITION:
		if (distPos)
		{
		    libTF::Pose3D::Position bodyPos;
		    m_link->globalTrans.getPosition(bodyPos);
		    
		    double dx = bodyPos.x - m_kc.pose.position.x;
		    double dy = bodyPos.y - m_kc.pose.position.y;
		    double dz = bodyPos.z - m_kc.pose.position.z;
		    
		    *distPos = dx * dx + dy * dy + dz * dz;		    
		}
		if (distAng)
		    *distAng = 0.0;
		break;
		
	    case robot_msgs::KinematicConstraint::ONLY_ORIENTATION:
	    case robot_msgs::KinematicConstraint::COMPLETE_POSE:		
	    default:
		if (distPos)
		    *distPos = 0.0;
		if (distAng)
		    *distAng = 0.0;
		break;
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
    
    bool decide(void)
    {
	double dPos, dAng;
	evaluate(&dPos, &dAng);

	return decide(dPos, dAng);
    }
    
    bool decide(double dPos, double dAng)
    {
	bool result = true;
	switch (m_kc.type)
	{
	case robot_msgs::KinematicConstraint::ONLY_POSITION:
	    result = dPos < m_kc.position_distance;
	    break;
	    
	case robot_msgs::KinematicConstraint::ONLY_ORIENTATION:
	    result = dAng < m_kc.orientation_distance;
	    break;
	    
	case robot_msgs::KinematicConstraint::COMPLETE_POSE:		
	    result = (dPos < m_kc.position_distance) && (dAng < m_kc.orientation_distance);
	    break;
	    
	default:
	    break;
	}
	return result;
    }
    
    void print(std::ostream &out = std::cout) const
    {
	if (m_link)
	{
	    out << "Constraint on link '" << m_kc.robot_link << "'" << std::endl;
	    if (m_kc.type !=  robot_msgs::KinematicConstraint::ONLY_ORIENTATION)
		out << "  Desired position: " << m_kc.pose.position.x << ", " << m_kc.pose.position.y << ", " << m_kc.pose.position.z
		    << " (within distance: " << m_kc.position_distance << ")" << std::endl;
	    if (m_kc.type !=  robot_msgs::KinematicConstraint::ONLY_POSITION)
		out << "  Desired orientation: " << m_kc.pose.orientation.x << ", " << m_kc.pose.orientation.y << ", " << m_kc.pose.orientation.z << ", " << m_kc.pose.orientation.w 
		    << " (within distance: " << m_kc.orientation_distance << ")" << std::endl;
	}
	else
	    out << "No constraint" << std::endl;
    }
    
protected:
    
    robot_msgs::KinematicConstraint        m_kc;
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
	      
    void use(planning_models::KinematicModel *kmodel, const std::vector<robot_msgs::KinematicConstraint> &kc)
    {
	clear();
	for (unsigned int i = 0 ; i < kc.size() ; ++i)
	{
	    KinematicConstraintEvaluator *ev = new KinematicConstraintEvaluator();
	    ev->use(kmodel, kc[i]);
	    m_kce.push_back(ev);
	}
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
