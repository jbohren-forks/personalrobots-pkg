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
#include <angles/angles.h>
#include <iostream>
#include <vector>

namespace kinematic_planning
{
    
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
	virtual bool use(planning_models::KinematicModel *kmodel, const ros::Message *kc) = 0;
	virtual bool decide(void) const = 0;
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
	
	virtual bool use(planning_models::KinematicModel *kmodel, const ros::Message *kc)
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
	
	virtual bool decide(void) const
	{
	    double dPos, dAng;
	    evaluate(&dPos, &dAng);
	    
	    return decide(dPos, dAng);
	}
	
	void evaluate(double *distPos, double *distAng) const
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
			case robot_msgs::PoseConstraint::POSITION_XYZ:
			    dx = bodyPos.getX() - m_pc.x;
			    dy = bodyPos.getY() - m_pc.y;
			    dz = bodyPos.getZ() - m_pc.z;
			    *distPos = dx * dx + dy * dy + dz * dz;
			    break;
			case robot_msgs::PoseConstraint::POSITION_XY:
			    dx = bodyPos.getX() - m_pc.x;
			    dy = bodyPos.getY() - m_pc.y;
			    *distPos = dx * dx + dy * dy;
			    break;
			case robot_msgs::PoseConstraint::POSITION_XZ:
			    dx = bodyPos.getX() - m_pc.x;
			    dz = bodyPos.getZ() - m_pc.z;
			    *distPos = dx * dx + dz * dz;
			    break;
			case robot_msgs::PoseConstraint::POSITION_YZ:
			    dy = bodyPos.getY() - m_pc.y;
			    dz = bodyPos.getZ() - m_pc.z;
			    *distPos = dy * dy + dz * dz;
			    break;
			case robot_msgs::PoseConstraint::POSITION_X:
			    dx = bodyPos.getX() - m_pc.x;
			    *distPos = dx * dx;
			    break;
			case robot_msgs::PoseConstraint::POSITION_Y:
			    dy = bodyPos.getY() - m_pc.y;
			    *distPos = dy * dy;
			    break;
			case robot_msgs::PoseConstraint::POSITION_Z:
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
			case robot_msgs::PoseConstraint::ORIENTATION_RPY:
			    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll)) + 
				fabs(angles::shortest_angular_distance(pitch, m_pc.pitch)) +
				fabs(angles::shortest_angular_distance(yaw, m_pc.yaw));
			    break;
			case robot_msgs::PoseConstraint::ORIENTATION_RP:
			    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll)) + 
				fabs(angles::shortest_angular_distance(pitch, m_pc.pitch));
			    break;
			case robot_msgs::PoseConstraint::ORIENTATION_RY:
			    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll)) + 
				fabs(angles::shortest_angular_distance(yaw, m_pc.yaw));
			    break;
			case robot_msgs::PoseConstraint::ORIENTATION_PY:
			    *distAng = fabs(angles::shortest_angular_distance(pitch, m_pc.pitch)) +
				fabs(angles::shortest_angular_distance(yaw, m_pc.yaw));
			    break;
			case robot_msgs::PoseConstraint::ORIENTATION_R:
			    *distAng = fabs(angles::shortest_angular_distance(roll, m_pc.roll));
			    break;
			case robot_msgs::PoseConstraint::ORIENTATION_P:
			    *distAng = fabs(angles::shortest_angular_distance(pitch, m_pc.pitch));
			    break;
			case robot_msgs::PoseConstraint::ORIENTATION_Y:
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
	
	bool decide(double dPos, double dAng) const
	{
	    bool v1 = (m_pc.type & 0xFF) ? dPos < m_pc.position_distance : true;
	    bool v2 = (m_pc.type & (~0xFF)) ? dAng < m_pc.orientation_distance : true;
	    return v1 && v2;
	}
	
	const robot_msgs::PoseConstraint& getConstraintMessage(void) const
	{
	    return m_pc;
	}
	
	virtual void print(std::ostream &out = std::cout) const
	{
	    if (m_link)
	    {
		out << "Constraint on link '" << m_pc.robot_link << "'" << std::endl;

		if (m_pc.type & 0xFF)
		{
		    out << "  Desired position: ";
		    switch (m_pc.type & 0xFF)
		    {
		    case robot_msgs::PoseConstraint::POSITION_XYZ:
			out << "x = " << m_pc.x << " ";
			out << "y = " << m_pc.y << " ";
			out << "z = " << m_pc.z << " ";
			break;
		    case robot_msgs::PoseConstraint::POSITION_XY:
			out << "x = " << m_pc.x << " ";
			out << "y = " << m_pc.y << " ";
			break;
		    case robot_msgs::PoseConstraint::POSITION_XZ:
			out << "x = " << m_pc.x << " ";
			out << "z = " << m_pc.z << " ";
			break;
		    case robot_msgs::PoseConstraint::POSITION_YZ:
			out << "y = " << m_pc.y << " ";
			out << "z = " << m_pc.z << " ";
			break;
		    case robot_msgs::PoseConstraint::POSITION_X:
			out << "x = " << m_pc.x << " ";
			break;
		    case robot_msgs::PoseConstraint::POSITION_Y:
			out << "y = " << m_pc.y << " ";
			break;
		    case robot_msgs::PoseConstraint::POSITION_Z:
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
		    case robot_msgs::PoseConstraint::ORIENTATION_RPY:
			out << "roll = " << m_pc.roll << " ";
			out << "pitch = " << m_pc.pitch << " ";
			out << "yaw = " << m_pc.yaw << " ";
			break;
		    case robot_msgs::PoseConstraint::ORIENTATION_RP:
			out << "roll = " << m_pc.roll << " ";
			out << "pitch = " << m_pc.pitch << " ";
			break;
		    case robot_msgs::PoseConstraint::ORIENTATION_RY:
			out << "roll = " << m_pc.roll << " ";
			out << "yaw = " << m_pc.yaw << " ";
			break;
		    case robot_msgs::PoseConstraint::ORIENTATION_PY:
			out << "pitch = " << m_pc.pitch << " ";
			out << "yaw = " << m_pc.yaw << " ";
			break;
		    case robot_msgs::PoseConstraint::ORIENTATION_R:
			out << "roll = " << m_pc.roll << " ";
			break;
		    case robot_msgs::PoseConstraint::ORIENTATION_P:
			out << "pitch = " << m_pc.pitch << " ";
			break;
		    case robot_msgs::PoseConstraint::ORIENTATION_Y:
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
	
	bool decide(void) const
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
} // kinematic_planning


#endif
