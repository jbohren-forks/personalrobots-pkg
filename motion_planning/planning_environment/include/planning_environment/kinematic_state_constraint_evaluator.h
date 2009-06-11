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

#ifndef PLANNING_ENVIRONMENT_KINEMATIC_STATE_CONSTRAINT_EVALUATOR_
#define PLANNING_ENVIRONMENT_KINEMATIC_STATE_CONSTRAINT_EVALUATOR_

#include <planning_models/kinematic.h>
#include <motion_planning_msgs/KinematicConstraints.h>
#include <angles/angles.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <vector>

namespace planning_environment
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
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc) = 0;

	/** Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params, const int groupID) const = 0;

	virtual void print(std::ostream &out = std::cout) const
	{
	}
    };
    
    class JointConstraintEvaluator : public KinematicConstraintEvaluator
    {
    public:  

	JointConstraintEvaluator(void) : KinematicConstraintEvaluator()
	{
	    m_joint = NULL;
	    m_kmodel = NULL;
	}

	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc);
	bool use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::JointConstraint &jc);
	virtual bool decide(const double *params, const int groupID) const;
	virtual void clear(void);
	const motion_planning_msgs::JointConstraint& getConstraintMessage(void) const;
	virtual void print(std::ostream &out = std::cout) const;

    protected:
	
	motion_planning_msgs::JointConstraint   m_jc;
	planning_models::KinematicModel::Joint *m_joint;    
	const planning_models::KinematicModel  *m_kmodel;
	
    };
    
	
    class PoseConstraintEvaluator : public KinematicConstraintEvaluator
    {
    public:
	
        PoseConstraintEvaluator(void) : KinematicConstraintEvaluator()
	{
	    m_link = NULL;
	}
	
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc);
	bool use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::PoseConstraint &pc);
	virtual void clear(void);
	virtual bool decide(const double *params, int groupID) const;
	void evaluate(double *distPos, double *distAng) const;
	bool decide(double dPos, double dAng) const;
	
	const motion_planning_msgs::PoseConstraint& getConstraintMessage(void) const;
	void print(std::ostream &out = std::cout) const;
	
    protected:
	
	motion_planning_msgs::PoseConstraint   m_pc;
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
	
	void clear(void);
	bool add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::JointConstraint> &jc);
	bool add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::PoseConstraint> &kc);
	bool decide(const double *params, int groupID) const;
	void print(std::ostream &out = std::cout) const;

    protected:
	
	std::vector<KinematicConstraintEvaluator*> m_kce;
    
    };
} // planning_environment


#endif
