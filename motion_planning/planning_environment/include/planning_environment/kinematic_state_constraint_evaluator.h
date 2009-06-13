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
	
	/** Clear the stored constraint */
	virtual void clear(void) = 0;

	/** This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc) = 0;

	/** Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params, const int groupID) const = 0;

	/** Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params) const;

	/** Print the constraint data */
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

	/** This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc);

	/** This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	bool use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::JointConstraint &jc);

	/** Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params, const int groupID) const;

	/** Clear the stored constraint */
	virtual void clear(void);

	/** Print the constraint data */
	virtual void print(std::ostream &out = std::cout) const;

	/** Get the constraint message */
	const motion_planning_msgs::JointConstraint& getConstraintMessage(void) const;

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
	
	/** This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc);

	/** This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	bool use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::PoseConstraint &pc);

	/** Clear the stored constraint */
	virtual void clear(void);
	
	/** Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params, int groupID) const;

	/** Evaluate the distances to the position and to the orientation are given. */
	void evaluate(double *distPos, double *distAng) const;
	
	/** Decide whether the constraint is satisfied. The distances to the position and to the orientation are given. */
	bool decide(double dPos, double dAng) const;
	
	/** Print the constraint data */
	void print(std::ostream &out = std::cout) const;

	/** Get the constraint message */
	const motion_planning_msgs::PoseConstraint& getConstraintMessage(void) const;
	
    protected:
	
	motion_planning_msgs::PoseConstraint   m_pc;
	double                                 m_x;
	double                                 m_y;
	double                                 m_z;	
	double                                 m_yaw;
	double                                 m_pitch;
	double                                 m_roll;	
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
	
	/** Clear the stored constraints */
	void clear(void);
	
	/** Add a set of joint constraints */
	bool add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::JointConstraint> &jc);

	/** Add a set of pose constraints */
	bool add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::PoseConstraint> &kc);
	
	/** Decide whether the set of constraints is satisfied  */
	bool decide(const double *params, int groupID) const;

	/** Decide whether the set of constraints is satisfied  */
	bool decide(const double *params) const;

	/** Print the constraint data */
	void print(std::ostream &out = std::cout) const;

    protected:
	
	std::vector<KinematicConstraintEvaluator*> m_kce;
    
    };
} // planning_environment


#endif
