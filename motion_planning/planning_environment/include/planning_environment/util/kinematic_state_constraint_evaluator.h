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

#ifndef PLANNING_ENVIRONMENT_UTIL_KINEMATIC_STATE_CONSTRAINT_EVALUATOR_
#define PLANNING_ENVIRONMENT_UTIL_KINEMATIC_STATE_CONSTRAINT_EVALUATOR_

#include <planning_models/kinematic.h>
#include <motion_planning_msgs/KinematicConstraints.h>
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
	
	/** \brief Clear the stored constraint */
	virtual void clear(void) = 0;

	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc) = 0;

	/** \brief Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params, const int groupID) const = 0;

	/** \brief Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params) const;

	/** \brief Print the constraint data */
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

	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc);

	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	bool use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::JointConstraint &jc);

	/** \brief Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params, const int groupID) const;

	/** \brief Clear the stored constraint */
	virtual void clear(void);

	/** \brief Print the constraint data */
	virtual void print(std::ostream &out = std::cout) const;

	/** \brief Get the constraint message */
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
	
	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	virtual bool use(const planning_models::KinematicModel *kmodel, const ros::Message *kc);

	/** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
	bool use(const planning_models::KinematicModel *kmodel, const motion_planning_msgs::PoseConstraint &pc);

	/** \brief Clear the stored constraint */
	virtual void clear(void);
	
	/** \brief Decide whether the constraint is satisfied. The kinematic model is assumed to be at the state we want to decide. */
	virtual bool decide(const double *params, int groupID) const;

	/** \brief Evaluate the distances to the position and to the orientation are given. */
	void evaluate(double *distPos, double *distAng) const;
	
	/** \brief Decide whether the constraint is satisfied. The distances to the position and to the orientation are given. */
	bool decide(double dPos, double dAng) const;
	
	/** \brief Print the constraint data */
	void print(std::ostream &out = std::cout) const;

	/** \brief Get the constraint message */
	const motion_planning_msgs::PoseConstraint& getConstraintMessage(void) const;
	
    protected:
	
	motion_planning_msgs::PoseConstraint   m_pc;
	double                                 m_x, m_y, m_z;
	double                                 m_roll, m_pitch, m_yaw;
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
	
	/** \brief Clear the stored constraints */
	void clear(void);
	
	/** \brief Add a set of joint constraints */
	bool add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::JointConstraint> &jc);

	/** \brief Add a set of pose constraints */
	bool add(const planning_models::KinematicModel *kmodel, const std::vector<motion_planning_msgs::PoseConstraint> &kc);
	
	/** \brief Decide whether the set of constraints is satisfied  */
	bool decide(const double *params, int groupID) const;

	/** \brief Decide whether the set of constraints is satisfied  */
	bool decide(const double *params) const;

	/** \brief Print the constraint data */
	void print(std::ostream &out = std::cout) const;

    protected:
	
	std::vector<KinematicConstraintEvaluator*> m_kce;
    
    };
} // planning_environment


#endif
