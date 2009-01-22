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

#ifndef KINEMATIC_PLANNING_RKP_STATE_VALIDATOR
#define KINEMATIC_PLANNING_RKP_STATE_VALIDATOR

#include <ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h>
#include <planning_models/kinematic.h>
#include <collision_space/environment.h>
#include "kinematic_planning/RKPModelBase.h"
#include "kinematic_planning/RKPConstraintEvaluator.h"

namespace kinematic_planning
{
    
    class StateValidityPredicate : public ompl::SpaceInformation::StateValidityChecker
    {
    public:
        StateValidityPredicate(RKPModelBase *model) : ompl::SpaceInformation::StateValidityChecker()
	{
	    m_model = model;
	}
	
	virtual bool operator()(const ompl::SpaceInformation::State_t state)
	{
	    m_model->kmodel->computeTransforms(static_cast<const ompl::SpaceInformationKinematic::StateKinematic_t>(state)->values, m_model->groupID);
	    m_model->collisionSpace->updateRobotModel(m_model->collisionSpaceID);
	    
	    bool valid = !m_model->collisionSpace->isCollision(m_model->collisionSpaceID);
	    
	    if (valid)
		valid = m_kce.decide();
	    
	    return valid;
	}
	
	void setPoseConstraints(const std::vector<robot_msgs::PoseConstraint> &kc)
	{
	    m_kce.use(m_model->kmodel, kc);
	}
	
	void clearConstraints(void)
	{
	    m_kce.clear();
	}
	
    protected:
	RKPModelBase                    *m_model;
	KinematicConstraintEvaluatorSet  m_kce;
    };  
    
} // kinematic_planning

#endif
    
