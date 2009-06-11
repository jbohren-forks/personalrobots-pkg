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

#ifndef KINEMATIC_PLANNING_RKP_SPACE_INFORMATION_
#define KINEMATIC_PLANNING_RKP_SPACE_INFORMATION_

#include <ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h>
#include <motion_planning_msgs/JointConstraint.h>
#include "kinematic_planning/RKPModelBase.h"
#include <boost/shared_ptr.hpp>
#include <vector>

namespace kinematic_planning
{
    
    /** This class configures an instance of SpaceInformationKinematic with data from a KinematicModel */
    class SpaceInformationRKPModel : public ompl::sb::SpaceInformationKinematic
    {
    public:
        SpaceInformationRKPModel(RKPModelBase *model) : SpaceInformationKinematic()
	{	
	    m_kmodel = model->kmodel;
	    m_groupID = model->groupID;
	    m_divisions = 20.0;
	    setupRKP();
	}
	
	virtual ~SpaceInformationRKPModel(void)
	{
	}
	
	/** For planar and floating joints, we have infinite
	    dimensions. The bounds for these dimensions are set by the
	    user. */
	void setPlanningVolume(double x0, double y0, double z0, double x1, double y1, double z1);
	void setPlanningArea(double x0, double y0, double x1, double y1);
	
	/** Reduce bounds on the state space to incorporate joint constraints */
	void setJointConstraints(const std::vector<motion_planning_msgs::JointConstraint> &jc);

	/** Restore the constraints to the ones when the class was instantiated */
	void clearJointConstraints(void);
	
    protected:

	void setupRKP(void);
	void checkResolution(void);
	bool checkBounds(void);
	
	std::vector<ompl::sb::StateComponent> m_basicStateComponent;
	double                                m_divisions;
	planning_models::KinematicModel      *m_kmodel;
	int                                   m_groupID;
	std::vector<int>                      m_floatingJoints;
	std::vector<int>                      m_planarJoints;
	
    };    
    
} // kinematic_planning

#endif
    
