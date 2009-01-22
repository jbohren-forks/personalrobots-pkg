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
#include "kinematic_planning/RKPModelBase.h"

#include <vector>

namespace kinematic_planning
{
    
    /** This class configures an instance of SpaceInformationKinematic with data from a KinematicModel */
    class SpaceInformationRKPModel : public ompl::SpaceInformationKinematic
    {
    public:
    SpaceInformationRKPModel(RKPModelBase *model, double divisions = 20.0) : SpaceInformationKinematic()
	{	
	    m_kmodel = model->kmodel;
	    m_groupID = model->groupID;
	    m_divisions = divisions;
	    
	    /* compute the state space for this group */
	    m_stateDimension = m_groupID >= 0 ? m_kmodel->groupStateIndexList[m_groupID].size() : m_kmodel->stateDimension;
	    m_stateComponent.resize(m_stateDimension);
	    
	    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    {	
		if (m_stateComponent[i].type == StateComponent::UNKNOWN)
		    m_stateComponent[i].type = StateComponent::NORMAL;
		int p = m_groupID >= 0 ? m_kmodel->groupStateIndexList[m_groupID][i] * 2 : i * 2;
		m_stateComponent[i].minValue   = m_kmodel->stateBounds[p    ];
		m_stateComponent[i].maxValue   = m_kmodel->stateBounds[p + 1];
		m_stateComponent[i].resolution = (m_stateComponent[i].maxValue - m_stateComponent[i].minValue) / m_divisions;
		
		for (unsigned int j = 0 ; j < m_kmodel->floatingJoints.size() ; ++j)
		    if (m_kmodel->floatingJoints[j] == p)
		    {
			m_floatingJoints.push_back(i);
			m_stateComponent[i + 3].type = StateComponent::QUATERNION;
			break;
		    }
		
		for (unsigned int j = 0 ; j < m_kmodel->planarJoints.size() ; ++j)
		    if (m_kmodel->planarJoints[j] == p)
		    {
			m_planarJoints.push_back(i);
			break;		    
		    }
	    }
	    updateResolution();
	}
	
	virtual ~SpaceInformationRKPModel(void)
	{
	}
	
	/** For planar and floating joints, we have infinite
	    dimensions. The bounds for these dimensions are set by the
	    user. */
	void setPlanningVolume(double x0, double y0, double z0, double x1, double y1, double z1)
	{
	    for (unsigned int i = 0 ; i < m_floatingJoints.size() ; ++i)
	    {
		int id = m_floatingJoints[i];		
		m_stateComponent[id    ].minValue = x0;
		m_stateComponent[id    ].maxValue = x1;
		m_stateComponent[id + 1].minValue = y0;
		m_stateComponent[id + 1].maxValue = y1;
		m_stateComponent[id + 2].minValue = z0;
		m_stateComponent[id + 2].maxValue = z1;
		for (int j = 0 ; j < 3 ; ++j)
		    m_stateComponent[j + id].resolution = (m_stateComponent[j + id].maxValue - m_stateComponent[j + id].minValue) / m_divisions;
	    }
	    updateResolution();
	}
	
	void setPlanningArea(double x0, double y0, double x1, double y1)
	{
	    for (unsigned int i = 0 ; i < m_planarJoints.size() ; ++i)
	    {
		int id = m_planarJoints[i];		
		m_stateComponent[id    ].minValue = x0;
		m_stateComponent[id    ].maxValue = x1;
		m_stateComponent[id + 1].minValue = y0;
		m_stateComponent[id + 1].maxValue = y1;
		for (int j = 0 ; j < 2 ; ++j)
		    m_stateComponent[j + id].resolution = (m_stateComponent[j + id].maxValue - m_stateComponent[j + id].minValue) / m_divisions;
	    }
	    updateResolution();
	}
	
    protected:
	
	void updateResolution(void)
	{
	    /* for movement in plane/space, we want to make sure the resolution is small enough */
	    for (unsigned int i = 0 ; i < m_planarJoints.size() ; ++i)
	    {
		if (m_stateComponent[m_planarJoints[i]].resolution > 0.1)
		    m_stateComponent[m_planarJoints[i]].resolution = 0.1;
		if (m_stateComponent[m_planarJoints[i] + 1].resolution > 0.1)
		    m_stateComponent[m_planarJoints[i] + 1].resolution = 0.1;
	    }
	    for (unsigned int i = 0 ; i < m_floatingJoints.size() ; ++i)
	    {
		if (m_stateComponent[m_floatingJoints[i]].resolution > 0.1)
		    m_stateComponent[m_floatingJoints[i]].resolution = 0.1;
		if (m_stateComponent[m_floatingJoints[i] + 1].resolution > 0.1)
		    m_stateComponent[m_floatingJoints[i] + 1].resolution = 0.1;
		if (m_stateComponent[m_floatingJoints[i] + 2].resolution > 0.1)
		    m_stateComponent[m_floatingJoints[i] + 2].resolution = 0.1;
	    }
	}
	
	double                           m_divisions;
	planning_models::KinematicModel *m_kmodel;
	int                              m_groupID;
	std::vector<int>                 m_floatingJoints;
	std::vector<int>                 m_planarJoints;
	
    };    
    
} // kinematic_planning

#endif
    
