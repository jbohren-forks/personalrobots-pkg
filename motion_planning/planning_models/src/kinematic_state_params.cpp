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

#include <planning_models/kinematic_state_params.h>
#include <cassert>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <cstdlib>

planning_models::StateParams::StateParams(KinematicModel *model) : m_owner(model), m_mi(model->getModelInfo()), m_params(NULL)
{
    assert(model->isBuilt());
    m_params = m_mi.stateDimension > 0 ? new double[m_mi.stateDimension] : NULL;
    setAll(0);
    reset();
    if (m_mi.inRobotFrame)
	setInRobotFrame();
}

planning_models::StateParams::StateParams(const StateParams &sp) : m_owner(sp.m_owner), m_mi(sp.m_mi), m_params(NULL)
{
    copyFrom(sp);
}

planning_models::StateParams::~StateParams(void)
{
    if (m_params)
	delete[] m_params;
}

planning_models::StateParams& planning_models::StateParams::operator=(const StateParams &rhs) 
{
    if (this != &rhs)
	copyFrom(rhs);
    return *this;
}

bool planning_models::StateParams::operator==(const StateParams &rhs) const
{
    if (m_mi.stateDimension != rhs.m_mi.stateDimension)
	return false;
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	if (fabs(m_params[i] - rhs.m_params[i]) > DBL_MIN)
	    return false;
    return true;
}

void planning_models::StateParams::copyFrom(const StateParams &sp)
{
    m_owner = sp.m_owner;
    m_mi = sp.m_mi;
    if (m_params)
	delete[] m_params;    
    m_params = m_mi.stateDimension > 0 ? new double[m_mi.stateDimension] : NULL;
    if (m_params)
	for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	    m_params[i] = sp.m_params[i];
    m_seen = sp.m_seen;
}

void planning_models::StateParams::defaultState(void)
{
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
    {
	if (m_mi.stateBounds[2 * i] <= 0.0 && m_mi.stateBounds[2 * i + 1] >= 0.0)
	    m_params[i] = 0.0;
	else
	    m_params[i] = (m_mi.stateBounds[2 * i] + m_mi.stateBounds[2 * i + 1]) / 2.0;
	m_seen[i] = true;
    }
}

void planning_models::StateParams::randomStateGroup(const std::string &group)
{
    randomStateGroup(m_owner->getGroupID(group));
}


void planning_models::StateParams::randomStateGroup(int groupID)
{
    assert(groupID >= 0 && groupID < (int)m_owner->getGroupCount());
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
    {
	unsigned int j = m_mi.groupStateIndexList[groupID][i];
	m_params[j] = (m_mi.stateBounds[2 * j + 1] - m_mi.stateBounds[2 * j]) * ((double)rand() / (RAND_MAX + 1.0)) +  m_mi.stateBounds[2 * j];
	m_seen[j] = true;
    }
}
    
void planning_models::StateParams::randomState(void)
{   
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
    {
	m_params[i] = (m_mi.stateBounds[2 * i + 1] - m_mi.stateBounds[2 * i]) * ((double)rand() / (RAND_MAX + 1.0)) +  m_mi.stateBounds[2 * i];
	m_seen[i] = true;
    }
}

void planning_models::StateParams::perturbStateGroup(double factor, const std::string &group)    
{   
    perturbStateGroup(factor, m_owner->getGroupID(group));
}


void planning_models::StateParams::perturbStateGroup(double factor, int groupID)    
{   
    assert(groupID >= 0 && groupID < (int)m_owner->getGroupCount());
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
    {
	unsigned int j = m_mi.groupStateIndexList[groupID][i];
	m_params[j] += factor * (m_mi.stateBounds[2 * j + 1] - m_mi.stateBounds[2 * j]) * (2.0 * ((double)rand() / (RAND_MAX + 1.0)) - 1.0);
	m_seen[j] = true;
    }
}

void planning_models::StateParams::perturbState(double factor)
{   
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	m_params[i] += factor * (m_mi.stateBounds[2 * i + 1] - m_mi.stateBounds[2 * i]) * (2.0 * ((double)rand() / (RAND_MAX + 1.0)) - 1.0);
    enforceBounds();
}

void planning_models::StateParams::enforceBounds(void)
{  
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
    {
	if (m_params[i] > m_mi.stateBounds[2 * i + 1])
	    m_params[i] = m_mi.stateBounds[2 * i + 1];
	else
	    if (m_params[i] < m_mi.stateBounds[2 * i])
		m_params[i] = m_mi.stateBounds[2 * i];
    }
}

void planning_models::StateParams::reset(void)
{
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	m_seen[i] = false;
}

void planning_models::StateParams::resetGroup(const std::string &group)
{
    resetGroup(m_owner->getGroupID(group));
}

void planning_models::StateParams::resetGroup(int groupID)
{
    assert(groupID >= 0 && groupID < (int)m_owner->getGroupCount());
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
    {
	unsigned int j = m_mi.groupStateIndexList[groupID][i];
	m_seen[j] = false;
    }
}

bool planning_models::StateParams::seenAll(void) const
{
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
    {
	std::map<unsigned int, bool>::const_iterator it = m_seen.find(i);
	if (!it->second)
	    return false;
    }  
    return true;
}

bool planning_models::StateParams::seenAllGroup(const std::string &group) const
{
    return seenAllGroup(m_owner->getGroupID(group));
}

bool planning_models::StateParams::seenAllGroup(int groupID) const
{    
    assert(groupID >= 0 && groupID < (int)m_owner->getGroupCount());
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
    {
	unsigned int j = m_mi.groupStateIndexList[groupID][i];
	std::map<unsigned int, bool>::const_iterator it = m_seen.find(j);
	if (!it->second)
	    return false;
    }
    return true;
}

bool planning_models::StateParams::seenJoint(const std::string &name) const
{
    KinematicModel::Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	int idx = m_mi.parameterIndex[name];
	for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
	{
	    std::map<unsigned int, bool>::const_iterator it = m_seen.find(idx + i);
	    if (!it->second)
		return false;
	}
	return true;
    }
    else
    {
	m_msg.error("Unknown joint: '" + name + "'");
	return false;
    }
}

void planning_models::StateParams::missing(std::ostream &out)
{
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	if (!m_seen[i])
	    out << m_mi.parameterName[i] << " ";
}

const double* planning_models::StateParams::getParamsJoint(const std::string &name) const
{
    KinematicModel::Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
	if (it != m_mi.parameterIndex.end())
	    return m_params + it->second;
	else
	    return NULL;
    }
    else
	return NULL;    
}

bool planning_models::StateParams::setParamsJoint(const std::vector<double> &params, const std::string &name)
{
    bool result = false;
    KinematicModel::Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	double *dparams = new double[joint->usedParams];
	std::copy(params.begin(), params.end(), dparams);	
	result = setParamsJoint(dparams, name);
	delete[] dparams;
    }
    return result;
}

bool planning_models::StateParams::setParamsJoint(const double *params, const std::string &name)
{
    bool result = false;
    KinematicModel::Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	unsigned int pos = m_mi.parameterIndex[name];
	for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
	{
	    unsigned int pos_i = pos + i;
	    if (m_params[pos_i] != params[i] || !m_seen[pos_i])
	    {
		m_params[pos_i] = params[i];
		m_seen[pos_i] = true;		
		result = true;
	    }
	}
    }
    else
	m_msg.error("Unknown joint: '" + name + "'");
    return result;
}

bool planning_models::StateParams::setParams(const std::vector<double> &params)
{ 
    double *dparams = new double[m_mi.stateDimension];
    std::copy(params.begin(), params.end(), dparams);
    bool result = setParams(dparams);
    delete[] dparams;
    return result;
}

bool planning_models::StateParams::setParams(const double *params)
{  
    bool result = false;
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	if (m_params[i] != params[i] || !m_seen[i])
	{
	    m_params[i] = params[i];
	    m_seen[i] = true;
	    result = true;
	}
    return result;
}

bool planning_models::StateParams::setParamsGroup(const std::vector<double> &params, const std::string &group)
{
    return setParamsGroup(params, m_owner->getGroupID(group));
}

bool planning_models::StateParams::setParamsGroup(const std::vector<double> &params, int groupID)
{
    double *dparams = new double[m_owner->getGroupDimension(groupID)];
    std::copy(params.begin(), params.end(), dparams);
    bool result = setParamsGroup(dparams, groupID);
    delete[] dparams;
    return result;
}

bool planning_models::StateParams::setParamsGroup(const double *params, const std::string &group)
{
    return setParamsGroup(params, m_owner->getGroupID(group));
}

bool planning_models::StateParams::setParamsGroup(const double *params, int groupID)
{
    bool result = false;
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
    {
	unsigned int j = m_mi.groupStateIndexList[groupID][i];
	if (m_params[j] != params[i] || !m_seen[j])
	{
	    m_params[j] = params[i];
	    m_seen[j] = true;
	    result = true;
	}
    }
    return result;
}

void planning_models::StateParams::setAllInGroup(const double value, const std::string &group)
{
    setAllInGroup(value, m_owner->getGroupID(group));
}

void planning_models::StateParams::setAllInGroup(const double value, int groupID)
{
    assert(groupID >= 0 && groupID < (int)m_owner->getGroupCount());
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
    {
	unsigned int j = m_mi.groupStateIndexList[groupID][i];
	m_params[j] = value;
	m_seen[j] = true;
    }	
}

void planning_models::StateParams::setAll(const double value)
{
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
    {
	m_params[i] = value;
	m_seen[i] = true;
    }   
}

void planning_models::StateParams::setInRobotFrame(void)
{
    for (unsigned int j = 0 ; j < m_mi.floatingJoints.size() ; ++j)
    {
	double vals[7] = {0, 0, 0, 0, 0, 0, 1};
	setParamsJoint(vals, m_mi.parameterName[m_mi.floatingJoints[j]]);
    }
    
    for (unsigned int j = 0 ; j < m_mi.planarJoints.size() ; ++j)
    {
	double vals[3] = {0, 0, 0};
	setParamsJoint(vals, m_mi.parameterName[m_mi.planarJoints[j]]);
    }
}

const double* planning_models::StateParams::getParams(void) const
{
    return m_params;
}

void planning_models::StateParams::copyParamsJoint(double *params, const std::string &name) const
{
    KinematicModel::Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
	if (it != m_mi.parameterIndex.end())
	{
	    unsigned int pos = it->second;
	    for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
		params[i] = m_params[pos + i];
	    return;
	}
    }
    m_msg.error("Unknown joint: '" + name + "'");
}

void planning_models::StateParams::copyParamsJoint(std::vector<double> &params, const std::string &name) const
{
    KinematicModel::Joint *joint = m_owner->getJoint(name);
    if (joint)
    {
	params.resize(joint->usedParams);
	std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.find(name);
	if (it != m_mi.parameterIndex.end())
	{
	    unsigned int pos = it->second;
	    for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
		params[i] = m_params[pos + i];
	    return;
	}
    }
    m_msg.error("Unknown joint: '" + name + "'");
}

void planning_models::StateParams::copyParams(double *params) const
{
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	params[i] = m_params[i];
}

void planning_models::StateParams::copyParams(std::vector<double> &params) const
{
    params.resize(m_mi.stateDimension);
    for (unsigned int i = 0 ; i < m_mi.stateDimension ; ++i)
	params[i] = m_params[i];
}

void planning_models::StateParams::copyParamsGroup(double *params, const std::string &group) const
{
    copyParamsGroup(params, m_owner->getGroupID(group));
}

void planning_models::StateParams::copyParamsGroup(std::vector<double> &params, const std::string &group) const
{
    copyParamsGroup(params, m_owner->getGroupID(group));
}

void planning_models::StateParams::copyParamsGroup(std::vector<double> &params, int groupID) const
{ 
    unsigned int dim = m_owner->getGroupDimension(groupID);
    double *dparams = new double[dim];
    copyParamsGroup(dparams, groupID);
    params.resize(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	params[i] = dparams[i];
    delete[] dparams;
}

void planning_models::StateParams::copyParamsGroup(double *params, int groupID) const
{   
    assert(groupID >= 0 && groupID < (int)m_owner->getGroupCount());
    for (unsigned int i = 0 ; i < m_mi.groupStateIndexList[groupID].size() ; ++i)
	params[i] = m_params[m_mi.groupStateIndexList[groupID][i]];
}

void planning_models::StateParams::print(std::ostream &out) const
{
    out << std::endl;
    for (std::map<std::string, unsigned int>::const_iterator it = m_mi.parameterIndex.begin() ; it != m_mi.parameterIndex.end() ; ++it)
    {
	KinematicModel::Joint* joint = m_owner->getJoint(it->first);
	if (joint)
	{
	    out << it->first;
	    std::map<unsigned int, bool>::const_iterator sit = m_seen.find(it->second);
	    if (!sit->second)
		out << "[ *** UNSEEN *** ]";
	    out << ": ";
	    for (unsigned int i = 0 ; i < joint->usedParams ; ++i)
		out << m_params[it->second + i] << std::endl;
	}
    }
    out << std::endl;
    for (unsigned int i = 0; i < m_mi.stateDimension ; ++i)
	out << m_params[i] << " ";
    out << std::endl;
}
