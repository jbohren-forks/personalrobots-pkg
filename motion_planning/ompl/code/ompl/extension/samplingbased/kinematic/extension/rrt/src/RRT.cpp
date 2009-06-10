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

/* \author Ioan Sucan */

#include "ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h"

bool ompl::sb::RRT::solve(double solveTime)
{
    SpaceInformationKinematic *si     = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    GoalRegion                *goal_r = dynamic_cast<GoalRegion*>(si->getGoal()); 
    GoalRegionKinematic       *goal_k = dynamic_cast<GoalRegionKinematic*>(si->getGoal());
    GoalState                 *goal_s = dynamic_cast<GoalState*>(si->getGoal());
    unsigned int                  dim = si->getStateDimension();
    
    if (!goal_s && !goal_r)
    {
	m_msg.error("RRT: Unknown type of goal (or goal undefined)");
	return false;
    }

    bool biasSample = goal_k || goal_s;
    
    time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);

    if (m_nn.size() == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, dynamic_cast<State*>(si->getStartState(i)));
	    if (si->isValid(motion->state))
		m_nn.add(motion);
	    else
	    {
		m_msg.error("RRT: Initial state is invalid!");
		delete motion;
	    }	
	}
    }
    
    if (m_nn.size() == 0)
    {
	m_msg.error("RRT: There are no valid initial states!");
	return false;	
    }    

    m_msg.inform("RRT: Starting with %u states", m_nn.size());
    
    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = INFINITY;
    Motion *rmotion   = new Motion(dim);
    State  *rstate    = rmotion->state;
    State  *xstate    = new State(dim);
    
    while (time_utils::Time::now() < endTime)
    {

	/* sample random state (with goal biasing) */
	if (biasSample && m_rng.uniform(0.0, 1.0) < m_goalBias)
	{
	    if (goal_s)
		si->copyState(rstate, static_cast<State*>(goal_s->state));
	    else
		goal_k->sampleNearGoal(rstate);
	}
	else
	    m_sCore.sample(rstate);

	/* find closest state in the tree */
	Motion *nmotion = m_nn.nearest(rmotion);

	/* find state to add */
	for (unsigned int i = 0 ; i < dim ; ++i)
	{
	    double diff = rmotion->state->values[i] - nmotion->state->values[i];
	    xstate->values[i] = fabs(diff) < range[i] ? rmotion->state->values[i] : nmotion->state->values[i] + diff * m_rho;
	}
	
	if (si->checkMotionSubdivision(nmotion->state, xstate))
	{
	    /* create a motion */
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, xstate);
	    motion->parent = nmotion;

	    m_nn.add(motion);
	    double dist = 0.0;
	    bool solved = goal_r->isSatisfied(motion->state, &dist);
	    if (solved)
	    {
		approxdif = dist;
		solution = motion;
		break;
	    }
	    if (dist < approxdif)
	    {
		approxdif = dist;
		approxsol = motion;
	    }
	}
    }
    
    bool approximate = false;
    if (solution == NULL)
    {
	solution = approxsol;
	approximate = true;
    }
    
    if (solution != NULL)
    {
	/* construct the solution path */
	std::vector<Motion*> mpath;
	while (solution != NULL)
	{
	    mpath.push_back(solution);
	    solution = solution->parent;
	}

	/* set the solution path */
	PathKinematic *path = new PathKinematic(static_cast<SpaceInformation*>(m_si));
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	{   
	    State *st = new State(dim);
	    si->copyState(st, mpath[i]->state);
	    path->states.push_back(st);
	}
	goal_r->setDifference(approxdif);
	goal_r->setSolutionPath(path, approximate);

	if (approximate)
	    m_msg.warn("RRT: Found approximate solution");
    }

    delete xstate;
    delete rmotion;
	
    m_msg.inform("RRT: Created %u states", m_nn.size());
    
    return goal_r->isAchieved();
}
