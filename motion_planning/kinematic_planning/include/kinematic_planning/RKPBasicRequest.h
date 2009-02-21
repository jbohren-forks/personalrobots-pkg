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

#ifndef KINEMATIC_PLANNING_RKP_BASIC_REQUEST_
#define KINEMATIC_PLANNING_RKP_BASIC_REQUEST_

#include "kinematic_planning/RKPModel.h"

#include <robot_msgs/KinematicPath.h>
#include <robot_msgs/KinematicSpaceParameters.h>
#include <robot_msgs/PoseConstraint.h>

#include <ros/console.h>
#include <iostream>
#include <sstream>

/** Main namespace */
namespace kinematic_planning
{    
   
    static const int R_NONE = 0; // undefined type of request
    
    
    /** This class represents a basic request to a kinematic motion
	planner. The code a request executes is almost the same
	regardless of its type. What differes is the starting state
	specification and the goal specification. */
    template<typename _R>
    class RKPBasicRequest
    {
    public:
	
	RKPBasicRequest(void)
	{
	    m_type = R_NONE;
	    m_activePSetup = NULL;
	}
	
	virtual ~RKPBasicRequest(void)
	{
	}
	
	int getType(void) const
	{
	    return m_type;
	}
		
	/** Set up all the data needed by motion planning based on a request and lock the planner setup
	 *  using this data */
	bool configure(ModelMap &models, const _R &req)
	{
	    // make sure the same motion planner instance is not used by other calls
	    m_activeReq = req;
	    
	    if (!isRequestValid(models, m_activeReq))
		return false;
	    
	    ROS_INFO("Selected motion planner: '%s'", m_activeReq.params.planner_id.c_str());
	    	    
	    /* find the data we need */
	    RKPModel *m = models[m_activeReq.params.model_id];

	    // lock the model
	    m->lock.lock();
	    
	    // get the planner setup
	    m_activePSetup = m->planners[m_activeReq.params.planner_id];

	    update();
	    
	    /* print some information */
	    ROS_INFO("=======================================");
	    std::stringstream ss;
	    m_activePSetup->si->printSettings(ss);
	    static_cast<StateValidityPredicate*>(m_activePSetup->si->getStateValidityChecker())->getKinematicConstraintEvaluatorSet().print(ss);
	    ROS_INFO("%s", ss.str().c_str());
	    ROS_INFO("=======================================");	
	    
	    /* clear memory */
	    cleanupPlanningData(m_activePSetup);
	    
	    return true;
	}

	void release(void)
	{
	    if (m_activePSetup)
	    {
		m_activePSetup->model->lock.unlock();
		m_activePSetup = NULL;
	    }
	}
	
	bool isActive(void) 
	{
	    return m_activePSetup != NULL;
	}
	
	RKPPlannerSetup *activePlannerSetup(void)
	{
	    return m_activePSetup;
	}
	
	_R& activeRequest(void)
	{
	    return m_activeReq;
	}
	
	bool isStillValid(robot_msgs::KinematicPath &path)
	{
	    update();
	    
	    /* copy the path msg into a kinematic path structure (ompl style) */
	    ompl::SpaceInformationKinematic::PathKinematic_t kpath = new ompl::SpaceInformationKinematic::PathKinematic(m_activePSetup->si);
	    unsigned int dim = m_activePSetup->si->getStateDimension();
	    for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	    {
		ompl::SpaceInformationKinematic::StateKinematic_t kstate = new ompl::SpaceInformationKinematic::StateKinematic(dim);
		kpath->states.push_back(kstate);
		for (unsigned int j = 0 ; j < dim ; ++j)
		    kstate->values[j] = path.states[i].vals[j];
	    }
	    
	    m_activePSetup->model->collisionSpace->lock();
	    bool valid = m_activePSetup->si->checkPath(kpath);
	    m_activePSetup->model->collisionSpace->unlock();
	    
	    delete kpath;
	    
	    /* clear memory */
	    cleanupPlanningData(m_activePSetup);
	    
	    return valid;	    
	}
	
	bool isTrivial(double *distance = NULL)
	{
	    update();
	    
	    m_activePSetup->model->collisionSpace->lock();
	    bool trivial = m_activePSetup->mp->isTrivial(NULL, distance);
	    m_activePSetup->model->collisionSpace->unlock();
	    
	    /* clear memory */
	    cleanupPlanningData(m_activePSetup);
	    
	    return trivial;	    
	}
	
	void execute(robot_msgs::KinematicPath &path, double &distance, bool &trivial, bool &approximate)
	{
	    update();
	    
	    /* compute actual motion plan */
	    ompl::SpaceInformationKinematic::PathKinematic_t bestPath       = NULL;
	    double                                           bestDifference = 0.0;	
	    
	    m_activePSetup->model->collisionSpace->lock();
	    trivial = computePlan(m_activePSetup, m_activeReq.times, m_activeReq.allowed_time, m_activeReq.interpolate,
				  bestPath, bestDifference, approximate);
	    m_activePSetup->model->collisionSpace->unlock();
	    
	    /* fill in the results */
	    fillSolution(m_activePSetup, bestPath, bestDifference, path, distance);
	    
	    /* clear memory */
	    cleanupPlanningData(m_activePSetup);
	}

    protected:

	/** Specializations of this class should implement this function */
	bool isRequestValid(ModelMap &models, _R &req)
	{
	    return areSpaceParamsValid(models, req.params);
	}
	
	/** Specializations of this class should implement this function */
	void setupGoalState(RKPPlannerSetup *psetup, _R &req)
	{
	}
	
	void update(void)
	{
	    /* configure state space and starting state */
	    setupStateSpaceAndStartState(m_activePSetup, m_activeReq.params, m_activeReq.start_state);
	    
	    std::vector<robot_msgs::PoseConstraint> cstrs;
	    m_activeReq.constraints.get_pose_vec(cstrs);
	    setupPoseConstraints(m_activePSetup, cstrs);
	    
	    /* add goal state */
	    setupGoalState(m_activePSetup, m_activeReq);	    
	}

	/** Validate common space parameters */
	bool areSpaceParamsValid(const ModelMap &modelsRef, robot_msgs::KinematicSpaceParameters &params) const
	{ 
	    ModelMap::const_iterator pos = modelsRef.find(params.model_id);
	    
	    if (pos == modelsRef.end())
	    {
		ROS_ERROR("Cannot plan for '%s'. Model does not exist", params.model_id.c_str());
		return false;
	    }
	    
	    /* find the model */
	    RKPModel *m = pos->second;
	    
	    /* if the user did not specify a planner, use the first available one */
	    if (params.planner_id.empty())
	    {
		if (!m->planners.empty())
		    params.planner_id = m->planners.begin()->first;
	    }
	    
	    /* check if desired planner exists */
	    std::map<std::string, RKPPlannerSetup*>::iterator plannerIt = m->planners.find(params.planner_id);
	    if (plannerIt == m->planners.end())
	    {
		ROS_ERROR("Motion planner not found: '%s'", params.planner_id.c_str());
		return false;
	    }
	    	    
	    RKPPlannerSetup *psetup = plannerIt->second;
	    
	    /* check if the desired distance metric is defined */
	    if (psetup->sde.find(params.distance_metric) == psetup->sde.end())
	    {
		ROS_ERROR("Distance evaluator not found: '%s'", params.distance_metric.c_str());
		return false;
	    }
	    
	    return true;
	}
	
	/** Configure the state space for a given set of parameters and set the starting state */
	void setupStateSpaceAndStartState(RKPPlannerSetup *psetup,
					  robot_msgs::KinematicSpaceParameters &params,
					  robot_msgs::KinematicState &start_state)
	{
	    /* set the workspace volume for planning */
	    // only area or volume should go through... not sure what happens on really complicated models where 
	    // we have both multiple planar and multiple floating joints... it might work :)
	    static_cast<SpaceInformationRKPModel*>(psetup->si)->setPlanningArea(params.volumeMin.x, params.volumeMin.y,
										params.volumeMax.x, params.volumeMax.y);
	    static_cast<SpaceInformationRKPModel*>(psetup->si)->setPlanningVolume(params.volumeMin.x, params.volumeMin.y, params.volumeMin.z,
										  params.volumeMax.x, params.volumeMax.y, params.volumeMax.z);
	    
	    psetup->si->setStateDistanceEvaluator(psetup->sde[params.distance_metric]);
	    
	    /* set the starting state */
	    const unsigned int dim = psetup->si->getStateDimension();
	    ompl::SpaceInformationKinematic::StateKinematic_t start = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	    
	    if (psetup->model->groupID >= 0)
	    {
		/* set the pose of the whole robot */
		psetup->model->kmodel->computeTransforms(&start_state.vals[0]);
		psetup->model->collisionSpace->updateRobotModel(psetup->model->collisionSpaceID);
		
		/* extract the components needed for the start state of the desired group */
		for (unsigned int i = 0 ; i < dim ; ++i)
		    start->values[i] = start_state.vals[psetup->model->kmodel->getModelInfo().groupStateIndexList[psetup->model->groupID][i]];
	    }
	    else
	    {
		/* the start state is the complete state */
		for (unsigned int i = 0 ; i < dim ; ++i)
		    start->values[i] = start_state.vals[i];
	    }
	    
	    psetup->si->addStartState(start);
	}
	
	/** Set the kinematic constraints to follow */
	void setupPoseConstraints(RKPPlannerSetup *psetup, const std::vector<robot_msgs::PoseConstraint> &cstrs)
	{
	    static_cast<StateValidityPredicate*>(psetup->si->getStateValidityChecker())->setPoseConstraints(cstrs);
	}    
	
	/** Compute the actual motion plan. Return true if computed plan was trivial (start state already in goal region) */
	bool computePlan(RKPPlannerSetup *psetup, int times, double allowed_time, bool interpolate,
			 ompl::SpaceInformationKinematic::PathKinematic_t &bestPath, double &bestDifference, bool &approximate)
	{
	    
	    if (times <= 0)
	    {
		ROS_ERROR("Motion plan cannot be computed %d times", times);
		return false;
	    }

	    if (dynamic_cast<ompl::SpaceInformationKinematic::GoalRegionKinematic_t>(psetup->si->getGoal()))
		ROS_INFO("Goal threshold is %g", dynamic_cast<ompl::SpaceInformationKinematic::GoalRegionKinematic_t>(psetup->si->getGoal())->threshold);

	    unsigned int t_index = 0;
	    double t_distance = 0.0;
	    bool result = psetup->mp->isTrivial(&t_index, &t_distance);
	    
	    if (result)
	    {
		ROS_INFO("Solution already achieved");
		bestDifference = t_distance;
		approximate = false;
		
		/* we want to maintain the invariant that a path will
		   at least consist of start & goal states, so we copy
		   the start state twice */
		bestPath = new ompl::SpaceInformationKinematic::PathKinematic(psetup->si);
		
		ompl::SpaceInformationKinematic::StateKinematic_t s0 = new ompl::SpaceInformationKinematic::StateKinematic(psetup->si->getStateDimension());
		ompl::SpaceInformationKinematic::StateKinematic_t s1 = new ompl::SpaceInformationKinematic::StateKinematic(psetup->si->getStateDimension());
		psetup->si->copyState(s0, static_cast<ompl::SpaceInformationKinematic::StateKinematic_t>(psetup->si->getStartState(t_index)));
		psetup->si->copyState(s1, static_cast<ompl::SpaceInformationKinematic::StateKinematic_t>(psetup->si->getStartState(t_index)));
		bestPath->states.push_back(s0);
		bestPath->states.push_back(s1);
	    }
	    else
	    {		
		/* do the planning */
		bestPath = NULL;
		bestDifference = 0.0;
		double totalTime = 0.0;
		ompl::SpaceInformation::Goal_t goal = psetup->si->getGoal();
		
		for (int i = 0 ; i < times ; ++i)
		{
		    ros::Time startTime = ros::Time::now();
		    bool ok = psetup->mp->solve(allowed_time); 
		    double tsolve = (ros::Time::now() - startTime).toSec();	
		    ROS_INFO("%s Motion planner spent %g seconds", (ok ? "[Success]" : "[Failure]"), tsolve);
		    totalTime += tsolve;
		    
		    /* do path smoothing */
		    if (ok)
		    {
			ros::Time startTime = ros::Time::now();
			ompl::SpaceInformationKinematic::PathKinematic_t path = static_cast<ompl::SpaceInformationKinematic::PathKinematic_t>(goal->getSolutionPath());
			psetup->smoother->smoothMax(path);
			double tsmooth = (ros::Time::now() - startTime).toSec();
			ROS_INFO("          Smoother spent %g seconds (%g seconds in total)", tsmooth, tsmooth + tsolve);		    
			if (interpolate)
			    psetup->si->interpolatePath(path);
			if (bestPath == NULL || bestDifference > goal->getDifference() || 
			    (bestPath && bestDifference == goal->getDifference() && bestPath->states.size() > path->states.size()))
			{
			    if (bestPath)
				delete bestPath;
			    bestPath = path;
			    bestDifference = goal->getDifference();
			    approximate = goal->isApproximate();
			    goal->forgetSolutionPath();
			    ROS_INFO("          Obtained better solution: distance is %f", bestDifference);
			}
		    }
		    psetup->mp->clear();	    
		}
		
		ROS_INFO("Total planning time: %g; Average planning time: %g", totalTime, (totalTime / (double)times));
	    }
	    return result;
	}
	
	void fillSolution(RKPPlannerSetup *psetup, ompl::SpaceInformationKinematic::PathKinematic_t bestPath, double bestDifference,
			  robot_msgs::KinematicPath &path, double &distance)
	{
	    const unsigned int dim = psetup->si->getStateDimension();
	    
	    /* copy the solution to the result */
	    if (bestPath)
	    {
		path.set_states_size(bestPath->states.size());
		for (unsigned int i = 0 ; i < bestPath->states.size() ; ++i)
		{
		    path.states[i].set_vals_size(dim);
		    for (unsigned int j = 0 ; j < dim ; ++j)
			path.states[i].vals[j] = bestPath->states[i]->values[j];
		}
		distance = bestDifference;
		delete bestPath;
	    }
	    else
	    {
		path.set_states_size(0);
		distance = -1.0;
	    }
	}
	
	void cleanupPlanningData(RKPPlannerSetup *psetup)
	{
	    /* cleanup */
	    psetup->si->clearGoal();
	    psetup->si->clearStartStates();	
	}

	// the type of request (unique ID for each type)
	int              m_type;
	
	_R               m_activeReq;
	RKPPlannerSetup *m_activePSetup;
    };
    
} // kinematic_planning

#endif
    
