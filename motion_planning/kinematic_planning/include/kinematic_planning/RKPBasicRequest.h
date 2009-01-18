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
#include <boost/thread/mutex.hpp>
#include <iostream>

/** Main namespace */
namespace kinematic_planning
{
    
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
	}
	
	virtual ~RKPBasicRequest(void)
	{
	}
	
	/** Specializations of this class should implement this function */
	bool isRequestValid(ModelMap &models, _R &req)
	{
	    return areSpaceParamsValid(models, req.params);
	}
	
	/** Specializations of this class should implement this function */
	void setupGoalState(RKPModel *model, RKPPlannerSetup *psetup, _R &req)
	{
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
	    
	    ROS_INFO("Selected motion planner: '%s'", params.planner_id.c_str());
	    
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
	void setupStateSpaceAndStartState(RKPModel *model, RKPPlannerSetup *psetup,
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
	    
	    if (model->groupID >= 0)
	    {
		/* set the pose of the whole robot */
		model->kmodel->computeTransforms(&start_state.vals[0]);
		model->collisionSpace->updateRobotModel(model->collisionSpaceID);
		
		/* extract the components needed for the start state of the desired group */
		for (unsigned int i = 0 ; i < dim ; ++i)
		    start->values[i] = start_state.vals[model->kmodel->groupStateIndexList[model->groupID][i]];
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
	
	/** Compute the actual motion plan */
	void computePlan(RKPPlannerSetup *psetup, int times, double allowed_time, bool interpolate,
			 ompl::SpaceInformationKinematic::PathKinematic_t &bestPath, double &bestDifference)
	{
	    
	    if (times <= 0)
	    {
		ROS_ERROR("Request specifies motion plan cannot be computed %d times", times);
		return;
	    }
	    
	    /* do the planning */
	    bestPath = NULL;
	    bestDifference = 0.0;
	    double totalTime = 0.0;
	    ompl::SpaceInformation::Goal_t goal = psetup->si->getGoal();
	    
	    for (int i = 0 ; i < times ; ++i)
	    {
		ros::Time startTime = ros::Time::now();
		bool ok = psetup->mp->solve(allowed_time); 
		double tsolve = (ros::Time::now() - startTime).to_double();	
		ROS_INFO("%s Motion planner spend %g seconds", (ok ? "[Success]" : "[Failure]"), tsolve);
		totalTime += tsolve;
		
		/* do path smoothing */
		if (ok)
		{
		    ros::Time startTime = ros::Time::now();
		    ompl::SpaceInformationKinematic::PathKinematic_t path = static_cast<ompl::SpaceInformationKinematic::PathKinematic_t>(goal->getSolutionPath());
		    psetup->smoother->smoothMax(path);
		    double tsmooth = (ros::Time::now() - startTime).to_double();
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
			goal->forgetSolutionPath();
			ROS_INFO("          Obtained better solution");
		    }
		}
		psetup->mp->clear();	    
	    }
	    
	    ROS_INFO("\nTotal planning time: %g; Average planning time: %g", totalTime, (totalTime / (double)times));
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
	
	bool execute(ModelMap &models, _R &req, robot_msgs::KinematicPath &path, double &distance)
	{
	    // make sure the same motion planner instance is not used by other calls
	    boost::mutex::scoped_lock(m_lock);
	    
	    if (!isRequestValid(models, req))
		return false;
	    
	    /* find the data we need */
	    RKPModel             *m = models[req.params.model_id];
	    RKPPlannerSetup *psetup = m->planners[req.params.planner_id];
	    
	    /* configure state space and starting state */
	    setupStateSpaceAndStartState(m, psetup, req.params, req.start_state);
	    
	    std::vector<robot_msgs::PoseConstraint> cstrs;
	    req.constraints.get_pose_vec(cstrs);
	    setupPoseConstraints(psetup, cstrs);
	    
	    /* add goal state */
	    setupGoalState(m, psetup, req);
	    
	    /* print some information */
	    ROS_INFO("=======================================");
	    std::stringstream ss;
	    psetup->si->printSettings(ss);
	    ROS_INFO(ss.str().c_str());
	    ROS_INFO("=======================================");	
	    
	    /* compute actual motion plan */
	    ompl::SpaceInformationKinematic::PathKinematic_t bestPath       = NULL;
	    double                                           bestDifference = 0.0;	
	    
	    m->collisionSpace->lock();
	    computePlan(psetup, req.times, req.allowed_time, req.interpolate, bestPath, bestDifference);
	    m->collisionSpace->unlock();
	    
	    /* fill in the results */
	    fillSolution(psetup, bestPath, bestDifference, path, distance);
	    
	    /* clear memory */
	    cleanupPlanningData(psetup);
	    
	    return true;
	}
    protected:
	
	boost::mutex m_lock;
    };
    
} // kinematic_planning

#endif
    
