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

#include "kinematic_planning/RKPRequestHandler.h"
#include <ros/console.h>
#include <sstream>

/** Check if request looks valid  */
bool kinematic_planning::RKPRequestHandler::isRequestValid(ModelMap &models, motion_planning_srvs::KinematicPlan::Request &req)
{   
    ModelMap::const_iterator pos = models.find(req.params.model_id);
    
    if (pos == models.end())
    {
	ROS_ERROR("Cannot plan for '%s'. Model does not exist", req.params.model_id.c_str());
	return false;
    }
    
    /* find the model */
    RKPModel *m = pos->second;
    
    /* if the user did not specify a planner, use the first available one */
    if (req.params.planner_id.empty())
    {
	if (!m->planners.empty())
	    req.params.planner_id = m->planners.begin()->first;
    }

    /* check if desired planner exists */
    std::map<std::string, RKPPlannerSetup*>::iterator plannerIt = m->planners.find(req.params.planner_id);
    if (plannerIt == m->planners.end())
    {
	ROS_ERROR("Motion planner not found: '%s'", req.params.planner_id.c_str());
	return false;
    }
    
    RKPPlannerSetup *psetup = plannerIt->second;
    
    /* check if the desired distance metric is defined */
    if (psetup->sde.find(req.params.distance_metric) == psetup->sde.end())
    {
	ROS_ERROR("Distance evaluator not found: '%s'", req.params.distance_metric.c_str());
	return false;
    }
    
    // check headers
    for (unsigned int i = 0 ; i < req.goal_constraints.pose_constraint.size() ; ++i)
	if (!m->planningMonitor->getTransformListener()->frameExists(req.goal_constraints.pose_constraint[i].pose.header.frame_id))
	{
	    ROS_ERROR("Frame '%s' is not defined for goal pose constraint message %u", req.goal_constraints.pose_constraint[i].pose.header.frame_id.c_str(), i);
	    return false;
	}
    
    for (unsigned int i = 0 ; i < req.goal_constraints.joint_constraint.size() ; ++i)
	if (!m->planningMonitor->getTransformListener()->frameExists(req.goal_constraints.joint_constraint[i].header.frame_id))
	{
	    ROS_ERROR("Frame '%s' is not defined for goal joint constraint message %u", req.goal_constraints.joint_constraint[i].header.frame_id.c_str(), i);
	    return false;
	}
    
    for (unsigned int i = 0 ; i < req.path_constraints.pose_constraint.size() ; ++i)
	if (!m->planningMonitor->getTransformListener()->frameExists(req.path_constraints.pose_constraint[i].pose.header.frame_id))
	{
	    ROS_ERROR("Frame '%s' is not defined for path pose constraint message %u", req.path_constraints.pose_constraint[i].pose.header.frame_id.c_str(), i);
	    return false;
	}
    
    for (unsigned int i = 0 ; i < req.path_constraints.joint_constraint.size() ; ++i)
	if (!m->planningMonitor->getTransformListener()->frameExists(req.path_constraints.joint_constraint[i].header.frame_id))
	{
	    ROS_ERROR("Frame '%s' is not defined for path joint constraint message %u", req.path_constraints.joint_constraint[i].header.frame_id.c_str(), i);
	    return false;
	}
    
    return true;
}

void kinematic_planning::RKPRequestHandler::configure(const planning_models::StateParams *startState, motion_planning_srvs::KinematicPlan::Request &req, RKPPlannerSetup *psetup)
{
    /* clear memory */
    psetup->si->clearGoal();
    psetup->si->clearStartStates();	

    /* before configuring, we may need to update bounds on the state space */
    static_cast<StateValidityPredicate*>(psetup->si->getStateValidityChecker())->setConstraints(req.path_constraints);

    /* set the workspace volume for planning */
    // only area or volume should go through
    static_cast<SpaceInformationRKPModel*>(psetup->si)->setPlanningArea(req.params.volumeMin.x, req.params.volumeMin.y,
									req.params.volumeMax.x, req.params.volumeMax.y);
    static_cast<SpaceInformationRKPModel*>(psetup->si)->setPlanningVolume(req.params.volumeMin.x, req.params.volumeMin.y, req.params.volumeMin.z,
									  req.params.volumeMax.x, req.params.volumeMax.y, req.params.volumeMax.z);
    
    psetup->si->setStateDistanceEvaluator(psetup->sde[req.params.distance_metric]);
    
    /* set the starting state */
    const unsigned int dim = psetup->si->getStateDimension();
    ompl::sb::State *start = new ompl::sb::State(dim);
    
    if (psetup->model->groupID >= 0)
    {
	/* set the pose of the whole robot */
	psetup->model->kmodel->computeTransforms(startState->getParams());
	psetup->model->collisionSpace->updateRobotModel();
	
	/* extract the components needed for the start state of the desired group */
	startState->copyParamsGroup(start->values, psetup->model->groupID);
    }
    else
	/* the start state is the complete state */
	startState->copyParams(start->values);
    
    psetup->si->addStartState(start);
    
    /* add goal state */
    psetup->si->setGoal(computeGoalFromConstraints(psetup->si, psetup->model, req.goal_constraints));

    /* add bounds for automatic state fixing (in case a state is invalid) */
    std::vector<double> rhoStart(psetup->si->getStateDimension(), 0.05);
    std::vector<double> rhoGoal(rhoStart);
    // in case we have large bounds, we may have a larger area to sample,
    // so we increase it if we can
    GoalToState *gs = dynamic_cast<GoalToState*>(psetup->si->getGoal());
    if (gs)
    {
	std::vector< std::pair<double, double> > bounds = gs->getBounds();
	for (unsigned int i = 0 ; i < rhoGoal.size() ; ++i)
	{
	    double dif = bounds[i].second - bounds[i].first;
	    if (dif > rhoGoal[i])
		rhoGoal[i] = dif;
	}
    }
    psetup->si->fixInvalidInputStates(rhoStart, rhoGoal, 100);
    
    /* print some information */
    ROS_DEBUG("=======================================");
    std::stringstream ss;
    psetup->si->printSettings(ss);
    ss << "Path constraints:" << std::endl;
    static_cast<StateValidityPredicate*>(psetup->si->getStateValidityChecker())->getKinematicConstraintEvaluatorSet().print(ss);
    ROS_DEBUG("%s", ss.str().c_str());
    ROS_DEBUG("=======================================");	
}

bool kinematic_planning::RKPRequestHandler::computePlan(ModelMap &models, const planning_models::StateParams *start, 
							motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res)
{
    if (!isRequestValid(models, req))
	return false;
    
    ROS_INFO("Selected motion planner: '%s'", req.params.planner_id.c_str());
    
    // find the data we need 
    RKPModel *m = models[req.params.model_id];
    
    // get the planner setup
    RKPPlannerSetup *psetup = m->planners[req.params.planner_id];
    
    // configure the planner
    configure(start, req, psetup);
    
    /* compute actual motion plan */
    ompl::sb::PathKinematic *bestPath       = NULL;
    double                   bestDifference = 0.0;
    
    psetup->model->collisionSpace->lock();
    psetup->model->kmodel->lock();
    
    bool approximate = false;    
    callPlanner(psetup, req.times, req.allowed_time, true, bestPath, bestDifference, approximate);
    
    psetup->model->collisionSpace->unlock();
    psetup->model->kmodel->unlock();

    psetup->si->clearGoal();
    psetup->si->clearStartStates();	

    /* copy the solution to the result */
    if (bestPath)
    {
	std::vector<planning_models::KinematicModel::Joint*> joints;
	psetup->model->kmodel->getJoints(joints);
	res.path.start_state.resize(joints.size());
	for (unsigned int i = 0 ; i < joints.size() ; ++i)
	{
	    res.path.start_state[i].header = res.path.header;
	    res.path.start_state[i].joint_name = joints[i]->name;
	    start->copyParamsJoint(res.path.start_state[i].value, joints[i]->name);
	}
	
	res.path.states.resize(bestPath->states.size());
	res.path.times.resize(bestPath->states.size());
	res.path.names.clear();
	psetup->model->kmodel->getJointsInGroup(res.path.names, psetup->model->groupID);

	const unsigned int dim = psetup->si->getStateDimension();
	for (unsigned int i = 0 ; i < bestPath->states.size() ; ++i)
	{
	    res.path.times[i] = i * 0.02;
	    res.path.states[i].vals.resize(dim);
	    for (unsigned int j = 0 ; j < dim ; ++j)
		res.path.states[i].vals[j] = bestPath->states[i]->values[j];
	}
	res.distance = bestDifference;
	res.approximate = approximate ? 1 : 0;
	delete bestPath;
    }
    
    return true;
}

/** Compute the actual motion plan. Return true if computed plan was trivial (start state already in goal region) */
bool kinematic_planning::RKPRequestHandler::callPlanner(RKPPlannerSetup *psetup, int times, double allowed_time, bool interpolate,
							ompl::sb::PathKinematic* &bestPath, double &bestDifference, bool &approximate)
{
    if (times <= 0)
    {
	ROS_ERROR("Motion plan cannot be computed %d times", times);
	return false;
    }
    
    if (dynamic_cast<ompl::sb::GoalRegion*>(psetup->si->getGoal()))
	ROS_DEBUG("Goal threshold is %g", dynamic_cast<ompl::sb::GoalRegion*>(psetup->si->getGoal())->threshold);
    
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
	bestPath = new ompl::sb::PathKinematic(psetup->si);
	
	ompl::sb::State *s0 = new ompl::sb::State(psetup->si->getStateDimension());
	ompl::sb::State *s1 = new ompl::sb::State(psetup->si->getStateDimension());
	psetup->si->copyState(s0, static_cast<ompl::sb::State*>(psetup->si->getStartState(t_index)));
	psetup->si->copyState(s1, static_cast<ompl::sb::State*>(psetup->si->getStartState(t_index)));
	bestPath->states.push_back(s0);
	bestPath->states.push_back(s1);
    }
    else
    {		
	/* do the planning */
	bestPath = NULL;
	bestDifference = 0.0;
	double totalTime = 0.0;
	ompl::base::Goal *goal = psetup->si->getGoal();
	
	for (int i = 0 ; i < times ; ++i)
	{
	    ros::WallTime startTime = ros::WallTime::now();
	    bool ok = psetup->mp->solve(allowed_time); 
	    double tsolve = (ros::WallTime::now() - startTime).toSec();	
	    ROS_DEBUG("%s Motion planner spent %g seconds", (ok ? "[Success]" : "[Failure]"), tsolve);
	    totalTime += tsolve;
	    
	    /* do path smoothing */
	    if (ok)
	    {
		ros::WallTime startTime = ros::WallTime::now();
		ompl::sb::PathKinematic *path = static_cast<ompl::sb::PathKinematic*>(goal->getSolutionPath());
		psetup->smoother->smoothMax(path);
		double tsmooth = (ros::WallTime::now() - startTime).toSec();
		ROS_DEBUG("          Smoother spent %g seconds (%g seconds in total)", tsmooth, tsmooth + tsolve);		    
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
		    ROS_DEBUG("          Obtained better solution: distance is %f", bestDifference);
		}
	    }
	    psetup->mp->clear();	    
	}
	
	ROS_DEBUG("Total planning time: %g; Average planning time: %g", totalTime, (totalTime / (double)times));
    }
    return result;
}
