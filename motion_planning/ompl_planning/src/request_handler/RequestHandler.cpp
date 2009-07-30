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

#include "RequestHandler.h"
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <sstream>
#include <cstdlib>

bool ompl_planning::RequestHandler::isRequestValid(ModelMap &models, motion_planning_msgs::ConvertPoseToJointConstraint::Request &req)
{   
    ModelMap::const_iterator pos = models.find(req.params.model_id);
    
    if (pos == models.end())
    {
	ROS_ERROR("Cannot search for '%s'. Model does not exist", req.params.model_id.c_str());
	return false;
    }
    
    /* find the model */
    Model *m = pos->second;
    
    IKSetup *iksetup = m->ik;
    if (iksetup == NULL)
    {
	ROS_ERROR("No IK setup available for model '%s'", pos->first.c_str());
	return false;
    }
    
    /* check if the desired distance metric is defined */
    if (iksetup->sde.find(req.params.distance_metric) == iksetup->sde.end())
    {
	ROS_ERROR("Distance evaluator not found: '%s'", req.params.distance_metric.c_str());
	return false;
    }
    
    // check headers
    for (unsigned int i = 0 ; i < req.constraints.pose_constraint.size() ; ++i)
	if (!m->planningMonitor->getTransformListener()->frameExists(req.constraints.pose_constraint[i].pose.header.frame_id))
	{
	    ROS_ERROR("Frame '%s' is not defined for pose constraint message %u", req.constraints.pose_constraint[i].pose.header.frame_id.c_str(), i);
	    return false;
	}
    
    for (unsigned int i = 0 ; i < req.constraints.joint_constraint.size() ; ++i)
	if (!m->planningMonitor->getTransformListener()->frameExists(req.constraints.joint_constraint[i].header.frame_id))
	{
	    ROS_ERROR("Frame '%s' is not defined for joint constraint message %u", req.constraints.joint_constraint[i].header.frame_id.c_str(), i);
	    return false;
	}
    
    if (!req.states.empty())
    {
	// make sure all joints are in the group
	for (unsigned int i = 0 ; i < req.names.size() ; ++i)
	{
	    int idx = m->planningMonitor->getKinematicModel()->getJointIndexInGroup(req.names[i], m->groupID);
	    if (idx < 0)
		return false;
	}
	
	if (iksetup->si->getStateDimension() != m->planningMonitor->getKinematicModel()->getJointsDimension(req.names))
	{
	    ROS_ERROR("The state dimension for model '%s' does not match the dimension of the joints defining the hint states", req.params.model_id.c_str());
	    return false;
	}
    }
    
    return true;
}

bool ompl_planning::RequestHandler::isRequestValid(ModelMap &models, motion_planning_msgs::GetMotionPlan::Request &req)
{   
    ModelMap::const_iterator pos = models.find(req.params.model_id);
    
    if (pos == models.end())
    {
	ROS_ERROR("Cannot plan for '%s'. Model does not exist", req.params.model_id.c_str());
	return false;
    }
    
    /* find the model */
    Model *m = pos->second;
    
    /* if the user did not specify a planner, use the first available one */
    if (req.planner_id.empty())
	for (std::map<std::string, PlannerSetup*>::const_iterator it = m->planners.begin() ; it != m->planners.end() ; ++it)
	    if ((req.goal_constraints.pose_constraint.empty() && (it->second->mp->getType() & ompl::base::PLAN_TO_GOAL_STATE) != 0) ||
		(!req.goal_constraints.pose_constraint.empty() && (it->second->mp->getType() & ompl::base::PLAN_TO_GOAL_REGION) != 0))
	    {
		if (req.planner_id.empty())
		    req.planner_id = it->first;
		else
		    if (m->planners[req.planner_id]->priority < it->second->priority ||
			(m->planners[req.planner_id]->priority == it->second->priority && rand() % 2 == 1))
			req.planner_id = it->first;
	    }
    
    /* check if desired planner exists */
    std::map<std::string, PlannerSetup*>::iterator plannerIt = m->planners.end();
    for (std::map<std::string, PlannerSetup*>::iterator it = m->planners.begin() ; it != m->planners.end() ; ++it)
	if (it->first.find(req.planner_id) != std::string::npos)
	{
	    req.planner_id = it->first;
	    plannerIt = it;
	    break;
	}
    
    if (plannerIt == m->planners.end())
    {
	ROS_ERROR("Motion planner not found: '%s'", req.planner_id.c_str());
	return false;
    }
    
    PlannerSetup *psetup = plannerIt->second;
    
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

void ompl_planning::RequestHandler::configure(const planning_models::StateParams *startState, motion_planning_msgs::ConvertPoseToJointConstraint::Request &req, IKSetup *iksetup)
{
    /* clear memory */
    iksetup->si->clearGoal();
    
    // clear clones of environments 
    iksetup->model->clearEnvironmentDescriptions();

    setWorkspaceBounds(req.params, iksetup->model, iksetup->si);
    iksetup->si->setStateDistanceEvaluator(iksetup->sde[req.params.distance_metric]);

    /* set the pose of the whole robot */
    EnvironmentDescription *ed = iksetup->model->getEnvironmentDescription();
    ed->kmodel->computeTransforms(startState->getParams());
    ed->collisionSpace->updateRobotModel();
    
    /* add goal state */
    iksetup->model->planningMonitor->transformConstraintsToFrame(req.constraints, iksetup->model->planningMonitor->getFrameId());
    iksetup->si->setGoal(computeGoalFromConstraints(iksetup->si, iksetup->model, req.constraints));

    /* print some information */
    printSettings(iksetup->si);
}

void ompl_planning::RequestHandler::setWorkspaceBounds(motion_planning_msgs::KinematicSpaceParameters &params, ModelBase *model, ompl::base::SpaceInformation *si)
{
    /* set the workspace volume for planning */
    if (model->planningMonitor->getTransformListener()->frameExists(params.volumeMin.header.frame_id) &&
	model->planningMonitor->getTransformListener()->frameExists(params.volumeMax.header.frame_id))
    {
	bool err = false;
	try
	{
	    model->planningMonitor->getTransformListener()->transformPoint(model->planningMonitor->getFrameId(), params.volumeMin, params.volumeMin);
	    model->planningMonitor->getTransformListener()->transformPoint(model->planningMonitor->getFrameId(), params.volumeMax, params.volumeMax);
	}
	catch(...)
	{
	    err = true;
	}
	if (err)
	    ROS_ERROR("Unable to transform workspace bounds to planning frame");
	else
	{	    
	    // only area or volume should go through
	    if (SpaceInformationKinematicModel *s = dynamic_cast<SpaceInformationKinematicModel*>(si))
	    {
		s->setPlanningArea(params.volumeMin.point.x, params.volumeMin.point.y,
				   params.volumeMax.point.x, params.volumeMax.point.y);
		s->setPlanningVolume(params.volumeMin.point.x, params.volumeMin.point.y, params.volumeMin.point.z,
				     params.volumeMax.point.x, params.volumeMax.point.y, params.volumeMax.point.z);
	    }
	    if (SpaceInformationDynamicModel *s = dynamic_cast<SpaceInformationDynamicModel*>(si))
	    {
		s->setPlanningArea(params.volumeMin.point.x, params.volumeMin.point.y,
				   params.volumeMax.point.x, params.volumeMax.point.y);
		s->setPlanningVolume(params.volumeMin.point.x, params.volumeMin.point.y, params.volumeMin.point.z,
				     params.volumeMax.point.x, params.volumeMax.point.y, params.volumeMax.point.z);
	    }
	}
    }
    else
	ROS_DEBUG("No workspace bounding volume was set");
}

void ompl_planning::RequestHandler::configure(const planning_models::StateParams *startState, motion_planning_msgs::GetMotionPlan::Request &req, PlannerSetup *psetup)
{
    /* clear memory */
    psetup->si->clearGoal();           // goal definitions
    psetup->si->clearStartStates();    // start states
    
    // clear clones of environments 
    psetup->model->clearEnvironmentDescriptions();

    /* before configuring, we may need to update bounds on the state space */
    psetup->model->planningMonitor->transformConstraintsToFrame(req.path_constraints, psetup->model->planningMonitor->getFrameId());
    static_cast<StateValidityPredicate*>(psetup->si->getStateValidityChecker())->setConstraints(req.path_constraints);
    setWorkspaceBounds(req.params, psetup->model, psetup->si);
    psetup->si->setStateDistanceEvaluator(psetup->sde[req.params.distance_metric]);
    
    /* set the starting state */
    const unsigned int dim = psetup->si->getStateDimension();
    ompl::base::State *start = new ompl::base::State(dim);
    
    if (psetup->model->groupID >= 0)
    {
	/* set the pose of the whole robot */
	EnvironmentDescription *ed = psetup->model->getEnvironmentDescription();
	ed->kmodel->computeTransforms(startState->getParams());
	ed->collisionSpace->updateRobotModel();
	
	/* extract the components needed for the start state of the desired group */
	startState->copyParamsGroup(start->values, psetup->model->groupID);
    }
    else
	/* the start state is the complete state */
	startState->copyParams(start->values);
    
    psetup->si->addStartState(start);
    
    /* add goal state */
    psetup->model->planningMonitor->transformConstraintsToFrame(req.goal_constraints, psetup->model->planningMonitor->getFrameId());
    psetup->si->setGoal(computeGoalFromConstraints(psetup->si, psetup->model, req.goal_constraints));

    /* fix invalid input states, if we have any */
    fixInputStates(psetup, 0.02, 50);
    fixInputStates(psetup, 0.05, 50);
    
    /* print some information */
    printSettings(psetup->si);
}

void ompl_planning::RequestHandler::printSettings(ompl::base::SpaceInformation *si)
{
    ROS_DEBUG("=======================================");
    std::stringstream ss;
    si->printSettings(ss);
    static_cast<StateValidityPredicate*>(si->getStateValidityChecker())->printSettings(ss);
    ROS_DEBUG("%s", ss.str().c_str());
    ROS_DEBUG("=======================================");	
}

bool ompl_planning::RequestHandler::fixInputStates(PlannerSetup *psetup, double value, unsigned int count)
{
    /* add bounds for automatic state fixing (in case a state is invalid) */
    std::vector<double> rhoStart(psetup->si->getStateDimension(), value);
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
    
    psetup->model->planningMonitor->getEnvironmentModel()->setVerbose(true);
    bool result = psetup->si->fixInvalidInputStates(rhoStart, rhoGoal, count);
    psetup->model->planningMonitor->getEnvironmentModel()->setVerbose(false);
    
    return result;
}

bool ompl_planning::RequestHandler::findState(ModelMap &models, const planning_models::StateParams *start, motion_planning_msgs::ConvertPoseToJointConstraint::Request &req,
					      motion_planning_msgs::ConvertPoseToJointConstraint::Response &res)
{    
    if (!isRequestValid(models, req))
	return false;
        
    // find the data we need 
    Model *m = models[req.params.model_id];
    IKSetup *iksetup = m->ik;
    
    // copy the hint states to OMPL datastructures
    const unsigned int dim = iksetup->si->getStateDimension();
    ompl::base::State *goal = new ompl::base::State(dim);
    std::vector<ompl::base::State*> hints;
    planning_models::StateParams hs(*start);
    unsigned int sdim = m->planningMonitor->getKinematicModel()->getJointsDimension(req.names);
    for (unsigned int i = 0 ; i < req.states.size() ; ++i)
    {
	if (req.states[i].vals.size() != sdim)
	{
	    ROS_ERROR("Incorrect number of parameters for hint state at index %u. Expected %u, got %d.", i, sdim, (int)req.states[i].vals.size());
	    continue;
	}
	ompl::base::State *st = new ompl::base::State(dim);
	hs.setParamsJoints(req.states[i].vals, req.names);
	hs.copyParamsGroup(st->values, m->groupID);
	std::stringstream ss;
	ss << "Hint state: ";
	iksetup->si->printState(st, ss);	
	ROS_DEBUG("%s", ss.str().c_str());
	hints.push_back(st);
    }
    
    m->planningMonitor->getEnvironmentModel()->lock();
    m->planningMonitor->getKinematicModel()->lock();
    
    configure(start, req, iksetup);
    ros::WallTime startTime = ros::WallTime::now();
    bool found = iksetup->gaik->solve(req.allowed_time, goal, hints);
    double stime = (ros::WallTime::now() - startTime).toSec();
    m->planningMonitor->getEnvironmentModel()->unlock();
    m->planningMonitor->getKinematicModel()->unlock();
    
    ROS_DEBUG("Spent %f seconds searching for state", stime);
    
    if (found)
    {
	int u = 0;
	std::vector<std::string> jnames;
	std::stringstream ss;
	m->planningMonitor->getKinematicModel()->getJointsInGroup(jnames, m->groupID);
	res.joint_constraint.resize(jnames.size());
	for (unsigned int i = 0; i < jnames.size() ; ++i)
	{
	    res.joint_constraint[i].header.frame_id = m->planningMonitor->getFrameId();
	    res.joint_constraint[i].header.stamp = m->planningMonitor->lastMapUpdate();
	    res.joint_constraint[i].joint_name = jnames[i];
	    planning_models::KinematicModel::Joint *joint = m->planningMonitor->getKinematicModel()->getJoint(jnames[i]);
	    res.joint_constraint[i].value.resize(joint->usedParams);
	    res.joint_constraint[i].tolerance_above.resize(joint->usedParams, 0.0);
	    res.joint_constraint[i].tolerance_below.resize(joint->usedParams, 0.0);
	    for (unsigned int j = 0 ; j < joint->usedParams ; ++j)
	    {
		res.joint_constraint[i].value[j] = goal->values[j + u];
		ss << goal->values[j + u] << " ";
	    }
	    u += joint->usedParams;
	}
	ROS_DEBUG("Solution was found: %s", ss.str().c_str());
    }
    else
	ROS_DEBUG("No solution was found");
    
    iksetup->si->clearGoal();
    delete goal;
    for (unsigned int i = 0 ; i < hints.size() ; ++i)
	delete hints[i];
    
    return true;
}

bool ompl_planning::RequestHandler::computePlan(ModelMap &models, const planning_models::StateParams *start, 
						motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
    if (!isRequestValid(models, req))
	return false;
    
    // find the data we need 
    Model *m = models[req.params.model_id];
    
    // get the planner setup
    PlannerSetup *psetup = m->planners[req.planner_id];
    
    ROS_INFO("Selected motion planner: '%s', with priority %d", req.planner_id.c_str(), psetup->priority);
    
    m->planningMonitor->getEnvironmentModel()->lock();
    m->planningMonitor->getKinematicModel()->lock();

    // configure the planner
    configure(start, req, psetup);
    
    /* compute actual motion plan */
    Solution sol;
    sol.path = NULL;
    sol.difference = 0.0;
    sol.approximate = false;
    callPlanner(psetup, req.times, req.allowed_time, sol);
    
    psetup->model->planningMonitor->getEnvironmentModel()->unlock();
    psetup->model->planningMonitor->getKinematicModel()->unlock();

    psetup->si->clearGoal();
    psetup->si->clearStartStates();

    /* copy the solution to the result */
    if (sol.path)
    {
	fillResult(psetup, start, res, sol);
	delete sol.path;
	if (!sol.approximate)
	{
	    psetup->priority++;
	    if (psetup->priority > (int)m->planners.size())
	        psetup->priority = m->planners.size();
	}
    }
    
    if (!sol.path || sol.approximate)
    {
	psetup->priority--;
	if (psetup->priority < -(int)m->planners.size())
	    psetup->priority = -m->planners.size();
    }
    ROS_DEBUG("New motion priority for  '%s' is %d", req.planner_id.c_str(), psetup->priority);
    return true;
}

void ompl_planning::RequestHandler::fillResult(PlannerSetup *psetup, const planning_models::StateParams *start, motion_planning_msgs::GetMotionPlan::Response &res, const Solution &sol)
{   
    std::vector<planning_models::KinematicModel::Joint*> joints;
    psetup->model->planningMonitor->getKinematicModel()->getJoints(joints);
    res.path.start_state.resize(joints.size());
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
    {
	res.path.start_state[i].header = res.path.header;
	res.path.start_state[i].joint_name = joints[i]->name;
	start->copyParamsJoint(res.path.start_state[i].value, joints[i]->name);
    }
    
    ompl::kinematic::PathKinematic *kpath = dynamic_cast<ompl::kinematic::PathKinematic*>(sol.path);
    if (kpath)
    {
	res.path.states.resize(kpath->states.size());
	res.path.times.resize(kpath->states.size());
	res.path.names.clear();
	psetup->model->planningMonitor->getKinematicModel()->getJointsInGroup(res.path.names, psetup->model->groupID);
	
	const unsigned int dim = psetup->si->getStateDimension();
	for (unsigned int i = 0 ; i < kpath->states.size() ; ++i)
	{
	    res.path.times[i] = i * 0.02;
	    res.path.states[i].vals.resize(dim);
	    for (unsigned int j = 0 ; j < dim ; ++j)
		res.path.states[i].vals[j] = kpath->states[i]->values[j];
	}
    }
    
    ompl::dynamic::PathDynamic *dpath = dynamic_cast<ompl::dynamic::PathDynamic*>(sol.path);
    if (dpath)
    {
	res.path.states.resize(dpath->states.size());
	res.path.times.resize(dpath->states.size());
	res.path.names.clear();
	psetup->model->planningMonitor->getKinematicModel()->getJointsInGroup(res.path.names, psetup->model->groupID);
	
	const unsigned int dim = psetup->si->getStateDimension();
	for (unsigned int i = 0 ; i < dpath->states.size() ; ++i)
	{
	    res.path.times[i] = i * 0.02;
	    res.path.states[i].vals.resize(dim);
	    for (unsigned int j = 0 ; j < dim ; ++j)
		res.path.states[i].vals[j] = dpath->states[i]->values[j];
	}
    }
    assert(kpath || dpath);
    
    res.distance = sol.difference;
    res.approximate = sol.approximate ? 1 : 0;
}

bool ompl_planning::RequestHandler::callPlanner(PlannerSetup *psetup, int times, double allowed_time, Solution &sol)
{
    if (times <= 0)
    {
	ROS_ERROR("Motion plan cannot be computed %d times", times);
	return false;
    }
    
    if (dynamic_cast<ompl::base::GoalRegion*>(psetup->si->getGoal()))
	ROS_DEBUG("Goal threshold is %g", dynamic_cast<ompl::base::GoalRegion*>(psetup->si->getGoal())->threshold);
    
    unsigned int t_index = 0;
    double t_distance = 0.0;
    bool result = psetup->mp->isTrivial(&t_index, &t_distance);
    
    if (result)
    {
	ROS_INFO("Solution already achieved");
	sol.difference = t_distance;
	sol.approximate = false;
	
	/* we want to maintain the invariant that a path will
	   at least consist of start & goal states, so we copy
	   the start state twice */
	ompl::kinematic::PathKinematic *kpath = new ompl::kinematic::PathKinematic(psetup->si);
	ompl::base::State *s0 = new ompl::base::State(psetup->si->getStateDimension());
	ompl::base::State *s1 = new ompl::base::State(psetup->si->getStateDimension());
	psetup->si->copyState(s0, psetup->si->getStartState(t_index));
	psetup->si->copyState(s1, psetup->si->getStartState(t_index));
	kpath->states.push_back(s0);
	kpath->states.push_back(s1);
	sol.path = kpath;
    }
    else
    {		
	/* do the planning */
	sol.path = NULL;
	sol.difference = 0.0;
	double totalTime = 0.0;
	ompl::base::Goal *goal = psetup->si->getGoal();
	
	for (int i = 0 ; i < times ; ++i)
	{
	    ros::WallTime startTime = ros::WallTime::now();
	    bool ok = psetup->mp->solve(allowed_time); 
	    double tsolve = (ros::WallTime::now() - startTime).toSec();	
	    ROS_DEBUG("%s Motion planner spent %g seconds", (ok ? "[Success]" : "[Failure]"), tsolve);
	    totalTime += tsolve;
	    

	    if (ok)
	    {
		/* do path smoothing, if we are doing kinematic planning */
		if (psetup->smoother)
		{
		    ompl::kinematic::PathKinematic *path = dynamic_cast<ompl::kinematic::PathKinematic*>(goal->getSolutionPath());
		    if (path)
		    {
			ros::WallTime startTime = ros::WallTime::now();
			psetup->smoother->smoothMax(path);
			double tsmooth = (ros::WallTime::now() - startTime).toSec();
			ROS_DEBUG("          Smoother spent %g seconds (%g seconds in total)", tsmooth, tsmooth + tsolve);
			dynamic_cast<SpaceInformationKinematicModel*>(psetup->si)->interpolatePath(path, 0.3);
		    }
		}
		
		if (sol.path == NULL || sol.difference > goal->getDifference() || 
		    (sol.path && sol.difference == goal->getDifference() && sol.path->length() > goal->getSolutionPath()->length()))
		{
		    if (sol.path)
			delete sol.path;
		    sol.path = goal->getSolutionPath();
		    sol.difference = goal->getDifference();
		    sol.approximate = goal->isApproximate();
		    goal->forgetSolutionPath();
		    ROS_DEBUG("          Obtained better solution: distance is %f", sol.difference);
		}
	    }

	    if (debug_)
		display(psetup);
	    
	    psetup->mp->clear();	    
	}
	
	ROS_DEBUG("Total planning time: %g; Average planning time: %g", totalTime, (totalTime / (double)times));
    }
    return result;
}

void ompl_planning::RequestHandler::enableDebugMode(int idx1, int idx2, int idx3)
{
    px_ = idx1;
    py_ = idx2;
    pz_ = idx3;
    debug_ = true;
    displayPublisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void ompl_planning::RequestHandler::disableDebugMode(void)
{
    debug_ = false;
    displayPublisher_.shutdown();
}

void ompl_planning::RequestHandler::display(PlannerSetup *psetup)
{
    int dim = psetup->si->getStateDimension();
    if (px_ >= dim || py_ >= dim || pz_ >= dim)
    {
	ROS_WARN("Display projection out of bounds. Not publishing markers");
	return;
    }
    
    std::vector<const ompl::base::State*> states;
    psetup->mp->getStates(states);
    
    if (states.empty())
	return;
    
    visualization_msgs::Marker mk;        
    mk.header.stamp = psetup->model->planningMonitor->lastMechanismStateUpdate();
    mk.header.frame_id = psetup->model->planningMonitor->getFrameId();
    mk.ns = ros::this_node::getName();
    mk.id = 1;    
    mk.type = visualization_msgs::Marker::POINTS;
    mk.action = visualization_msgs::Marker::ADD;
    mk.lifetime = ros::Duration(0);
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.04;
    mk.color.b = 0.04;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    mk.scale.x = 0.01;
    mk.scale.y = 0.01;
    mk.scale.z = 0.01;
    mk.points.resize(states.size());
    
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	mk.points[i].x = px_ >= 0 ? states[i]->values[px_] : 0.0;
	mk.points[i].y = py_ >= 0 ? states[i]->values[py_] : 0.0;
	mk.points[i].z = pz_ >= 0 ? states[i]->values[pz_] : 0.0;
    }
    
    displayPublisher_.publish(mk);
    ROS_INFO("Published %d points in the diffusion tree", (int)states.size());
}
