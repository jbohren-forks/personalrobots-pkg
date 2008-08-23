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

/** \Author Ioan Sucan */

/**

@mainpage

@htmlinclude ../manifest.html

@b KinematicPlanning is a node capable of planning kinematic paths for
a set of robot models.

<hr>

@section usage Usage
@verbatim
$ kinematic_planning robot_model [standard ROS args]
@endverbatim

@par Example

@verbatim
$ kinematic_planning robotdesc/pr2
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Additional subscriptions due to inheritance from NodeCollisionModel:
- @b localizedpose/RobotBase2DOdom : localized position of the robot base
- @b world_3d_map/PointCloudFloat32 : point cloud with data describing the 3D environment

Publishes to (name/type):
- None

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):
- @b "plan_kinematic_path"/KinematicMotionPlan : given a robot model, starting ang goal states, this service computes a collision free path

<hr>

@section parameters ROS parameters
- None

**/

#include <planning_node_util/cnode.h>
#include <robot_srvs/KinematicPlanState.h>
#include <robot_srvs/KinematicPlanLinkPosition.h>

#include <ompl/extension/samplingbased/kinematic/PathSmootherKinematic.h>
#include <ompl/extension/samplingbased/kinematic/extension/rrt/LazyRRT.h>
#include <ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h>

#include "KinematicPlanningXMLModel.h"

#include <profiling_utils/profiler.h>
#include <string_utils/string_utils.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

class KinematicPlanning : public ros::node,
			  public planning_node_util::NodeCollisionModel
{
public:

    KinematicPlanning(const std::string &robot_model) : ros::node("kinematic_planning"),
							planning_node_util::NodeCollisionModel(dynamic_cast<ros::node*>(this),
											       robot_model)
    {
	advertise_service("plan_kinematic_path_state", &KinematicPlanning::planToState);
	advertise_service("plan_kinematic_path_position", &KinematicPlanning::planToPosition);
    }
    
    virtual ~KinematicPlanning(void)
    {
	for (std::map<std::string, XMLModel*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
    }
    
    bool areSpaceParamsValid(robot_msgs::KinematicSpaceParameters &params)
    {
	if (m_models.find(params.model_id) == m_models.end())
	{
	    std::cerr << "Cannot plan for '" << params.model_id << "'. Model does not exist" << std::endl;
	    return false;	    
	}

	/* find the model */
	XMLModel *m = m_models[params.model_id];
	
	/* if the user did not specify a planner, use the first available one */
	if (params.planner_id.empty())
	{
	    if (!m->planners.empty())
		params.planner_id = m->planners.begin()->first;
	}
	
	/* check if desired planner exists */
	std::map<std::string, KinematicPlannerSetup>::iterator plannerIt = m->planners.find(params.planner_id);
	if (plannerIt == m->planners.end())
	{
	    std::cerr << "Motion planner not found: '" << params.planner_id << "'" << std::endl;
	    return false;
	}
	
	KinematicPlannerSetup &psetup = plannerIt->second;

	/* check if the desired distance metric is defined */
	if (psetup.sde.find(params.distance_metric) == psetup.sde.end())
	{
	    std::cerr << "Distance evaluator not found: '" << params.distance_metric << "'" << std::endl;
	    return false;
	}
	return true;
    }
    
    bool isRequestValid(robot_srvs::KinematicPlanState::request &req)
    {
	if (!areSpaceParamsValid(req.params))
	    return false;

	XMLModel *m = m_models[req.params.model_id];
	KinematicPlannerSetup &psetup = m->planners[req.params.planner_id];
	
	if (m->kmodel->stateDimension != req.start_state.get_vals_size())
	{
	    std::cerr << "Dimension of start state expected to be " << m->kmodel->stateDimension << " but was received as " << req.start_state.get_vals_size() << std::endl;
	    return false;
	}
	
	if (psetup.si->getStateDimension() != req.goal_state.get_vals_size())
	{
	    std::cerr << "Dimension of start goal expected to be " << psetup.si->getStateDimension() << " but was received as " <<  req.goal_state.get_vals_size() << std::endl;
	    return false;
	}

	return true;
    }

    bool isRequestValid(robot_srvs::KinematicPlanLinkPosition::request &req)
    {
	if (!areSpaceParamsValid(req.params))
	    return false;
	
	XMLModel *m = m_models[req.params.model_id];
	
	if (m->kmodel->stateDimension != req.start_state.get_vals_size())
	{
	    std::cerr << "Dimension of start state expected to be " << m->kmodel->stateDimension << " but was received as " << req.start_state.get_vals_size() << std::endl;
	    return false;
	}
	
	return true;
    }

    void setupStateSpaceAndStartState(XMLModel *m, KinematicPlannerSetup &p,
				      robot_msgs::KinematicSpaceParameters &params,
				      robot_msgs::KinematicState &start_state)
    {
	/* set the workspace volume for planning */
	// only area or volume should go through... not sure what happens on really complicated models where 
	// we have both multiple planar and multiple floating joints...
	static_cast<SpaceInformationXMLModel*>(p.si)->setPlanningArea(params.volumeMin.x, params.volumeMin.y,
								      params.volumeMax.x, params.volumeMax.y);
        static_cast<SpaceInformationXMLModel*>(p.si)->setPlanningVolume(params.volumeMin.x, params.volumeMin.y, params.volumeMin.z,
									params.volumeMax.x, params.volumeMax.y, params.volumeMax.z);

	p.si->setStateDistanceEvaluator(p.sde[params.distance_metric]);
	
	/* set the starting state */
	const unsigned int dim = p.si->getStateDimension();
	ompl::SpaceInformationKinematic::StateKinematic_t start = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	
	if (m->groupID >= 0)
	{
	    /* set the pose of the whole robot */
	    m->kmodel->computeTransforms(start_state.vals);
	    m->collisionSpace->updateRobotModel(m->collisionSpaceID);
	    
	    /* extract the components needed for the start state of the desired group */
	    for (unsigned int i = 0 ; i < dim ; ++i)
		start->values[i] = start_state.vals[m->kmodel->groupStateIndexList[m->groupID][i]];
	}
	else
	{
	    /* the start state is the complete state */
	    for (unsigned int i = 0 ; i < dim ; ++i)
		start->values[i] = start_state.vals[i];
	}
	
	p.si->addStartState(start);
    }
    
    void setupGoalState(KinematicPlannerSetup &p, robot_srvs::KinematicPlanState::request &req)
    {
	/* set the goal */
	ompl::SpaceInformationKinematic::GoalStateKinematic_t goal = new ompl::SpaceInformationKinematic::GoalStateKinematic(p.si);
	const unsigned int dim = p.si->getStateDimension();
	goal->state = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	for (unsigned int i = 0 ; i < dim ; ++i)
	    goal->state->values[i] = req.goal_state.vals[i];
	goal->threshold = req.threshold;
	p.si->setGoal(goal);
    }    
    
    void setupGoalState(KinematicPlannerSetup &p, robot_srvs::KinematicPlanLinkPosition::request &req)
    {
	/* set the goal */
	/*
	ompl::SpaceInformationKinematic::GoalStateKinematic_t goal = new ompl::SpaceInformationKinematic::GoalStateKinematic(p.si);
	const unsigned int dim = p.si->getStateDimension();
	goal->state = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	for (unsigned int i = 0 ; i < dim ; ++i)
	    goal->state->values[i] = req.goal_state.vals[i];
	goal->threshold = req.threshold;
	p.si->setGoal(goal); */
    }    
    
    void computePlan(KinematicPlannerSetup &p, int times, double allowed_time, bool interpolate,
		     ompl::SpaceInformationKinematic::PathKinematic_t &bestPath, double &bestDifference)
    {
		
	if (times <= 0)
	{
	    std::cerr << "Request specifies motion plan cannot be computed " << times << " times" << std::endl;
	    return;
	}

	/* do the planning */
	bestPath = NULL;
	bestDifference = 0.0;
        double totalTime = 0.0;
	ompl::SpaceInformation::Goal_t goal = p.si->getGoal();
	
	m_collisionSpace->lock();
	
	profiling_utils::Profiler::Start();

	for (int i = 0 ; i < times ; ++i)
	{
	    ros::Time startTime = ros::Time::now();
	    bool ok = p.mp->solve(allowed_time); 
	    double tsolve = (ros::Time::now() - startTime).to_double();	
	    std::cout << (ok ? "[Success]" : "[Failure]") << " Motion planner spent " << tsolve << " seconds" << std::endl;
	    totalTime += tsolve;
	    
	    /* do path smoothing */
	    if (ok)
	    {
		ros::Time startTime = ros::Time::now();
		ompl::SpaceInformationKinematic::PathKinematic_t path = static_cast<ompl::SpaceInformationKinematic::PathKinematic_t>(goal->getSolutionPath());
		p.smoother->smoothVertices(path);
		double tsmooth = (ros::Time::now() - startTime).to_double();
		std::cout << "          Smoother spent " << tsmooth << " seconds (" << (tsmooth + tsolve) << " seconds in total)" << std::endl;
		if (interpolate)
		    p.si->interpolatePath(path);
		if (bestPath == NULL || bestDifference > goal->getDifference() || 
		    (bestPath && bestDifference == goal->getDifference() && bestPath->states.size() > path->states.size()))
		{
		    if (bestPath)
			delete bestPath;
		    bestPath = path;
		    bestDifference = goal->getDifference();
		    goal->forgetSolutionPath();
		    std::cout << "          Obtained better solution" << std::endl;
		}
	    }
	    p.mp->clear();	    
	}
	
	profiling_utils::Profiler::Stop();
	m_collisionSpace->unlock();
	
        std::cout << std::endl << "Total planning time: " << totalTime << "; Average planning time: " << (totalTime / (double)times) << " (seconds)" << std::endl;
    }

    void fillSolution(KinematicPlannerSetup &p, ompl::SpaceInformationKinematic::PathKinematic_t bestPath, double bestDifference,
		      robot_msgs::KinematicPath &path, double &distance)
    {
	const unsigned int dim = p.si->getStateDimension();
	
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
    
    void cleanupPlanningData(KinematicPlannerSetup &p)
    {
	/* cleanup */
	p.si->clearGoal();
	p.si->clearStartStates();	
    }
    
    bool planToState(robot_srvs::KinematicPlanState::request &req, robot_srvs::KinematicPlanState::response &res)
    {
	if (!isRequestValid(req))
	    return false;
	
	/* find the data we need */
	XMLModel                   *m = m_models[req.params.model_id];
	KinematicPlannerSetup &psetup = m->planners[req.params.planner_id];
	
	/* configure state space and starting state */
	setupStateSpaceAndStartState(m, psetup, req.params, req.start_state);

	/* add goal state */
	setupGoalState(psetup, req);
	
	/* print some information */
	printf("=======================================\n");
	psetup.si->printSettings();
	printf("=======================================\n");	
	
	/* compute actual motion plan */
	ompl::SpaceInformationKinematic::PathKinematic_t bestPath       = NULL;
	double                                           bestDifference = 0.0;	
	computePlan(psetup, req.times, req.allowed_time, req.interpolate, bestPath, bestDifference);
	
	/* fill in the results */
	fillSolution(psetup, bestPath, bestDifference, res.path, res.distance);
	
	/* clear memory */
	cleanupPlanningData(psetup);
	
	return true;
    }

    bool planToPosition(robot_srvs::KinematicPlanLinkPosition::request &req, robot_srvs::KinematicPlanLinkPosition::response &res)
    {
	if (!isRequestValid(req))
	    return false;
	
	/* find the data we need */
	XMLModel                   *m = m_models[req.params.model_id];
	KinematicPlannerSetup &psetup = m->planners[req.params.planner_id];
	
	/* configure state space and starting state */
	setupStateSpaceAndStartState(m, psetup, req.params, req.start_state);

	/* add goal state */
	setupGoalState(psetup, req);
	
	/* print some information */
	printf("=======================================\n");
	psetup.si->printSettings();
	printf("=======================================\n");	
	
	/* compute actual motion plan */
	ompl::SpaceInformationKinematic::PathKinematic_t bestPath       = NULL;
	double                                           bestDifference = 0.0;	
	computePlan(psetup, req.times, req.allowed_time, req.interpolate, bestPath, bestDifference);
	
	/* fill in the results */
	fillSolution(psetup, bestPath, bestDifference, res.path, res.distance);
	
	/* clear memory */
	cleanupPlanningData(psetup);
	
	return true;
    }

    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	planning_node_util::NodeCollisionModel::setRobotDescription(file);	
	defaultPosition();
	
	/* set the data for the model */
	XMLModel *model = new XMLModel();
	model->collisionSpaceID = 0;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = m_kmodel;
	model->groupName = m_kmodel->name;
	createMotionPlanningInstances(model);
	
	/* remember the model by the robot's name */
	m_models[model->groupName] = model;
	
	printf("=======================================\n");	
	m_kmodel->printModelInfo();
	printf("=======================================\n");

	/* create a model for each group */
	std::vector<std::string> groups;
	m_kmodel->getGroups(groups);

	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    XMLModel *model = new XMLModel();
	    model->collisionSpaceID = 0;
	    model->collisionSpace = m_collisionSpace;
	    model->kmodel = m_kmodel;
	    model->groupID = m_kmodel->getGroupID(groups[i]);
	    model->groupName = groups[i];
	    createMotionPlanningInstances(model);
	    m_models[model->groupName] = model;
	}
    }
    
    void knownModels(std::vector<std::string> &model_ids)
    {
	for (std::map<std::string, XMLModel*>::const_iterator i = m_models.begin() ; i != m_models.end() ; ++i)
	    model_ids.push_back(i->first);
    }
    
private:
    
    class StateValidityPredicate : public ompl::SpaceInformation::StateValidityChecker
    {
    public:
	StateValidityPredicate(XMLModel *model) : ompl::SpaceInformation::StateValidityChecker()
	{
	    m_model = model;
	    m_kc = NULL;
	}
	
	virtual bool operator()(const ompl::SpaceInformation::State_t state)
	{
	    m_model->kmodel->computeTransforms(static_cast<const ompl::SpaceInformationKinematic::StateKinematic_t>(state)->values, m_model->groupID);
	    m_model->collisionSpace->updateRobotModel(m_model->collisionSpaceID);
	    
	    bool collision = m_model->collisionSpace->isCollision(m_model->collisionSpaceID);
	    return !collision;
	}
	
	void setConstraints(robot_msgs::KinematicConstraint &kc)
	{
	    m_kc = &kc;
	}
	
    protected:
	robot_msgs::KinematicConstraint *m_kc;
	XMLModel                        *m_model;	
    };

    robot_desc::URDF::Group* getURDFGroup(const std::string &gname)
    {
	std::string urdfGroup = gname;
	urdfGroup.erase(0, urdfGroup.find_last_of(":") + 1);
	return m_urdf->getGroup(urdfGroup);
    }

    /* instantiate the planners that can be used  */
    void createMotionPlanningInstances(XMLModel* model)
    {
	addRRTInstance(model);
	addLazyRRTInstance(model);
    }

    /* instantiate and configure RRT */
    void addRRTInstance(XMLModel *model)
    {
	KinematicPlannerSetup p;
	std::cout << "Adding RRT instance for motion planning: " << model->groupName << std::endl;
	
	p.si       = new SpaceInformationXMLModel(model->kmodel, model->groupID);
	p.svc      = new StateValidityPredicate(model);
	p.si->setStateValidityChecker(p.svc);
	
	p.smoother = new ompl::PathSmootherKinematic(p.si);
	p.smoother->setMaxSteps(20);
	
	ompl::RRT_t rrt = new ompl::RRT(p.si);
	p.mp            = rrt;
	
	robot_desc::URDF::Group *group = getURDFGroup(model->groupName);
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    std::map<std::string, std::string> info = data.getMapTagValues("plan", "RRT");
	    
	    if (info.find("range") != info.end())
	    {
		double range = string_utils::fromString<double>(info["range"]);
		rrt->setRange(range);
		std::cout << "Range is set to " << range << std::endl;		
	    }
	    
	    if (info.find("goal_bias") != info.end())
	    {	
		double bias = string_utils::fromString<double>(info["goal_bias"]);
		rrt->setGoalBias(bias);
		std::cout << "Goal bias is set to " << bias << std::endl;
	    }
	}	
	
	setupDistanceEvaluators(p);
	p.si->setup();

	model->planners["RRT"] = p;
    }
    
    /* instantiate and configure LazyRRT */
    void addLazyRRTInstance(XMLModel *model)
    {
	KinematicPlannerSetup p;
	std::cout << "Added LazyRRT instance for motion planning: " << model->groupName << std::endl;

	p.si       = new SpaceInformationXMLModel(model->kmodel, model->groupID);
	p.svc      = new StateValidityPredicate(model);
	p.si->setStateValidityChecker(p.svc);
	
	p.smoother = new ompl::PathSmootherKinematic(p.si);
	p.smoother->setMaxSteps(20);
	
	ompl::LazyRRT_t lrrt = new ompl::LazyRRT(p.si);
	p.mp                 = lrrt;	

	robot_desc::URDF::Group *group = getURDFGroup(model->groupName);
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    std::map<std::string, std::string> info = data.getMapTagValues("plan", "LazyRRT");
	     
	    if (info.find("range") != info.end())
	    {
		double range = string_utils::fromString<double>(info["range"]);
		lrrt->setRange(range);
		std::cout << "Range is set to " << range << std::endl;		
	    }
	    
	    if (info.find("goal_bias") != info.end())
	    {	
		double bias = string_utils::fromString<double>(info["goal_bias"]);
		lrrt->setGoalBias(bias);
		std::cout << "Goal bias is set to " << bias << std::endl;
	    }
	}
	
	setupDistanceEvaluators(p);
	p.si->setup();

	model->planners["LazyRRT"] = p;
    }
    
    /* for each planner definition, define the set of distance metrics it can use */
    void setupDistanceEvaluators(KinematicPlannerSetup &p)
    {
	p.sde["L2Square"] = new ompl::SpaceInformationKinematic::StateKinematicL2SquareDistanceEvaluator(p.si);
    }
    
    std::map<std::string, XMLModel*> m_models;
};

void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used for planning.\n");
}

int main(int argc, char **argv)
{ 
    if (argc == 2)
    { 
	ros::init(argc, argv);
	
	KinematicPlanning *planner = new KinematicPlanning(argv[1]);
	planner->loadRobotDescription();
	
	std::vector<std::string> mlist;    
	planner->knownModels(mlist);
	printf("Known models:\n");    
	for (unsigned int i = 0 ; i < mlist.size() ; ++i)
	    printf("  * %s\n", mlist[i].c_str());    
	if (mlist.size() > 0)
	    planner->spin();
	else
	    printf("No models defined. Kinematic planning node cannot start.\n");
	
	planner->shutdown();

	delete planner;	
    }
    else
	usage(argv[0]);
    
    profiling_utils::Profiler::Status();
    
    return 0;    
}
