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

#include "SpaceInformationXMLModel.h"

#include <profiling_utils/profiler.h>

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
	advertise_service("plan_kinematic_path_state", &KinematicPlanning::plan);


	double sphere[3] = {0.7, 0.5, 0.7 };
	m_collisionSpace->addPointCloud(1, sphere, 0.2);

    }
    
    ~KinematicPlanning(void)
    {
	for (std::map<std::string, Model*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
    }    
    
    bool plan(robot_srvs::KinematicPlanState::request &req, robot_srvs::KinematicPlanState::response &res)
    {
	if (m_models.find(req.model_id) == m_models.end())
	{
	    std::cerr << "Cannot plan for '" << req.model_id << "'. Model does not exist" << std::endl;
	    return false;	    
	}
	
	Model   *m = m_models[req.model_id];
	Planner &p = m->planners[0];
	
	if (m->kmodel->stateDimension != req.start_state.get_vals_size())
	{
	    std::cerr << "Dimension of start state expected to be " << m->kmodel->stateDimension << " but was received as " << req.start_state.get_vals_size() << std::endl;
	    return false;	    
	}
	
	const int dim = req.goal_state.get_vals_size();
	if ((int)p.si->getStateDimension() != dim)
	{
	    std::cerr << "Dimension of start goal expected to be " << p.si->getStateDimension() << " but was received as " << dim << std::endl;
	    return false;
	}
	
	if (req.times <= 0)
	{
	    std::cerr << "Request specifies motion plan should be computed " << req.times << " times" << std::endl;
	    return false;
	}
	
	
	/* set the workspace volume for planning */
	// only area or volume should go through... not sure what happens on really complicated models where 
	// we have both multiple planar and multiple floating joints...
	static_cast<SpaceInformationXMLModel*>(p.si)->setPlanningArea(req.volumeMin.x, req.volumeMin.y,
								      req.volumeMax.x, req.volumeMax.y);
        static_cast<SpaceInformationXMLModel*>(p.si)->setPlanningVolume(req.volumeMin.x, req.volumeMin.y, req.volumeMin.z,
									req.volumeMax.x, req.volumeMax.y, req.volumeMax.z);
    
	/* set the starting state */
        ompl::SpaceInformationKinematic::StateKinematic_t start = new ompl::SpaceInformationKinematic::StateKinematic(dim);
    
	if (m->groupID >= 0)
	{
	    /* set the pose of the whole robot */
	    m->kmodel->computeTransforms(req.start_state.vals);
	    m->collisionSpace->updateRobotModel(m->collisionSpaceID);
	    
	    /* extract the components needed for the start state of the desired group */
	    for (int i = 0 ; i < dim ; ++i)
		start->values[i] = req.start_state.vals[m->kmodel->groupStateIndexList[m->groupID][i]];
	}
	else
	{
	    /* the start state is the complete state */
	    for (int i = 0 ; i < dim ; ++i)
		start->values[i] = req.start_state.vals[i];
	}
	
	p.si->addStartState(start);
	
	/* set the goal */
	ompl::SpaceInformationKinematic::GoalStateKinematic_t goal = new ompl::SpaceInformationKinematic::GoalStateKinematic(p.si);
	goal->state = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	for (int i = 0 ; i < dim ; ++i)
	    goal->state->values[i] = req.goal_state.vals[i];
	goal->threshold = req.threshold;
	p.si->setGoal(goal);

	printf("=======================================\n");
	p.si->printSettings();
	printf("=======================================\n");	
	
	
	/* do the planning */
        ompl::SpaceInformationKinematic::PathKinematic_t bestPath = NULL;
        double                                           bestDifference = 0.0;
        double                                           totalTime = 0.0;
    
	m_collisionSpace->lock();
	
	profiling_utils::Profiler::Start();

	for (int i = 0 ; i < req.times ; ++i)
	{
	    ros::Time startTime = ros::Time::now();
	    bool ok = p.mp->solve(req.allowed_time); 
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
		if (req.interpolate)
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

        std::cout << std::endl << "Total planning time: " << totalTime << "; Average planning time: " << (totalTime / (double)req.times) << " (seconds)" << std::endl;

	/* copy the solution to the result */
	if (bestPath)
	{
	    res.path.set_states_size(bestPath->states.size());
	    for (unsigned int i = 0 ; i < bestPath->states.size() ; ++i)
	    {
		res.path.states[i].set_vals_size(dim);
		for (int j = 0 ; j < dim ; ++j)
		    res.path.states[i].vals[j] = bestPath->states[i]->values[j];
	    }
	    res.distance = bestDifference;
	}
	else
	{
	    res.path.set_states_size(0);
	    res.distance = -1.0;
	}
	
	/* cleanup */
	p.si->clearGoal();
	p.si->clearStartStates();
	
	return true;
    }

    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	planning_node_util::NodeCollisionModel::setRobotDescription(file);	
	defaultPosition();
	
	/* set the data for the model */
	Model *model = new Model();
	model->collisionSpaceID = 0;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = m_kmodel;
	createMotionPlanningInstances(model);
	
	/* remember the model by the robot's name */
	m_models[m_urdf->getRobotName()] = model;
	
	printf("=======================================\n");	
	m_kmodel->printModelInfo();
	printf("=======================================\n");

	/* create a model for each group */
	std::vector<std::string> groups;
	m_kmodel->getGroups(groups);

	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    Model *model = new Model();
	    model->collisionSpaceID = 0;
	    model->collisionSpace = m_collisionSpace;
	    model->kmodel = m_kmodel;
	    model->groupID = m_kmodel->getGroupID(groups[i]);
	    createMotionPlanningInstances(model);
	    m_models[groups[i]] = model;
	}
    }
    
    void knownModels(std::vector<std::string> &model_ids)
    {
	for (std::map<std::string, Model*>::const_iterator i = m_models.begin() ; i != m_models.end() ; ++i)
	    model_ids.push_back(i->first);
    }
    
private:
    
    class StateValidityPredicate;
    
    struct Planner
    {
	ompl::Planner_t                   mp;
	ompl::SpaceInformationKinematic_t si;
	StateValidityPredicate*           svc;
	ompl::PathSmootherKinematic_t     smoother;
    };
    	
    struct Model
    {
	Model(void)
	{
	    groupID          = -1;
	    collisionSpaceID = 0;
	    collisionSpace   = NULL;	    
	}
	
	~Model(void)
	{
	    for (unsigned int i = 0 ; i < planners.size() ; ++i)
	    {
		delete planners[i].mp;
		delete planners[i].svc;
		delete planners[i].smoother;
		delete planners[i].si;
	    }
	}
	
	std::vector<Planner>               planners;
	collision_space::EnvironmentModel *collisionSpace;    	
	unsigned int                       collisionSpaceID;
	planning_models::KinematicModel   *kmodel;
	int                                groupID;	
    };
    
    class StateValidityPredicate : public ompl::SpaceInformation::StateValidityChecker
    {
    public:
	StateValidityPredicate(Model *model) : ompl::SpaceInformation::StateValidityChecker()
	{
	    m_model = model;
	}
	
	virtual bool operator()(const ompl::SpaceInformation::State_t state)
	{
	    m_model->kmodel->computeTransforms(static_cast<const ompl::SpaceInformationKinematic::StateKinematic_t>(state)->values, m_model->groupID);
	    m_model->collisionSpace->updateRobotModel(m_model->collisionSpaceID);

	    bool collision = m_model->collisionSpace->isCollision(m_model->collisionSpaceID);
	    return !collision;
	}
	
    protected:
	
	Model *m_model;	
    };
    
	
    void createMotionPlanningInstances(Model* model)
    {
	addRRTInstance(model);
    }
    
    void addRRTInstance(Model *model)
    {
	Planner p;
	p.si       = new SpaceInformationXMLModel(model->kmodel, model->groupID);
	p.svc      = new StateValidityPredicate(model);
	p.si->setStateValidityChecker(p.svc);
	p.smoother = new ompl::PathSmootherKinematic(p.si);
	p.smoother->setMaxSteps(20);	
	p.mp        = new ompl::RRT(p.si);
	p.si->setup();
	model->planners.push_back(p);
	std::cout << "Added RRT instance for motion planning" << std::endl;
    }
    
    void addLazyRRTInstance(Model *model)
    {
	Planner p;
	p.si       = new SpaceInformationXMLModel(model->kmodel, model->groupID);
	p.svc      = new StateValidityPredicate(model);
	p.si->setStateValidityChecker(p.svc);
	p.smoother = new ompl::PathSmootherKinematic(p.si);
	p.smoother->setMaxSteps(20);	
	p.mp        = new ompl::LazyRRT(p.si);
	p.si->setup();
	model->planners.push_back(p);
	std::cout << "Added LazyRRT instance for motion planning" << std::endl;
    }

    std::map<std::string, Model*> m_models;
    
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
