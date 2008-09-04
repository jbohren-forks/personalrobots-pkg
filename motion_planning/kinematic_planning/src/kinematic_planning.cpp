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
- @b planning_statistics/String : a messsage with statistics about computed motion plans

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):
- @b "plan_kinematic_path_state"/KinematicPlanState : given a robot model, starting and goal states, this service computes a collision free path
- @b "plan_kinematic_path_position"/KinematicPlanLinkPosition : given a robot model, starting state and goal poses of certain links, this service computes a collision free path

<hr>

@section parameters ROS parameters
- None

**/

#include <planning_node_util/cnode.h>
#include <std_msgs/String.h>

#include "RKPModel.h"
#include "RKPBasicRequestState.h"
#include "RKPBasicRequestLinkPosition.h"

#include <profiling_utils/profiler.h>

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
	advertise<std_msgs::String>("planning_statistics", 10);
    }
    
    /** Free the memory */
    virtual ~KinematicPlanning(void)
    {
	for (std::map<std::string, RKPModel*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
    }
    
    bool planToState(robot_srvs::KinematicPlanState::request &req, robot_srvs::KinematicPlanState::response &res)
    {
	if (!m_requestState.isRequestValid(m_models, req))
	    return false;
	
	/* find the data we need */
	RKPModel             *m = m_models[req.params.model_id];
	RKPPlannerSetup *psetup = m->planners[req.params.planner_id];
	
	/* configure state space and starting state */
	m_requestState.setupStateSpaceAndStartState(m, psetup, req.params, req.start_state);
	
	std::vector<robot_msgs::PoseConstraint> cstrs;
	req.constraints.get_pose_vec(cstrs);
	m_requestState.setupPoseConstraints(psetup, cstrs);
	
	/* add goal state */
	m_requestState.setupGoalState(m, psetup, req);
	
	/* print some information */
	printf("=======================================\n");
	psetup->si->printSettings();
	printf("=======================================\n");	
	
	/* compute actual motion plan */
	ompl::SpaceInformationKinematic::PathKinematic_t bestPath       = NULL;
	double                                           bestDifference = 0.0;	

	m_collisionSpace->lock();
	profiling_utils::Profiler::Start();
	m_requestState.computePlan(psetup, req.times, req.allowed_time, req.interpolate, bestPath, bestDifference);
	profiling_utils::Profiler::Stop();
	m_collisionSpace->unlock();

	/* fill in the results */
	m_requestState.fillSolution(psetup, bestPath, bestDifference, res.path, res.distance);
	
	/* clear memory */
	m_requestState.cleanupPlanningData(psetup);
	
	return true;
    }

    bool planToPosition(robot_srvs::KinematicPlanLinkPosition::request &req, robot_srvs::KinematicPlanLinkPosition::response &res)
    {
	if (!m_requestLinkPosition.isRequestValid(m_models, req))
	    return false;
	
	/* find the data we need */
	RKPModel             *m = m_models[req.params.model_id];
	RKPPlannerSetup *psetup = m->planners[req.params.planner_id];
	
	/* configure state space and starting state */
	m_requestLinkPosition.setupStateSpaceAndStartState(m, psetup, req.params, req.start_state);
	
	std::vector<robot_msgs::PoseConstraint> cstrs;
	req.constraints.get_pose_vec(cstrs);
	m_requestLinkPosition.setupPoseConstraints(psetup, cstrs);

	/* add goal state */
	m_requestLinkPosition.setupGoalState(m, psetup, req);
	
	/* print some information */
	printf("=======================================\n");
	psetup->si->printSettings();
	printf("=======================================\n");	
	
	/* compute actual motion plan */
	ompl::SpaceInformationKinematic::PathKinematic_t bestPath       = NULL;
	double                                           bestDifference = 0.0;	

	m_collisionSpace->lock();
	profiling_utils::Profiler::Start();

	m_requestLinkPosition.computePlan(psetup, req.times, req.allowed_time, req.interpolate, bestPath, bestDifference);

	profiling_utils::Profiler::Stop();
	m_collisionSpace->unlock();

	/* fill in the results */
	m_requestLinkPosition.fillSolution(psetup, bestPath, bestDifference, res.path, res.distance);
	
	/* clear memory */
	m_requestLinkPosition.cleanupPlanningData(psetup);
	
	return true;
    }

    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	planning_node_util::NodeCollisionModel::setRobotDescription(file);	
	defaultPosition();
	
	printf("=======================================\n");	
	m_kmodel->printModelInfo();
	printf("=======================================\n");

	/* set the data for the model */
	RKPModel *model = new RKPModel();
	model->collisionSpaceID = 0;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = m_kmodel;
	model->groupName = m_kmodel->name;
	createMotionPlanningInstances(model);
	
	/* remember the model by the robot's name */
	m_models[model->groupName] = model;
	
	/* create a model for each group */
	std::vector<std::string> groups;
	m_kmodel->getGroups(groups);

	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    RKPModel *model = new RKPModel();
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
	for (std::map<std::string, RKPModel*>::const_iterator i = m_models.begin() ; i != m_models.end() ; ++i)
	    model_ids.push_back(i->first);
    }
    
private:
    
    /* instantiate the planners that can be used  */
    void createMotionPlanningInstances(RKPModel* model)
    {	
	std::map<std::string, std::string> options;
	robot_desc::URDF::Group *group = m_urdf->getGroup(model->kmodel->getURDFGroup(model->groupName));
	
	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "RRT");
	}
	model->addRRT(options);
	

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "LazyRRT");
	}
	model->addLazyRRT(options);
	

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "SBL");
	}
	model->addSBL(options);
    }
    
    ModelMap                                                        m_models;
    RKPBasicRequest<robot_srvs::KinematicPlanState::request>        m_requestState;
    RKPBasicRequest<robot_srvs::KinematicPlanLinkPosition::request> m_requestLinkPosition;
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
