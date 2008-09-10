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

/**

@mainpage

@htmlinclude ../manifest.html

@b KinematicPlanning is a node capable of planning kinematic paths for
a set of robot models. Each robot model is a complete model specified
in URDF or consists of an URDF group.

Organization:
 - there are multiple models
 - there are multiple planners that can be used for each model
 - there are multiple types of planning requests

The code is mostly implemented in the included RKP* files (ROS
Kinematic Planning). There exists one basic class (RKPBasicRequest)
that can handle different requests. However, since the type of a
request may vary, the code for this basic request is templated. The
functions that vary with the type of request have a simple default
implementation (this implementation should not be used since it is
incomplete) and specializations for the different types of
requests. For new requests to be added, only specialization is
needed. Inheritance is also possible, on top of RKPBasicRequest, but
is not needed for the types of requets currently handled.

A model is defined for each loaded URDF model, and for each of the
URDF groups marked for planning. This model includes a kinematic
model, a collision space (shared between models) and a set of
planners. If a planner is used for different models, it is
instantiated each time. Since planners may require different
setup/configuration code, there exists a base class that defines the
functionality and an inherited class for each type of planner that can
be instantiated. The planners are associated to string names: RRT,
LazyRRT, EST, SBL. These string names can be used for the planner_id
component of the planning request.

When checking states for validity, a resolution at which paths are
check needs to be defined. To make things easier for the user, this
parameter is computed by default by the SpaceInformationRKPModel
class. The current settings work fine for the PR2, but if another
robot is to be used, different settings man need to be used.

\todo
- Find a better way to specify resolution for state validity
checking.
- Move code from header files to .cpp files (maybe define a library?)


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
	return m_requestState.execute(m_models, req, res.path, res.distance);
    }

    bool planToPosition(robot_srvs::KinematicPlanLinkPosition::request &req, robot_srvs::KinematicPlanLinkPosition::response &res)
    {	
	return m_requestLinkPosition.execute(m_models, req, res.path, res.distance);
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

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "EST");
	}
	model->addEST(options);
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
