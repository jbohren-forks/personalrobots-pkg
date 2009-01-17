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
- @b world_3d_map/PointCloud : point cloud with data describing the 3D environment

Publishes to (name/type):
- None

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):
- @b "plan_kinematic_path_state"/KinematicPlanState : given a robot model, starting and goal states, this service computes a collision free path
- @b "plan_kinematic_path_named"/NamedKinematicPlanState : given a robot model, starting and goal states, this service computes a collision free path
- @b "plan_kinematic_path_position"/KinematicPlanLinkPosition : given a robot model, starting state and goal poses of certain links, this service computes a collision free path
- @b "plan_joint_state_names/PlanNames : returns a list of all the names of joints in the system.

<hr>

@section parameters ROS parameters
- None

**/

#include "kinematic_planning/CollisionSpaceMonitor.h"
#include <std_msgs/String.h>

#include "kinematic_planning/RKPModel.h"
#include "kinematic_planning/RKPBasicRequestState.h"
#include "kinematic_planning/RKPBasicRequestLinkPosition.h"
#include <robot_srvs/PlanNames.h>
#include <robot_srvs/NamedKinematicPlanState.h>
using namespace kinematic_planning;

class KinematicPlanning : public ros::Node,
			  public CollisionSpaceMonitor
{
public:

    KinematicPlanning(const std::string &robot_model) : ros::Node("kinematic_planning"),
							CollisionSpaceMonitor(dynamic_cast<ros::Node*>(this),
									      robot_model)
    {
	advertiseService("plan_kinematic_path_state", &KinematicPlanning::planToState);
	advertiseService("plan_kinematic_path_named", &KinematicPlanning::planToStateNamed);
	advertiseService("plan_kinematic_path_position", &KinematicPlanning::planToPosition);
	advertiseService("plan_joint_state_names", &KinematicPlanning::planJointNames);
    }
    
    /** Free the memory */
    virtual ~KinematicPlanning(void)
    {
	for (std::map<std::string, RKPModel*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
    }

    bool planToStateNamed(robot_srvs::NamedKinematicPlanState::request &reqn, robot_srvs::NamedKinematicPlanState::response &resn)
    {
        robot_srvs::KinematicPlanState::request req;
        robot_srvs::KinematicPlanState::response res;




        //Use name lookups to create the state vector.
        //Find the number of parameters needed and print errors.
        unsigned int nparam = 0;
        for (unsigned int i = 0; i < reqn.start_state.get_joints_size(); i++) {
            if (!m_kmodel->getJoint(reqn.start_state.names[i])) {
                std::cerr << "Non-existant joint ignored: " << reqn.start_state.names[i] << std::endl;
                continue;
            }
            nparam += m_kmodel->getJoint(reqn.start_state.names[i])->usedParams;
            if (reqn.start_state.joints[i].get_vals_size() != m_kmodel->getJoint(reqn.start_state.names[i])->usedParams) {
                std::cerr << "You do not have the right number of axes for a the joint: "
                          << reqn.start_state.names[i] << " (" << reqn.start_state.joints[i].get_vals_size()
                          << " != " << m_kmodel->getJoint(reqn.start_state.names[i])->usedParams << "). "
                          << "Extra axes will be ignored, non-specified ones will be set to zero.\n";
                  
            }
        }
        //std::cout << "Name set up\nNeed " << nparam << " parameters.\n";
        //Create the state vector and set it to zero.
        req.start_state.set_vals_size(nparam);
        for (unsigned int i = 0; i < req.start_state.get_vals_size(); i++) {
            req.start_state.vals[i] = 0;
        }
        //Iterate over all the joints and populate the state vector.
        for (unsigned int i = 0; i < reqn.start_state.get_joints_size(); i++) {
            if (m_kmodel->parameterNames.find(reqn.start_state.names[i]) == m_kmodel->parameterNames.end()) {
                continue;
            }
            unsigned int p = m_kmodel->parameterNames.find(reqn.start_state.names[i])->second;
            
            for (unsigned int k = 0; k < m_kmodel->getJoint(reqn.start_state.names[i])->usedParams; k++) {
                req.start_state.vals[p + k] = reqn.start_state.joints[i].vals[k];
            }

            //std::cout << reqn.start_state.names[i] << " [" << p << "-" 
            //          << p + m_kmodel->getJoint(reqn.start_state.names[i])->usedParams - 1 << "]\n";
        }
        

        
        //Use name lookups to populate the joint group.

        unsigned int group = m_kmodel->getGroupID(reqn.params.model_id);

        
        //std::cout << "Joints:\n";
        
        //Create a vector with the joint map for the planning subspace.
        std::vector<planning_models::KinematicModel::Joint*> subspaceJointMapSparse;
        std::vector<planning_models::KinematicModel::Joint*> subspaceJointMap;

        //Fill it with nulls.
        for (unsigned int i = 0; i < req.start_state.get_vals_size(); i++) {
            subspaceJointMapSparse.push_back(NULL);
        }

        //Iterate over the joint parameter names table, and fill up the subspace joint
        //map with the joints in the target group.
        for (std::map<std::string, unsigned int>::iterator it = m_kmodel->parameterNames.begin();
               it != m_kmodel->parameterNames.end(); it++) {
          if (m_kmodel->getJoint(it->first)->inGroup[group]) {
            subspaceJointMapSparse[it->second] = m_kmodel->getJoint(it->first);
          }
        }

        //Create a non-sparse subspace joint map, and get nparam.
        nparam = 0;
        for (unsigned int i = 0; i < subspaceJointMapSparse.size(); i++) {
          if (subspaceJointMapSparse[i]) {
            subspaceJointMap.push_back(subspaceJointMapSparse[i]);
            nparam += subspaceJointMapSparse[i]->usedParams;
            std::cout << subspaceJointMapSparse[i]->name << std::endl;
          }
        }



        //Copy the goal state.

        //Zero the output messages.
        req.goal_state.set_vals_size(nparam);
        for (unsigned int i = 0; i < nparam; i++) {
            req.goal_state.vals[i] = 0.0;
        }

        //Copy the input into the message using the jointmap.
        nparam = 0;
        for (unsigned int i = 0; i < subspaceJointMap.size(); i++) {
            bool found = false;
            for (unsigned int j = 0; j < reqn.goal_state.get_names_size() && j < reqn.goal_state.get_joints_size(); j++) {
                if (reqn.goal_state.names[j] == subspaceJointMap[i]->name) {
                    if (subspaceJointMap[i]->usedParams != reqn.goal_state.joints[j].get_vals_size()) {
                        std::cerr << "You do not have the right number of axes for a the joint: "
                                  << subspaceJointMap[i]->name << " (" << subspaceJointMap[i]->usedParams
                                  << " != " << reqn.goal_state.joints[j].get_vals_size() << "). "
                                  << "Extra axes will be ignored, non-specified ones will be set to zero.\n";
                    }
                    for (unsigned int k = 0; k < subspaceJointMap[i]->usedParams && k < reqn.goal_state.joints[j].get_vals_size(); k++) {
                        std::cout << "Writing " << nparam << " for " << subspaceJointMap[i]->name << std::endl;
                        req.goal_state.vals[nparam] = reqn.goal_state.joints[j].vals[k];
                        nparam++;
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::cerr << "You did not specify a goal value for the joint:" << subspaceJointMap[i]->name << std::endl;
            }
        }
        

        //Blind copy for the rest.
        req.params = reqn.params;
        req.constraints = reqn.constraints;
        req.times = reqn.times;
        req.interpolate = reqn.interpolate;
        req.allowed_time = reqn.allowed_time;
        req.threshold = reqn.threshold;



        //Acutally run the service.
	bool r = m_requestState.execute(m_models, req, res.path, res.distance);


        //Copy the path.
        resn.path.set_states_size(res.path.get_states_size());
        
        for (unsigned int j = 0; j < resn.path.get_states_size(); j++) {
            nparam = 0;
            resn.path.states[j].set_names_size(subspaceJointMap.size());
            resn.path.states[j].set_joints_size(subspaceJointMap.size());
            for (unsigned int i = 0; i < subspaceJointMap.size(); i++) {
                resn.path.states[j].names[i] = subspaceJointMap[i]->name;
                resn.path.states[j].joints[i].set_vals_size(subspaceJointMap[i]->usedParams);
                for (unsigned int k = 0; k < subspaceJointMap[i]->usedParams; k++) {
                    resn.path.states[j].joints[i].vals[k] = res.path.states[j].vals[nparam];
                    nparam++;
                }
            }
        }

        

        //Simply copy the other results.
        resn.distance = res.distance;
        return r;
    }

    bool planJointNames(robot_srvs::PlanNames::request &req, robot_srvs::PlanNames::response &res) {
        std::vector<planning_models::KinematicModel::Joint*> joints;
        m_kmodel->getJoints(joints);
        res.set_names_size(joints.size());
        res.set_num_values_size(joints.size());
        for (unsigned int i = 0; i < joints.size(); i++) {
            res.names[i] = joints[i]->name;
            res.num_values[i] = joints[i]->usedParams;
        }
        return true;
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
	CollisionSpaceMonitor::setRobotDescription(file);	
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
    
    return 0;    
}
