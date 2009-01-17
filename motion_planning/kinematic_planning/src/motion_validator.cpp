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

@b MotionValidator is a node capable of verifying if a path is valid
or not (collides or does not collide).

<hr>

@section usage Usage
@verbatim
$ motion_validator robot_model [standard ROS args]
@endverbatim

@par Example

@verbatim
$ motion_validator robotdesc/pr2
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
- @b "validate_path"/KinematicPlanState : given a robot model, starting and goal states, this service computes a collision free path


<hr>

@section parameters ROS parameters
- None

**/

#include <robot_model/cnode.h>
#include <ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h>

#include <robot_srvs/ValidateKinematicPath.h>

#include "RKPStateValidator.h"
#include "RKPSpaceInformation.h"

#include <iostream>
#include <string>
#include <map>

class MotionValidator : public ros::Node,
			public robot_model::NodeCollisionModel
{
public:
    
    class myModel : public RKPModelBase
    {
    public:
	myModel(void) : RKPModelBase()
	{
	    si = NULL;
	    svc = NULL;
	}
	
	virtual ~myModel(void)
	{
	    if (svc)
		delete svc;
	    if (si)
		delete si;
	}
	
	ompl::SpaceInformationKinematic_t              si;
	ompl::SpaceInformation::StateValidityChecker_t svc;
    };    
    
    MotionValidator(const std::string &robot_model) : ros::Node("motion_validator"),
						      robot_model::NodeCollisionModel(dynamic_cast<ros::Node*>(this),
											     robot_model)
    {
	advertiseService("validate_path", &MotionValidator::validatePath);
    }
    
    /** Free the memory */
    virtual ~MotionValidator(void)
    {
	for (std::map<std::string, myModel*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
    }
    
    bool validatePath(robot_srvs::ValidateKinematicPath::request &req, robot_srvs::ValidateKinematicPath::response &res)
    {
	myModel *model = m_models[req.model_id];
	if (model)
	{
	    if (model->kmodel->stateDimension != req.start_state.get_vals_size())
	    {
		std::cerr << "Dimension of start state expected to be " << model->kmodel->stateDimension << " but was received as " << req.start_state.get_vals_size() << std::endl;
		return false;
	    }
	    
	    if (model->si->getStateDimension() != req.goal_state.get_vals_size())
	    {
		std::cerr << "Dimension of start goal expected to be " << model->si->getStateDimension() << " but was received as " <<  req.goal_state.get_vals_size() << std::endl;
		return false;
	    }
	    
	    std::cout << "Validating path for '" << req.model_id << "'..." << std::endl;
	    
	    const unsigned int dim = model->si->getStateDimension();
	    ompl::SpaceInformationKinematic::StateKinematic_t start = new ompl::SpaceInformationKinematic::StateKinematic(dim);

	    if (model->groupID >= 0)
	    {
		/* set the pose of the whole robot */
		model->kmodel->computeTransforms(&req.start_state.vals[0]);
		model->collisionSpace->updateRobotModel(model->collisionSpaceID);
		
		/* extract the components needed for the start state of the desired group */
		for (unsigned int i = 0 ; i < dim ; ++i)
		    start->values[i] = req.start_state.vals[model->kmodel->groupStateIndexList[model->groupID][i]];
	    }
	    else
	    {
		/* the start state is the complete state */
		for (unsigned int i = 0 ; i < dim ; ++i)
		    start->values[i] = req.start_state.vals[i];
	    }
	    
	    ompl::SpaceInformationKinematic::StateKinematic_t goal = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	    for (unsigned int i = 0 ; i < dim ; ++i)
		goal->values[i] = req.goal_state.vals[i];
	    
	    std::vector<robot_msgs::PoseConstraint> cstrs;
	    req.constraints.get_pose_vec(cstrs);
	    static_cast<StateValidityPredicate*>(model->svc)->setPoseConstraints(cstrs);

	    res.valid = model->si->checkMotionIncremental(start, goal);
	    
	    std::cout << "Result: " << (int)res.valid << std::endl;
	    
	    delete start;
	    delete goal;
	    
	    return true;	    
	}
	else
	{
	    std::cerr << "Model '" << req.model_id << "' not known" << std::endl;	    
	    return false;
	}
    }

    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	robot_model::NodeCollisionModel::setRobotDescription(file);	
	
	printf("=======================================\n");	
	m_kmodel->printModelInfo();
	printf("=======================================\n");

	/* set the data for the model */
	myModel *model = new myModel();
	model->collisionSpaceID = 0;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = m_kmodel;
	model->groupName = m_kmodel->name;
	setupModel(model);

	/* remember the model by the robot's name */
	m_models[model->groupName] = model;
	
	/* create a model for each group */
	std::vector<std::string> groups;
	m_kmodel->getGroups(groups);

	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    myModel *model = new myModel();
	    model->collisionSpaceID = 0;
	    model->collisionSpace = m_collisionSpace;
	    model->kmodel = m_kmodel;
	    model->groupID = m_kmodel->getGroupID(groups[i]);
	    model->groupName = groups[i];
	    setupModel(model);
	    m_models[model->groupName] = model;
	}
    }
    
    void knownModels(std::vector<std::string> &model_ids)
    {
	for (std::map<std::string, myModel*>::const_iterator i = m_models.begin() ; i != m_models.end() ; ++i)
	    model_ids.push_back(i->first);
    }
    
private:
    
    void setupModel(myModel *model)
    {
	model->si  = new SpaceInformationRKPModel(model);
	model->svc = new StateValidityPredicate(model);
	model->si->setStateValidityChecker(model->svc);
    }
    
    std::map<std::string, myModel*> m_models;

    
};

void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used for path validation.\n");
}

int main(int argc, char **argv)
{ 
    if (argc == 2)
    { 
	ros::init(argc, argv);
	
	MotionValidator *validator = new MotionValidator(argv[1]);
	validator->loadRobotDescription();
	
	std::vector<std::string> mlist;    
	validator->knownModels(mlist);
	printf("Known models:\n");    
	for (unsigned int i = 0 ; i < mlist.size() ; ++i)
	    printf("  * %s\n", mlist[i].c_str());    
	if (mlist.size() > 0)
	    validator->spin();
	else
	    printf("No models defined. Path validation node cannot start.\n");
	
	validator->shutdown();

	delete validator;	
    }
    else
	usage(argv[0]);
        
    return 0;    
}
