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

/**

@mainpage

@htmlinclude manifest.html

@b KinematicPlanning is a node capable of planning kinematic paths for
a set of robot models.

<hr>

@section usage Usage
@verbatim
$ kinematic_planning [standard ROS args]
@endverbatim

@par Example

@verbatim
$ kinematic_planning
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
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

#include <ros/node.h>
#include <ros/time.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/RobotBase2DOdom.h>
#include <robot_srvs/KinematicMotionPlan.h>

#include <urdf/URDF.h>
#include <collision_space/environmentODE.h>
#include <ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h>

#include "SpaceInformationXMLModel.h"

#include <vector>
#include <string>
#include <sstream>
#include <map>

class KinematicPlanning : public ros::node
{
public:

    KinematicPlanning(void) : ros::node("kinematic_planning")
    {
	advertise_service("plan_kinematic_path", &KinematicPlanning::plan);
	subscribe("world_3d_map", m_cloud, &KinematicPlanning::pointCloudCallback);
	subscribe("localizedpose", m_localizedPose, &KinematicPlanning::localizedPoseCallback);

	m_collisionSpace = new collision_space::EnvironmentModelODE();
	m_collisionSpace->setSelfCollision(false); // for now, disable self collision (incorrect model)
	
	m_collisionSpace->lock();
	loadRobotDescriptions();
	m_collisionSpace->unlock();
    }
    
    ~KinematicPlanning(void)
    {
	for (unsigned int i = 0 ; i < m_robotDescriptions.size() ; ++i)
	    delete m_robotDescriptions[i];
	for (std::map<std::string, Model*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
	if (m_collisionSpace)
	    delete m_collisionSpace;
    }
    

    void loadRobotDescriptions(void)
    {
	printf("Loading robot descriptions...\n\n");
	
	std::string description_files;
	if (get_param("robotdesc_list", description_files))
	{
	    std::stringstream sdf(description_files);
	    while (sdf.good())
	    {
		std::string file;
		std::string content;
		sdf >> file;
		if (get_param(file, content))
		    addRobotDescriptionFromData(content.c_str());
	    }
	}
	printf("\n\n");	
    }
    
    void localizedPoseCallback(void)
    {
	m_basePos[0] = m_localizedPose.pos.x;
	m_basePos[1] = m_localizedPose.pos.y;
	m_basePos[2] = m_localizedPose.pos.th;
    }
    
    void pointCloudCallback(void)
    {
	unsigned int n = m_cloud.get_pts_size();
	printf("Received %u points\n", n);

       	ros::Time startTime = ros::Time::now();
	double *data = new double[3 * n];	
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    unsigned int i3 = i * 3;	    
	    data[i3    ] = m_cloud.pts[i].x;
	    data[i3 + 1] = m_cloud.pts[i].y;
	    data[i3 + 2] = m_cloud.pts[i].z;
	}
	
	m_collisionSpace->lock();
	m_collisionSpace->clearObstacles();
	m_collisionSpace->addPointCloud(n, data, 0.03);
	m_collisionSpace->unlock();
	
	delete[] data;
	
	double tupd = (ros::Time::now() - startTime).to_double();	
	printf("Updated world model in %f seconds\n", tupd);
    }
    
    bool plan(robot_srvs::KinematicMotionPlan::request &req, robot_srvs::KinematicMotionPlan::response &res)
    {
	if (m_models.find(req.model_id) == m_models.end())
	{
	    fprintf(stderr, "Cannot plan for '%s'. Model does not exist\n", req.model_id.c_str());
	    return false;	    
	}
	
	Model   *m = m_models[req.model_id];
	Planner &p = m->planners[0];
	
	if (m->kmodel->stateDimension != req.start_state.get_vals_size())
	{
	    fprintf(stderr, "Dimension of start state expected to be %u but was received as %u\n", m->kmodel->stateDimension, req.start_state.get_vals_size());
	    return false;	    
	}

	const int dim = req.goal_state.get_vals_size();
	if ((int)p.si->getStateDimension() != dim)
	{
	    fprintf(stderr, "Dimension of goal state expected to be %u but was received as %d\n", p.si->getStateDimension(), dim);
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
	m->kmodel->printModelInfo();
	p.si->printSettings();
	printf("=======================================\n");	
	
	/* do the planning */
	m_collisionSpace->lock();
	
	ros::Time startTime = ros::Time::now();
	bool ok = p.mp->solve(req.allowed_time); 
	double tsolve = (ros::Time::now() - startTime).to_double();	
	printf("Motion planner spent %f seconds\n", tsolve);
	
	/* do path smoothing */
	if (ok)
	{
	    ros::Time startTime = ros::Time::now();
	    ompl::SpaceInformationKinematic::PathKinematic_t path = static_cast<ompl::SpaceInformationKinematic::PathKinematic_t>(goal->getSolutionPath());
	    p.si->smoothVertices(path);
	    double tsmooth = (ros::Time::now() - startTime).to_double();	
	    printf("Smoother spent %f seconds (%f seconds in total)\n", tsmooth, tsmooth + tsolve);
	}	
	
	m_collisionSpace->unlock();

	
	/* copy the solution to the result */
	if (ok)
	{
	    ompl::SpaceInformationKinematic::PathKinematic_t path = static_cast<ompl::SpaceInformationKinematic::PathKinematic_t>(goal->getSolutionPath());
	    res.path.set_states_size(path->states.size());
	    for (unsigned int i = 0 ; i < path->states.size() ; ++i)
	    {
		res.path.states[i].set_vals_size(dim);
		for (int j = 0 ; j < dim ; ++j)
		    res.path.states[i].vals[j] = path->states[i]->values[j];
	    }
	}
	else
	    res.path.set_states_size(0);
	
	/* cleanup */
	p.si->clearGoal();
	p.si->clearStartStates(true);
	p.mp->clear();
	
	return true;
    }

    void addRobotDescriptionFromFile(const char *filename)
    {
	robot_desc::URDF *file = new robot_desc::URDF();
	if (file->loadFile(filename))
	    addRobotDescription(file);   
	else
	    delete file;
    }

    void addRobotDescriptionFromData(const char *data)
    {
	robot_desc::URDF *file = new robot_desc::URDF();
	if (file->loadString(data))
	    addRobotDescription(file);
	else
	    delete file;
    }
    
    void addRobotDescription(robot_desc::URDF *file)
    {
	m_robotDescriptions.push_back(file);
	
	printf("\n\nCreating new kinematic model:\n");
	
	/* create a model for the whole robot (with the name given in the file) */
	planning_models::KinematicModel *kmodel = new planning_models::KinematicModel();
	kmodel->setVerbose(true);
	kmodel->build(*file);
	
	/* add the model to the collision space */
	unsigned int cid = m_collisionSpace->addRobotModel(kmodel);

	/* set the data for the model */
	Model *model = new Model();
	model->collisionSpaceID = cid;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = kmodel;
	createMotionPlanningInstances(model);
	
	/* remember the model by the robot's name */
	m_models[file->getRobotName()] = model;

	
	/* set all parameters to 0 */
	double defaultPose[kmodel->stateDimension];
	for (unsigned int i = 0 ; i < kmodel->stateDimension ; ++i)
	    defaultPose[i] = 0.0;

	kmodel->computeTransforms(defaultPose);
	m_collisionSpace->updateRobotModel(cid);

	/* create a model for each group */
	std::vector<std::string> groups;
	kmodel->getGroups(groups);

	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    Model *model = new Model();
	    model->collisionSpaceID = cid;
	    model->collisionSpace = m_collisionSpace;
	    model->kmodel = kmodel;
	    model->groupID = kmodel->getGroupID(groups[i]);
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
      
    struct Planner
    {
	ompl::Planner_t                   mp;
	ompl::SpaceInformationKinematic_t si;
	int                               type;
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
		delete planners[i].si;
	    }
	}
	
	std::vector<Planner>               planners;
	collision_space::EnvironmentModel *collisionSpace;    	
	unsigned int                       collisionSpaceID;
	planning_models::KinematicModel   *kmodel;
	int                                groupID;	
    };
    	
    std_msgs::PointCloudFloat32        m_cloud;
    std_msgs::RobotBase2DOdom          m_localizedPose;
    double                             m_basePos[3];    
    collision_space::EnvironmentModel *m_collisionSpace;    
    std::map<std::string, Model*>      m_models;
    std::vector<robot_desc::URDF*>     m_robotDescriptions;
    

    static bool isStateValid(const ompl::SpaceInformationKinematic::StateKinematic_t state, void *data)
    {
	Model *model = reinterpret_cast<Model*>(data);
	model->kmodel->computeTransforms(state->values, model->groupID);
	model->collisionSpace->updateRobotModel(model->collisionSpaceID);
	bool collision = model->collisionSpace->isCollision(model->collisionSpaceID);
	return !collision;
    }
    
    void createMotionPlanningInstances(Model* model)
    {
	Planner p;
	p.si   = new SpaceInformationXMLModel(model->kmodel, model->groupID);
	p.si->setStateValidFn(isStateValid, reinterpret_cast<void*>(model));
	p.mp   = new ompl::RRT(p.si);
	p.type = 0;	
	model->planners.push_back(p);
    }

};

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    KinematicPlanning planner;
    
    std::vector<std::string> mlist;    
    planner.knownModels(mlist);
    printf("Known models:\n");    
    for (unsigned int i = 0 ; i < mlist.size() ; ++i)
	printf("  * %s\n", mlist[i].c_str());    
    if (mlist.size() > 0)
	planner.spin();
    else
	printf("No models defined. Kinematic planning node cannot start.\n");
    
    planner.shutdown();
    
    return 0;    
}
