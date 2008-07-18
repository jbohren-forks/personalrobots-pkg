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
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/KinematicPath.h>
#include <std_srvs/KinematicMotionPlan.h>

#include <urdf/URDF.h>
#include <kinematic_planning/definitions.h>
#include <collision_space/environmentODE.h>
#include <ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h>

#include <vector>
#include <string>
#include <map>


class KinematicPlanning : public ros::node
{
public:

    KinematicPlanning(void) : ros::node("kinematic_planning")
    {
	advertise_service("plan_kinematic_path", &KinematicPlanning::plan);
	subscribe("world_3d_map", m_cloud, &KinematicPlanning::pointCloudCallback);

	m_collisionSpace = new collision_space::EnvironmentModelODE();
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
    
    void pointCloudCallback(void)
    {
	unsigned int n = m_cloud.get_pts_size();
	printf("received %u points\n", n);

	double *data = new double[3 * n];	
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    unsigned int i3 = i * 3;	    
	    data[i3    ] = m_cloud.pts[i].x;
	    data[i3 + 1] = m_cloud.pts[i].y;
	    data[i3 + 2] = m_cloud.pts[i].z;
	}
	delete[] data;
	
    }
    
    bool plan(std_srvs::KinematicMotionPlan::request &req, std_srvs::KinematicMotionPlan::response &res)
    {
	//	const int dim = req.start_state.vals_size;
	//	ompl::SpaceInformationKinematic::GoalStateKinematic_t goal = new ompl::SpaceInformationKinematic::GoalStateKinematic(m_si);
	//m_si->setGoal(goal);
	
	/*
	std::vector<double*> path;
	double start[dim];
	double goal[dim];
	
	for (int i = 0 ; i < dim ; ++i)
	    start[i] = req.start_state.vals[i];
	for (int i = 0 ; i < dim ; ++i)
	    goal[i] = req.goal_state.vals[i];
	
	/////////////////
	
	res.path.set_states_size(path.size());
	for (unsigned int i = 0 ; i < path.size() ; ++i)
	{
	    res.path.states[i].set_vals_size(dim);
	    for (int j = 0 ; j < dim ; ++j)
		res.path.states[i].vals[j] = path[i][j];
	    delete[] path[i];
	}
	*/
	//	m_si->clearGoal();	
	
	return true;	
    }

    void addRobotDescriptionFromFile(const char *filename)
    {
	robot_desc::URDF *file = new robot_desc::URDF(filename);
	m_robotDescriptions.push_back(file);

	std::vector<std::string> groups;
	file->getGroupNames(groups);
	std::string name = file->getRobotName();
	
	/* create a model for the whole robot (with the name given in the file) */
	Model *model = new Model();
	model->collisionSpaceID = m_collisionSpace->addRobotModel(*file);
	m_models[name] = model;
	createMotionPlanningInstances(model);
	
	/* create a model for each group */
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    Model *model = new Model();
	    model->collisionSpaceID = m_collisionSpace->addRobotModel(*file, groups[i].c_str());
	    m_models[name + "::" + groups[i]] = model;
	    createMotionPlanningInstances(model);
	}	
    }
        
private:
      
    struct Planner
    {
	ompl::MotionPlanner_t             mp;
	ompl::SpaceInformationKinematic_t si;
	int                               type;
    };
    	
    struct Model
    {
	Model(void)
	{
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
    };
    
    class SpaceInformationNode : public ompl::SpaceInformationKinematic
    {
    public:
	SpaceInformationNode(planning_models::KinematicModel *km) : SpaceInformationKinematic()
	{
	    m_stateDimension = km->stateDimension;
	    m_stateComponent.resize(m_stateDimension);
	    
	    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    {	
		m_stateComponent[i].type       = StateComponent::NORMAL;
		m_stateComponent[i].minValue   = km->stateBounds[i*2    ];
		m_stateComponent[i].maxValue   = km->stateBounds[i*2 + 1];
		m_stateComponent[i].resolution = (m_stateComponent[i].maxValue - m_stateComponent[i].minValue) / 20.0;
		if (m_stateComponent[i].resolution == 0.0)
		    m_stateComponent[i].resolution = 0.1; // this happens for floating joints
	    }
	}

	virtual ~SpaceInformationNode(void)
	{
	}
    };    
	
    std_msgs::PointCloudFloat32        m_cloud;
    collision_space::EnvironmentModel *m_collisionSpace;    
    std::map<std::string, Model*>      m_models;
    std::vector<robot_desc::URDF*>     m_robotDescriptions;    

    static bool isStateValid(const ompl::SpaceInformationKinematic::StateKinematic_t state, void *data)
    {
	Model *model = reinterpret_cast<Model*>(data);
	return model->collisionSpace->isCollision(model->collisionSpaceID);
    }
    
    void createMotionPlanningInstances(Model* model)
    {
	model->collisionSpace = m_collisionSpace;

	Planner p;
	p.si   = new SpaceInformationNode(m_collisionSpace->models[model->collisionSpaceID]);
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
    planner.spin();
    planner.shutdown();
    
    return 0;    
}
