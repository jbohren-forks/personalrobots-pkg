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
#include <robot_msgs/KinematicPath.h>
#include <robot_srvs/KinematicMotionPlan.h>

#include <urdf/URDF.h>
#include <collision_space/environmentODE.h>
#include <ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h>

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
	
	m_collisionSpace = new collision_space::EnvironmentModelODE();

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
	std::string description_files;
	get_param("robotdesc_list", description_files);
	std::stringstream sdf(description_files);
	while (sdf.good())
	{
	    std::string file;
	    std::string content;
	    sdf >> file;
	    get_param(file, content);
	    addRobotDescriptionFromData(content.c_str());
	}
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
	
	m_collisionSpace->lock();
	m_collisionSpace->clearObstacles();
	m_collisionSpace->addPointCloud(n, data, 0.01);
	m_collisionSpace->unlock();
	
	delete[] data;
	
    }
    
    bool plan(robot_srvs::KinematicMotionPlan::request &req, robot_srvs::KinematicMotionPlan::response &res)
    {
	Model   *m = m_models[req.model_id];
	Planner &p = m->planners[0];
	
	const int dim = req.start_state.vals_size;
	if ((int)p.si->getStateDimension() != dim)
	    return false;
	
	libTF::Pose3D &tf = m->collisionSpace->models[m->collisionSpaceID]->rootTransform;  
	tf.setPosition(req.transform.xt, req.transform.yt, req.transform.zt);
	tf.setQuaternion(req.transform.xr, req.transform.yr, req.transform.zr, req.transform.w);
	
	static_cast<SpaceInformationNode*>(p.si)->setPlanningVolume(req.volumeMin.x, req.volumeMin.y, req.volumeMin.z,
								    req.volumeMax.x, req.volumeMax.y, req.volumeMax.z);
	
	/* set the starting state */
	ompl::SpaceInformationKinematic::StateKinematic_t start = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	for (int i = 0 ; i < dim ; ++i)
	    start->values[i] = req.start_state.vals[i];
	p.si->addStartState(start);
	
	/* set the goal */
	ompl::SpaceInformationKinematic::GoalStateKinematic_t goal = new ompl::SpaceInformationKinematic::GoalStateKinematic(p.si);
	goal->state = new ompl::SpaceInformationKinematic::StateKinematic(dim);
	for (int i = 0 ; i < dim ; ++i)
	    goal->state->values[i] = req.goal_state.vals[i];
	p.si->setGoal(goal);

	m_collisionSpace->lock();
	bool ok = p.mp->solve(req.allowed_time); 
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

	/* cleanup */
	p.si->clearGoal();
	p.si->clearStartStates(true);
	p.mp->clear();
	
	return true;
    }

    void addRobotDescriptionFromFile(const char *filename)
    {
	robot_desc::URDF *file = new robot_desc::URDF(filename);
	addRobotDescription(file);   
    }

    void addRobotDescriptionFromData(const char *data)
    {
	robot_desc::URDF *file = new robot_desc::URDF();
	file->loadString(data);
	addRobotDescription(file);
    }
    
    void addRobotDescription(robot_desc::URDF *file)
    {
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
	    m_km = km;
	    m_divisions = 20.0;
	    
	    m_stateDimension = km->stateDimension;
	    m_stateComponent.resize(m_stateDimension);
	    
	    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    {	
		m_stateComponent[i].type       = StateComponent::NORMAL;
		m_stateComponent[i].minValue   = km->stateBounds[i*2    ];
		m_stateComponent[i].maxValue   = km->stateBounds[i*2 + 1];
		m_stateComponent[i].resolution = (m_stateComponent[i].maxValue - m_stateComponent[i].minValue) / m_divisions;
	    }
	}

	virtual ~SpaceInformationNode(void)
	{
	}

	void setPlanningVolume(double x0, double y0, double z0, double x1, double y1, double z1)
	{
	    for (unsigned int i = 0 ; i < m_km->floatingJoints.size() ; ++i)
	    {
		int id = m_km->floatingJoints[i];		
		m_stateComponent[id    ].minValue = x0;
		m_stateComponent[id    ].maxValue = x1;
		m_stateComponent[id + 1].minValue = y0;
		m_stateComponent[id + 1].maxValue = y1;
		m_stateComponent[id + 2].minValue = z0;
		m_stateComponent[id + 2].maxValue = z1;
		for (int j = 0 ; j < 3 ; ++j)
		    m_stateComponent[j + id].resolution = (m_stateComponent[j + id].maxValue - m_stateComponent[j + id].minValue) / m_divisions;
	    }
	}
	
    protected:
	
	double                           m_divisions;	
	planning_models::KinematicModel *m_km;
	
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
    
    std::vector<std::string> mlist;    
    planner.knownModels(mlist);
    printf("Known models:\n");    
    for (unsigned int i = 0 ; i < mlist.size() ; ++i)
	printf("  * %s\n", mlist[i].c_str());    
    
    planner.spin();
    planner.shutdown();
    
    return 0;    
}
