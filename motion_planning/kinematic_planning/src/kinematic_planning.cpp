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
#include <robot_srvs/KinematicMotionPlan.h>

#include <urdf/URDF.h>
#include <collision_space/environmentODE.h>
#include <ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h>

#include <vector>
#include <string>
#include <sstream>
#include <map>

//////////// temp:
#include <display_ode/displayODE.h>
static display_ode::DisplayODESpaces spaces;

class KinematicPlanning : public ros::node
{
public:

    KinematicPlanning(void) : ros::node("kinematic_planning")
    {
	advertise_service("plan_kinematic_path", &KinematicPlanning::plan);
	subscribe("world_3d_map", m_cloud, &KinematicPlanning::pointCloudCallback);
	
	m_collisionSpace = new collision_space::EnvironmentModelODE();
	m_collisionSpace->setSelfCollision(false); // for now, disable self collision (incorrect model)
	
	m_collisionSpace->lock();	
	loadRobotDescriptions();
	m_collisionSpace->unlock();



	double sphere[3] = {0.8,0.2,0.4};    
	
	m_collisionSpace->addPointCloud(1, sphere, 0.15);
	///////////////////////
	collision_space::EnvironmentModelODE* okm = dynamic_cast<collision_space::EnvironmentModelODE*>(m_collisionSpace);
	spaces.addSpace(okm->getODESpace());
	for (unsigned int i = 0 ; i < okm->getModelCount() ; ++i)
	    spaces.addSpace(okm->getModelODESpace(i));
	//////////////////////

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
	
	const int dim = req.goal_state.vals_size;
	if ((int)p.si->getStateDimension() != dim)
	    return false;

	/* set the workspace volume for planning */
	static_cast<SpaceInformationNode*>(p.si)->setPlanningVolume(req.volumeMin.x, req.volumeMin.y, req.volumeMin.z,
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
	goal->threshold = 1e-6;
	p.si->setGoal(goal);
	
	/* do the planning */
	m_collisionSpace->lock();
	bool ok = p.mp->solve(req.allowed_time); 
	m_collisionSpace->unlock();
	
	/* copy the solution to the result */
	if (ok)
	{
	    ompl::SpaceInformationKinematic::PathKinematic_t path = static_cast<ompl::SpaceInformationKinematic::PathKinematic_t>(goal->getSolutionPath());
	    p.si->smoothVertices(path);	    
	    res.path.set_states_size(path->states.size());
	    for (unsigned int i = 0 ; i < path->states.size() ; ++i)
	    {
		res.path.states[i].set_vals_size(dim);
		for (int j = 0 ; j < dim ; ++j)
		    res.path.states[i].vals[j] = path->states[i]->values[j];
		
		// hack
		m->kmodel->computeTransforms(path->states[i]->values, m->groupID);
		m->collisionSpace->updateRobotModel(m->collisionSpaceID);
		usleep(1000000);
		//
		
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
	unsigned int cid = m_collisionSpace->addRobotModel(*file);
	model->collisionSpaceID = cid;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = m_collisionSpace->getModel(cid);
	m_models[name] = model;
	createMotionPlanningInstances(model);
	
	/* create a model for each group */
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    Model *model = new Model();
	    std::string gname = name + "::" + groups[i];
	    model->collisionSpaceID = cid;
	    model->collisionSpace = m_collisionSpace;
	    model->kmodel = m_collisionSpace->getModel(cid);
	    model->groupID = model->kmodel->getGroupID(gname);
	    m_models[gname] = model;
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
    
    class SpaceInformationNode : public ompl::SpaceInformationKinematic
    {
    public:
	SpaceInformationNode(Model *model) : SpaceInformationKinematic()
	{
	    m_m = model;
	    m_divisions = 20.0;
	    
	    m_stateDimension = m_m->groupID >= 0 ? m_m->kmodel->groupStateIndexList[m_m->groupID].size() : m_m->kmodel->stateDimension;
	    m_stateComponent.resize(m_stateDimension);
	    
	    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    {	
		m_stateComponent[i].type       = StateComponent::NORMAL;
		unsigned int p = m_m->groupID >= 0 ? m_m->kmodel->groupStateIndexList[m_m->groupID][i] * 2 : i * 2;
		m_stateComponent[i].minValue   = m_m->kmodel->stateBounds[p    ];
		m_stateComponent[i].maxValue   = m_m->kmodel->stateBounds[p + 1];
		m_stateComponent[i].resolution = (m_stateComponent[i].maxValue - m_stateComponent[i].minValue) / m_divisions;
	    }
	}

	virtual ~SpaceInformationNode(void)
	{
	}

	void setPlanningVolume(double x0, double y0, double z0, double x1, double y1, double z1)
	{
	    if (m_m->groupID < 0)
		for (unsigned int i = 0 ; i < m_m->kmodel->floatingJoints.size() ; ++i)
		{
		    int id = m_m->kmodel->floatingJoints[i];		
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
	
	double  m_divisions;	
	Model  *m_m;
	
    };    
	
    std_msgs::PointCloudFloat32        m_cloud;
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
	p.si   = new SpaceInformationNode(model);
	p.si->setStateValidFn(isStateValid, reinterpret_cast<void*>(model));
	p.mp   = new ompl::RRT(p.si);
	p.type = 0;	
	model->planners.push_back(p);
    }

};


////////////////////////////////////////////

static void start(void)
{
    static float xyz[3] = {-0.2179,1.5278,0.8700};
    static float hpr[3] = {-77.5000,-19.5000,0.0000};
    
    dsSetViewpoint(xyz, hpr);
}

static void command(int cmd)
{
}

static void simLoop(int)
{
    spaces.displaySpaces();
}
////////////////////////////////////////////


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    KinematicPlanning planner;
    
    std::vector<std::string> mlist;    
    planner.knownModels(mlist);
    printf("Known models:\n");    
    for (unsigned int i = 0 ; i < mlist.size() ; ++i)
	printf("  * %s\n", mlist[i].c_str());    
    
    //    planner.spin();


    ////////////////////
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "./res";
    
    dsSimulationLoop(argc, argv, 640, 480, &fn);
    //////////////////


    planner.shutdown();
    
    return 0;    
}
