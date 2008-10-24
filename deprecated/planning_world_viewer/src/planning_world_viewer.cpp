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

@b PlanningWorldViewer is a node capable of displaying the state of the world 
the motion planner sees

<hr>

@section usage Usage
@verbatim
$ planning_world_viewer robot_model [standard ROS args]
@endverbatim

@par Example

@verbatim
$ planning_world_viewer robotdesc/pr2
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
- None

<hr>

@section parameters ROS parameters
- None

**/

#include <planning_node_util/cnode.h>
#include <rosthread/mutex.h>
#include <robot_msgs/DisplayKinematicPath.h>
#include <display_ode/displayODE.h>

#include <vector>
#include <string>
#include <sstream>
#include <map>

class PlanningWorldViewer : public ros::node,
			    public planning_node_util::NodeCollisionModel
{
public:
    
    PlanningWorldViewer(const std::string &robot_model) : ros::node("planning_world_viewer"),
							  planning_node_util::NodeCollisionModel(dynamic_cast<ros::node*>(this),
												 robot_model,
												 new collision_space::EnvironmentModelODE())
    {
	subscribe("display_kinematic_path", m_displayPath, &PlanningWorldViewer::displayPathCallback, 1);
	m_collisionSpaceODE = dynamic_cast<collision_space::EnvironmentModelODE*>(m_collisionSpace);
	
	m_follow = true;
	m_displayRobot = true;
	m_displayObstacles = true;
	m_checkCollision = false;
    }
    
    ~PlanningWorldViewer(void)
    {
    }
    
    void updateODESpaces(void)
    {
	if (!m_collisionSpaceODE)
	    return;
	
	m_collisionSpaceODE->lock();
	m_displayLock.lock();	
	m_spaces.clear();	
	if (m_displayObstacles)
	    m_spaces.addSpace(m_collisionSpaceODE->getODESpace(), 1.0f, 0.0f, 0.0f);
	if (m_displayRobot)
	    for (unsigned int i = 0 ; i < m_collisionSpaceODE->getModelCount() ; ++i)
	    m_spaces.addSpace(m_collisionSpaceODE->getModelODESpace(i), 0.1f, 0.5f, (float)(i + 1)/(float)m_collisionSpaceODE->getModelCount());
	m_displayLock.unlock();
	m_collisionSpaceODE->unlock();
    }
    
    bool checkCollision(void)
    {
	bool result = false;
	if (m_checkCollision && m_collisionSpace && m_collisionSpace->getModelCount() == 1)
	{
	    m_collisionSpace->lock();
	    ros::Time startTime = ros::Time::now();
	    result = m_collisionSpace->isCollision(0);
	    printf("Collision: %d     [%f s]\n", result, (ros::Time::now() - startTime).to_double());
	    m_collisionSpace->unlock();
	}
	return result;
    }
    
    void stateUpdate(void)
    {
	planning_node_util::NodeCollisionModel::stateUpdate();
	
	if (m_collisionSpace && m_collisionSpace->getModelCount() == 1 && m_follow)
	{
	    m_collisionSpace->lock();
	    m_collisionSpace->getRobotModel(0)->computeTransforms(m_robotState->getParams());
	    m_collisionSpace->updateRobotModel(0);
	    m_collisionSpace->unlock();
	    checkCollision();
	}
    }
    
    virtual void beforeWorldUpdate(void)
    {
	planning_node_util::NodeCollisionModel::beforeWorldUpdate();
	m_displayLock.lock();	
	m_spaces.clear();
	m_displayLock.unlock();	
    }

    virtual void afterWorldUpdate(void)
    {
	planning_node_util::NodeCollisionModel::afterWorldUpdate();
	updateODESpaces();
    }	
    
    void displayPathCallback(void)
    {
	bool follow = m_follow;
	m_follow = false;
	ros::Duration sleepTime(0.4);
	int groupID = m_kmodel->getGroupID(m_displayPath.name);
	
	for (unsigned int i = 0 ; i < m_displayPath.path.get_states_size() ; ++i)
	{
	    m_kmodel->computeTransforms(m_displayPath.path.states[i].vals, groupID);
	    m_collisionSpace->updateRobotModel(0);
	    checkCollision();
	    sleepTime.sleep();
	}
	m_follow = follow;
    }
    
    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	planning_node_util::NodeCollisionModel::setRobotDescription(file);
	defaultPosition();
    }
    
    bool getFollow(void) const
    {
	return m_follow;
    }
    
    void setFollow(bool follow)
    {
	m_follow = follow;
    }
    
    bool getCheckCollision(void) const
    {
	return m_checkCollision;
    }
    
    void setCheckCollision(bool collision)
    {
	m_checkCollision = collision;
    }
    
    bool getDisplayRobot(void) const
    {
	return m_displayRobot;
    }
    
    void setDisplayRobot(bool display)
    {
	if (m_displayRobot != display)
	{
	    m_displayRobot = display;
	    updateODESpaces();
	}	
    }
    
    bool getDisplayObstacles(void) const
    {
	return m_displayObstacles;
    }
    
    void setDisplayObstacles(bool display)
    {
	if (m_displayObstacles != display)
	{
	    m_displayObstacles = display;
	    updateODESpaces();
	}	
    }
    
    const double* getBasePos(void) const
    {
	return m_basePos;
    }
    
    void display(void)
    {
	m_displayLock.lock();
	m_spaces.displaySpaces();
	m_displayLock.unlock();
    }
    
    void printLinkPoses(void)
    {
	m_kmodel->printLinkPoses();
    }
    
private:
    
    collision_space::EnvironmentModelODE* m_collisionSpaceODE;    
    display_ode::DisplayODESpaces         m_spaces;
    ros::thread::mutex                    m_displayLock;
    
    robot_msgs::DisplayKinematicPath      m_displayPath;
    bool                                  m_follow;
    bool                                  m_displayRobot;
    bool                                  m_displayObstacles;  
    bool                                  m_checkCollision;
        
};

static PlanningWorldViewer *viewer = NULL;

static void start(void)
{
    static float xyz[3] = {0.0, 0.0 ,0.8700};
    static float hpr[3] = {-77.5000,-19.5000,0.0000};    
    dsSetViewpoint(xyz, hpr);
}

static void command(int cmd)
{
    const char key = (char)cmd;

    switch (key)
    {
    case 't':
    case 'T':
	{
	    const double *basePos = viewer->getBasePos();	    
	    float xyz[3] = {basePos[0], basePos[1], 6.0};
	    float hpr[3] = {basePos[2] * 180.0 / M_PI, -70.000,0.000};
	    dsSetViewpoint(xyz, hpr);
	}
	break;
    case 'f':
    case 'F':
	viewer->setFollow(!viewer->getFollow());	
	break;
    case 'c':
    case 'C':
	viewer->setCheckCollision(!viewer->getCheckCollision());	
	break;

    case 'R':
	viewer->setDisplayRobot(!viewer->getDisplayRobot());
	break;
    case 'O':
	viewer->setDisplayObstacles(!viewer->getDisplayObstacles());
	break;
    case 'D':
	viewer->defaultPosition();
	break;
    case 'p':
    case 'P':
	viewer->printLinkPoses();
	break;	
    default:
	break;
    }
}

static void simLoop(int)
{
    viewer->display();
}

void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used when showing the map.\n");
}

int main(int argc, char **argv)
{  
    if (argc == 2)
    {	
	ros::init(argc, argv);
        
	viewer = new PlanningWorldViewer(argv[1]);
	viewer->loadRobotDescription();
	viewer->updateODESpaces();

	
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start   = &start;
	fn.step    = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "./res";
	
	dsSimulationLoop(argc, argv, 640, 480, &fn);
	
	viewer->shutdown();
	
	delete viewer;
    }
    else
	usage(argv[0]);
    
    return 0;    
}
