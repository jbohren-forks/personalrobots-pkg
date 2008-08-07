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

@b PlanningWorldViewer is a node capable of displaying the state of the world 
the motion planner sees

<hr>

@section usage Usage
@verbatim
$ planning_world_viewer [standard ROS args]
@endverbatim

@par Example

@verbatim
$ planning_world_viewer
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
- None

<hr>

@section parameters ROS parameters
- None

**/

#include <ros/node.h>
#include <ros/time.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/RobotBase2DOdom.h>

#include <robot_msgs/NamedKinematicPath.h>

#include <urdf/URDF.h>
#include <collision_space/environmentODE.h>
#include <display_ode/displayODE.h>

#include <vector>
#include <string>
#include <sstream>
#include <map>

class PlanningWorldViewer : public ros::node
{
public:
    
    PlanningWorldViewer(void) : ros::node("planning_world_viewer")
    {
	subscribe("world_3d_map", m_cloud, &PlanningWorldViewer::pointCloudCallback);
	subscribe("localizedpose", m_localizedPose, &PlanningWorldViewer::localizedPoseCallback);
	subscribe("display_kinematic_path", m_displayPath, &PlanningWorldViewer::displayPathCallback);
	
	m_collisionSpace = new collision_space::EnvironmentModelODE();
	m_basePos[0] = m_basePos[1] = m_basePos[2] = 0.0;
	m_follow = true;
	
	m_collisionSpace->lock();
	loadRobotDescriptions();
	m_collisionSpace->unlock();
	
	updateODESpaces();
    }
    
    ~PlanningWorldViewer(void)
    {
	for (unsigned int i = 0 ; i < m_robotDescriptions.size() ; ++i)
	    delete m_robotDescriptions[i];
	if (m_collisionSpace)
	    delete m_collisionSpace;
    }
    
    void updateODESpaces(void)
    {
	m_spaces.addSpace(m_collisionSpace->getODESpace(), 1.0f, 0.0f, 0.0f);
	for (unsigned int i = 0 ; i < m_collisionSpace->getModelCount() ; ++i)
	    m_spaces.addSpace(m_collisionSpace->getModelODESpace(i), 0.1f, 0.5f, (float)(i + 1)/(float)m_collisionSpace->getModelCount());
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
	
	if (m_collisionSpace->getModelCount() == 1 && m_follow)
	{
	    int group = m_collisionSpace->getModel(0)->getGroupID("pr2::base");
	    if (group >= 0)
	    {
		m_collisionSpace->lock();
		m_collisionSpace->getModel(0)->computeTransforms(m_basePos, group);
		m_collisionSpace->updateRobotModel(0);
		m_collisionSpace->unlock();
	    }	    
	}
	
    }
    
    void pointCloudCallback(void)
    {
	unsigned int n = m_cloud.get_pts_size();
	printf("received %u points\n", n);
	
	m_spaces.clear();	
	
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
	
	updateODESpaces();
	
	double tupd = (ros::Time::now() - startTime).to_double();	
	printf("Updated world model in %f seconds\n", tupd);
	
    }
    
    void displayPathCallback(void)
    {
	if (m_collisionSpace->getModelCount() != 1)
	    return;
	
	ros::Duration sleepTime(0.2);
	planning_models::KinematicModel *kmodel = m_collisionSpace->getModel(0);
	int groupID = kmodel->getGroupID(m_displayPath.name);
	
	for (unsigned int i = 0 ; i < m_displayPath.path.get_states_size() ; ++i)
	{
	    kmodel->computeTransforms(m_displayPath.path.states[i].vals, groupID);
	    m_collisionSpace->updateRobotModel(0);
	    sleepTime.sleep();
	}
	
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

	double defaultPose[kmodel->stateDimension];
	for (unsigned int i = 0 ; i < kmodel->stateDimension ; ++i)
	    defaultPose[i] = 0.0;
	
	kmodel->computeTransforms(defaultPose);
	m_collisionSpace->updateRobotModel(cid);

    }
    
    unsigned int getRobotCount(void) const
    {
	return m_robotDescriptions.size();	
    }
    
    bool getFollow(void) const
    {
	return m_follow;
    }
    
    void setFollow(bool follow)
    {
	m_follow = follow;
    }
    
    const double* getBasePos(void) const
    {
	return m_basePos;
    }
    
    void display(void)
    {
	m_collisionSpace->lock();    
	m_spaces.displaySpaces();
	m_collisionSpace->unlock();
    }
    
private:
    
    std_msgs::PointCloudFloat32           m_cloud;
    std_msgs::RobotBase2DOdom             m_localizedPose;
    collision_space::EnvironmentModelODE *m_collisionSpace;    
    std::vector<robot_desc::URDF*>        m_robotDescriptions;
    double                                m_basePos[3];
    
    display_ode::DisplayODESpaces         m_spaces;
    robot_msgs::NamedKinematicPath        m_displayPath;
    bool                                  m_follow;
    
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
	    float xyz[3] = {basePos[0], basePos[1], 1.0};
	    float hpr[3] = {-77.5000,-19.5000,0.0000};    
	    dsSetViewpoint(xyz, hpr);
	}
	break;
    case 'f':
    case 'F':
	viewer->setFollow(!viewer->getFollow());	
	break;
    default:
	break;
    }
}

static void simLoop(int)
{
    viewer->display();
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    viewer = new PlanningWorldViewer();
    
    if (viewer->getRobotCount() > 0)
    {
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start   = &start;
	fn.step    = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "./res";
	
	dsSimulationLoop(argc, argv, 640, 480, &fn);
    }
    else
	printf("No models defined. Kinematic planning node cannot start.\n");
    
    viewer->shutdown();
    
    delete viewer;
    
    return 0;    
}
