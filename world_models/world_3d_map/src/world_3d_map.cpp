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

@b world_3d_map is a node capable of building 3D maps out of laser
scan data.  The node will put together a pointcloud in the FRAMEID_MAP
frame and will publish the pointcloud with the data it retained. The
node also downsamples the input data, to avoid a large number of
obstacles, removes the points in the cloud that intersect with a given
robot model and allows for limiting the frequency at which this map is
published.

<hr>

@section usage Usage
@verbatim
$ world_3d_map [standard ROS args]
@endverbatim

@par Example

@verbatim
$ world_3d_map
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b scan/LaserScan : scan data received from a laser
- @b robotpose/RobotBase2DOdom : position of the robot base

Publishes to (name/type):
- @b "world_3d_map"/PointCloudFloat32 : point cloud describing the 3D environment
- @b "roserr"/Log : output log messages

<hr>

Uses (name/type):
- None

Provides (name/type):
- None

<hr>

@section parameters ROS parameters
- @b "world_3d_map/max_publish_frequency" : @b [double] the maximum frequency (Hz) at which the data in the built 3D map is to be sent (default 0.5)
- @b "world_3d_map/retain_pointcloud_duration : @b [double] the time for which a point cloud is retained as part of the current world information (default 2)
- @b "world_3d_map/retain_pointcloud_fraction : @b [double] the time for which a point cloud is retained as part of the current world information (default 2)
- @b "world_3d_map/verbosity_level" : @b [int] sets the verbosity level (default 1)
**/

#include <ros/node.h>
#include <ros/time.h>
#include <rosthread/member_thread.h>
#include <rosthread/mutex.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/LaserScan.h>
#include <rostools/Log.h>

#include <rosTF/rosTF.h>
#include <urdf/URDF.h>
#include <planning_models/kinematic.h>
#include <collision_space/util.h>
#include <random_utils/random_utils.h>

#include <deque>
#include <cmath>

class World3DMap : public ros::node
{
public:

    World3DMap(const char *robot_model) : ros::node("world_3d_map"),
					  m_tf(*this, true, 1 * 1000000000ULL, 1000000000ULL)
    {
	advertise<std_msgs::PointCloudFloat32>("world_3d_map");
	advertise<rostools::Log>("roserr");

	param("max_publish_frequency", m_maxPublishFrequency, 0.5);
	param("retain_pointcloud_duration", m_retainPointcloudDuration, 60.0);
	param("retain_pointcloud_fraction", m_retainPointcloudFraction, 0.02);
	param("verbosity_level", m_verbose, 1);
	
	loadRobotDescription(robot_model);
	
	/* create a thread that does the processing of the input data.
	 * and one that handles the publishing of the data */
	m_active = true;
	m_working = false;
	m_shouldPublish = false;
	random_utils::init(&m_rng);
	m_basePos[0] = m_basePos[1] = m_basePos[2] = 0.0;
	
	m_processMutex.lock();
	m_publishMutex.lock();
	m_processingThread = ros::thread::member_thread::startMemberFunctionThread<World3DMap>(this, &World3DMap::processDataThread);
	m_publishingThread = ros::thread::member_thread::startMemberFunctionThread<World3DMap>(this, &World3DMap::publishDataThread);

	subscribe("scan", m_inputScan, &World3DMap::pointCloudCallback);
	subscribe("localizedpose", m_localizedPose, &World3DMap::localizedPoseCallback);
    }
    
    ~World3DMap(void)
    {
	/* terminate spawned threads */
	m_active = false;
	m_processMutex.unlock();
	
	pthread_join(*m_publishingThread, NULL);
	pthread_join(*m_processingThread, NULL);
	for (unsigned int i = 0 ; i < m_currentWorld.size() ; ++i)
	    delete m_currentWorld[i];

	for (unsigned int i = 0 ; i < m_selfSeeParts.size() ; ++i)
	    delete m_selfSeeParts[i].body;

	if (m_kmodel)
	    delete m_kmodel;
	if (m_urdf)
	    delete m_urdf;
    }
    
private:
    
    struct RobotPart
    {
	collision_space::bodies::Object       *body;
	planning_models::KinematicModel::Link *link;	
    };
  
    void setRobotDescriptionFromData(const char *data)
    {
	robot_desc::URDF *file = new robot_desc::URDF();
	if (file->loadString(data))
	    setRobotDescription(file);
	else
	    delete file;
    }

    void setRobotDescription(robot_desc::URDF *file)
    {
	m_urdf = file;
	m_kmodel = new planning_models::KinematicModel();
	m_kmodel->setVerbose(false);
	m_kmodel->build(*file);
	
	addSelfSeeBodies();
    }
  
    void loadRobotDescription(const char *robot_model)
    {
	if (!robot_model || strcmp(robot_model, "-") == 0)
	{
	    if (m_verbose)
		printf("No robot model will be used\n");
	}
	else
	{
	    std::string content;
	    if (m_verbose)
		printf("Attempting to load model '%s'\n", robot_model);
	    if (get_param(robot_model, content))
	    {
		setRobotDescriptionFromData(content.c_str());
		if (m_verbose)
		    printf("Success!\n");
	    }	
	    else
		fprintf(stderr, "Robot model '%s' not found!\n", robot_model);
	}
    }
    
    void localizedPoseCallback(void)
    {
	bool success = true;
	libTF::TFPose2D pose;
	pose.x = m_localizedPose.pos.x;
	pose.y = m_localizedPose.pos.y;
	pose.yaw = m_localizedPose.pos.th;
	pose.time = m_localizedPose.header.stamp.to_ull();
	pose.frame = m_localizedPose.header.frame_id;
	
	try
	{
	    pose = m_tf.transformPose2D("FRAMEID_MAP", pose);
	}
	catch(libTF::TransformReference::LookupException& ex)
	{
	    fprintf(stderr, "Discarding pose: Transform reference lookup exception\n");
	    success = false;
	}
	catch(libTF::TransformReference::ExtrapolateException& ex)
	{
	    fprintf(stderr, "Discarding pose: Extrapolation exception: %s\n", ex.what());
	    success = false;
	}
	catch(...)
	{
	    fprintf(stderr, "Discarding pose: Exception in pose computation\n");
	    success = false;
	}
	
	if (success)
	{
	    m_basePos[0] = pose.x;
	    m_basePos[1] = pose.y;
	    m_basePos[2] = pose.yaw;
	    
	    int group = m_kmodel->getGroupID(m_urdf->getRobotName() + "::base");
	    if (group >= 0)
		m_kmodel->computeTransforms(m_basePos, group);
	}
	
    }
    
    void pointCloudCallback(void)
    {
	/* The idea is that if processing of previous input data is
	   not done, data will be discarded. Hopefully this discarding
	   of data will not happen, but we don't want the node to
	   postpone processing latest data just because it is not done
	   with older data. */
	
	m_flagMutex.lock();
	bool discard = m_working;
	if (!discard)
	    m_working = true;
	
	if (discard)
	{
	    /* log the fact that input was discarded */
	    rostools::Log l;
	    l.level = 20;
	    l.name  = get_name();
	    l.msg   = "Discarded point cloud data (previous input set not done processing)";
	    publish("roserr", l);
	    fprintf(stderr, "Discarding pointcloud: Too much data to process\n");
	}
	else
	{
	    /* copy data to a place where incoming messages do not affect it */
	    bool success = true;
	    try
	    {
		m_tf.transformLaserScanToPointCloud("FRAMEID_MAP", m_toProcess, m_inputScan);
	    }
	    catch(libTF::TransformReference::LookupException& ex)
	    {
		fprintf(stderr, "Discarding pointcloud: Transform reference lookup exception\n");
		success = false;		
	    }
	    catch(libTF::TransformReference::ExtrapolateException& ex)
	    {
		fprintf(stderr, "Discarding pointcloud: Extrapolation exception: %s\n", ex.what());
		success = false;
	    }
	    catch(...)
	    {
		fprintf(stderr, "Discarding pointcloud: Exception in point cloud computation\n");
		success = false;
	    }  
	    if (success)
		m_processMutex.unlock();   /* let the processing thread know that there is data to process */
	    else 
		m_working = false;	   /* unmark the fact we are working */
	}
	m_flagMutex.unlock();
    }
    
    void processDataThread(void)
    {
	while (m_active)
	{
	    /* This mutex acts as a condition, but is safer (condition
	       messages may get lost or interrupted by signals) */
	    m_processMutex.lock();
	    if (m_active)
	    {
		m_worldDataMutex.lock();
		processData();
		m_worldDataMutex.unlock();
		
		/* notify the publishing thread that it can send data */
		m_shouldPublish = true;
	    }
	    
	    /* make a note that there is no active processing */
	    m_flagMutex.lock();
	    m_working = false;
	    m_flagMutex.unlock();
	}
    }
    

    void publishDataThread(void)
    {
	ros::Duration *d = new ros::Duration(1.0/m_maxPublishFrequency);
	
	/* while everything else is running (map building) check if
	   there are any updates to send, but do so at most at the
	   maximally allowed frequency of sending data */
	while (m_active)
	{
	    d->sleep();

	    if (m_shouldPublish)
	    {
		m_worldDataMutex.lock();
		if (m_active && m_currentWorld.size() > 0)
		{
		    std_msgs::PointCloudFloat32 toPublish;
		    toPublish.header = m_currentWorld.back()->header;
		    
		    unsigned int      npts  = 0;
		    for (unsigned int i = 0 ; i < m_currentWorld.size() ; ++i)
			npts += m_currentWorld[i]->get_pts_size();
		    
		    toPublish.set_pts_size(npts);
		    
		    unsigned int j = 0;
		    for (unsigned int i = 0 ; i < m_currentWorld.size() ; ++i)
		    {
			unsigned int n = m_currentWorld[i]->get_pts_size();			
			for (unsigned int k = 0 ; k < n ; ++k)
			    toPublish.pts[j++] =  m_currentWorld[i]->pts[k];
		    }
		    
		    toPublish.set_pts_size(j);
		    if (ok())
		    {
			if (m_verbose)
			    printf("Publishing a point cloud with %u points\n", toPublish.get_pts_size());
			publish("world_3d_map", toPublish);
		    }
		}
		m_shouldPublish = false;
		m_worldDataMutex.unlock();
	    }
	}
	delete d;
    }
    
    void addSelfSeeBodies(void)
    {
	robot_desc::URDF::Group *ss = m_urdf->getGroup("self_see");
	if (ss)
	{
	    for (unsigned int i = 0 ; i < ss->linkNames.size() ; ++i)
	    {
		planning_models::KinematicModel::Link *link = m_kmodel->getLink(ss->linkNames[i]);
		if (link)
		{
		    RobotPart rp = { NULL, link };    
		    
		    switch (link->geom->type)
		    {
		    case planning_models::KinematicModel::Geometry::BOX:
			rp.body = new collision_space::bodies::Box();
			break;			
		    case planning_models::KinematicModel::Geometry::SPHERE:
			rp.body = new collision_space::bodies::Sphere();
			break;
		    case planning_models::KinematicModel::Geometry::CYLINDER:
			rp.body = new collision_space::bodies::Cylinder();
			break;
		    default:
			break;
		    }
		    
		    if (!rp.body)
		    {
			fprintf(stderr, "Unknown body type: %d\n", link->geom->type);
			continue;
		    }
		    
		    rp.body->setDimensions(link->geom->size);
		    rp.body->setScale(1.5);
		    
		    m_selfSeeParts.push_back(rp);
		}
	    }
	}
	
	if (m_verbose)
	    printf("Ignoring point cloud data that intersects with %d robot parts\n", m_selfSeeParts.size());
    }
    
    void processData(void)
    {
	/* remove old data */
	ros::Time &time = m_toProcess.header.stamp;

	while (!m_currentWorld.empty() && (time - m_currentWorld.front()->header.stamp).to_double() > m_retainPointcloudDuration)
	{
	    std_msgs::PointCloudFloat32* old = m_currentWorld.front();
	    m_currentWorld.pop_front();
	    delete old;
	}

	/* add new data */
	std_msgs::PointCloudFloat32 *newData = runFilters(m_toProcess);
	if (newData)
	{
	    if (newData->get_pts_size() == 0)
		delete newData;
	    else
		m_currentWorld.push_back(newData);
	}
	
	if (m_verbose)
	{
	    double window = m_currentWorld.size() > 0 ? (m_currentWorld.back()->header.stamp - m_currentWorld.front()->header.stamp).to_double() : 0.0;
	    printf("World map containing %d point clouds (window size = %f seconds)\n", m_currentWorld.size(), window);
	}	
    }
    
    std_msgs::PointCloudFloat32* runFilters(const std_msgs::PointCloudFloat32 &cloud)
    {
	std_msgs::PointCloudFloat32 *cloudF = filter0(cloud, m_retainPointcloudFraction);
	
	if (cloudF)
	{
	    std_msgs::PointCloudFloat32 *temp = filter1(*cloudF);
	    delete cloudF;
	    cloudF = temp;
	}
	
	return cloudF;
    }
    
    /* Remove invalid floating point values and strip channel iformation.
     * Also keep a certain ratio of the cloud information only */
    std_msgs::PointCloudFloat32* filter0(const std_msgs::PointCloudFloat32 &cloud, double frac = 1.0)
    {
	std_msgs::PointCloudFloat32 *copy = new std_msgs::PointCloudFloat32();
	copy->header = cloud.header;

	unsigned int n = cloud.get_pts_size();
	unsigned int j = 0;
	copy->set_pts_size(n);	
	for (unsigned int k = 0 ; k < n ; ++k)
	    if (random_utils::uniform(&m_rng, 0.0, 1.0) < frac)
		if (isfinite(cloud.pts[k].x) && isfinite(cloud.pts[k].y) && isfinite(cloud.pts[k].z))
		    copy->pts[j++] = cloud.pts[k];
	copy->set_pts_size(j);

	return copy;	
    }    
    
    std_msgs::PointCloudFloat32* filter1(const std_msgs::PointCloudFloat32 &cloud)
    {
	std_msgs::PointCloudFloat32 *copy = new std_msgs::PointCloudFloat32();
	copy->header = cloud.header;
	
	for (int i = m_selfSeeParts.size() - 1 ; i >= 0 ; --i)
	    m_selfSeeParts[i].body->setPose(m_selfSeeParts[i].link->globalTrans);

	unsigned int n = cloud.get_pts_size();
	unsigned int j = 0;
	copy->set_pts_size(n);	
	for (unsigned int k = 0 ; k < n ; ++k)
	{
	    double x = cloud.pts[k].x;
	    double y = cloud.pts[k].y;
	    double z = cloud.pts[k].z;
	    
	    if (z > 0.01)
	    {
		bool keep = true;
		for (int i = m_selfSeeParts.size() - 1 ; keep && i >= 0 ; --i)
		    keep = !m_selfSeeParts[i].body->containsPoint(x, y, z);
		
		if (keep)
		    copy->pts[j++] = cloud.pts[k];
	    }
	}
	if (m_verbose)
	    printf("Discarded %d points\n", n - j);
	
	copy->set_pts_size(j);

	return copy;
    }
    
    rosTFClient                              m_tf;

    robot_desc::URDF                        *m_urdf;
    planning_models::KinematicModel         *m_kmodel;	
    
    std::vector<RobotPart>                   m_selfSeeParts;
    double                                   m_basePos[3];

    std::deque<std_msgs::PointCloudFloat32*> m_currentWorld;// Pointers to saved clouds


    
    double                           m_maxPublishFrequency;
    double                           m_retainPointcloudDuration;
    double                           m_retainPointcloudFraction;    
    int                              m_verbose;
    
    std_msgs::LaserScan              m_inputScan; //Buffer for recieving cloud
    std_msgs::PointCloudFloat32      m_toProcess; //Buffer (size 1) for incoming cloud
    std_msgs::RobotBase2DOdom        m_localizedPose;


    pthread_t                       *m_processingThread;
    pthread_t                       *m_publishingThread;
    ros::thread::mutex               m_processMutex, m_publishMutex, m_worldDataMutex, m_flagMutex;
    bool                             m_active, m_working, m_shouldPublish;
    random_utils::rngState           m_rng;

};

void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used when building the map.\n");
    printf("       This allows for removing parts of the laser scan that see the robot itself.\n");
    printf("       If no robot model is to be used, specify '-' for the \"robot_model\" argument.\n\n");
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    if (argc == 2)
    {
	World3DMap map(argv[1]);
	map.spin();
	map.shutdown();
    }
    else
	usage(argv[0]);
    
    return 0;    
}
