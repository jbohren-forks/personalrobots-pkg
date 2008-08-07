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

@b world_3d_map is a node capable of building 3D maps out of point
cloud data. The code is incomplete: it currently only forwards cloud
data.

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
- @b full_cloud/PointCloudFloat32 : point cloud with data from a complete laser scan (top to bottom)

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
- @b "world_3d_map/verbosity_level" : @b [int] sets the verbosity level (default 1)
**/

#include <ros/node.h>
#include <ros/time.h>
#include <rosthread/member_thread.h>
#include <rosthread/mutex.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/LaserScan.h>
#include <rostools/Log.h>

#include <rosTF/rosTF.h>
#include <urdf/URDF.h>
#include <planning_models/kinematic.h>
#include <collision_space/util.h>
#include <random_utils/random_utils.h>

#include <deque>
#include <cmath>

using namespace std_msgs;
using namespace ros::thread::member_thread;

class World3DMap : public ros::node
{
public:

    World3DMap(void) : ros::node("world_3d_map"),
		       m_tf(*this, true, 1 * 1000000000ULL, 1000000000ULL)
    {
	advertise<PointCloudFloat32>("world_3d_map");
	advertise<rostools::Log>("roserr");

	subscribe("tilt_scan", m_inputScan, &World3DMap::pointCloudCallback);
	
	param("world_3d_map/max_publish_frequency", m_maxPublishFrequency, 0.5);
	param("world_3d_map/retain_pointcloud_duration", m_retainPointcloudDuration, 60.0);
	param("world_3d_map/retain_pointcloud_fraction", m_retainPointcloudFraction, 0.02);
	param("world_3d_map/verbosity_level", m_verbose, 1);
	
	m_verbose = 0;
	
	loadRobotDescriptions();
	
	/* create a thread that does the processing of the input data.
	 * and one that handles the publishing of the data */
	m_active = true;
	m_working = false;
	m_shouldPublish = false;
	random_utils::init(&m_rng);
	
	m_processMutex.lock();
	m_publishMutex.lock();
	m_processingThread = startMemberFunctionThread<World3DMap>(this, &World3DMap::processDataThread);
	m_publishingThread = startMemberFunctionThread<World3DMap>(this, &World3DMap::publishDataThread);
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

	for (unsigned int i = 0 ; i < m_robotDescriptions.size() ; ++i)
	{
	    delete m_robotDescriptions[i].urdf;
	    delete m_robotDescriptions[i].kmodel;
	}	
    }
    
    void loadRobotDescriptions(void)
    {
	std::string model;
	if (get_param("world_3d_map/robot_model", model))
	{
	    std::string content;
	    if (get_param(model, content))
		addRobotDescriptionFromData(content.c_str());
	}
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
	planning_models::KinematicModel *kmodel = new planning_models::KinematicModel();
	kmodel->setVerbose(false);
	kmodel->build(*file);
	
	RobotDesc rd = { file, kmodel };
	m_robotDescriptions.push_back(rd);
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
	    catch(libTF::Pose3DCache::ExtrapolateException& ex)
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
		if (m_active)
		{
		    PointCloudFloat32 toPublish;
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
			if (m_verbose > 0)
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
    
    /* Remove invalid floating point values and strip channel iformation.
     * Also keep a certain ratio of the cloud information only */
    PointCloudFloat32* filter0(const PointCloudFloat32 &cloud, double frac = 1.0)
    {
	PointCloudFloat32 *copy = new PointCloudFloat32();
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
    
    PointCloudFloat32* runFilters(const PointCloudFloat32 &cloud)
    {
	PointCloudFloat32 *cloud0 = filter0(cloud, m_retainPointcloudFraction);
	
	return cloud0;
    }
    
    void processData(void)
    {
	/* remove old data */
	ros::Time &time = m_toProcess.header.stamp;

	while (!m_currentWorld.empty() && (time - m_currentWorld.front()->header.stamp).to_double() > m_retainPointcloudDuration)
	{
	    PointCloudFloat32* old = m_currentWorld.front();
	    m_currentWorld.pop_front();
	    delete old;
	}

	/* add new data */
	PointCloudFloat32 *newData = runFilters(m_toProcess);
	if (newData)
	{
	    if (newData->get_pts_size() == 0)
		delete newData;
	    else
		m_currentWorld.push_back(newData);
	}
	
	if (m_verbose > 0)
	{
	    double window = m_currentWorld.size() > 0 ? (m_currentWorld.back()->header.stamp - m_currentWorld.front()->header.stamp).to_double() : 0.0;
	    printf("World map containing %d point clouds (window size = %f seconds)\n", m_currentWorld.size(), window);
	}
	
    }
    
private:
    
    struct RobotDesc
    {
	robot_desc::URDF                *urdf;
	planning_models::KinematicModel *kmodel;	
    };    
    
    std::vector<RobotDesc>         m_robotDescriptions;
    
    LaserScan                      m_inputScan; //Buffer for recieving cloud
    PointCloudFloat32              m_toProcess; //Buffer (size 1) for incoming cloud
    std::deque<PointCloudFloat32*> m_currentWorld;// Pointers to saved clouds
    rosTFClient                    m_tf;

    double             m_maxPublishFrequency;
    double             m_retainPointcloudDuration;
    double             m_retainPointcloudFraction;    
    int                m_verbose;
    
    pthread_t         *m_processingThread;
    pthread_t         *m_publishingThread;
    ros::thread::mutex m_processMutex, m_publishMutex, m_worldDataMutex, m_flagMutex;
    bool               m_active, m_working, m_shouldPublish;

    random_utils::rngState m_rng;    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);

    World3DMap map;
    map.spin();
    map.shutdown();
    
    return 0;    
}
