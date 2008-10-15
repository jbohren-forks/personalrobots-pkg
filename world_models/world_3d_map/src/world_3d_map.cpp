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

@b world_3d_map is a node capable of building 3D maps out of laser
scan data.  The node will put together a pointcloud in the map
frame and will publish the pointcloud with the data it retained. The
node also downsamples the input data, to avoid a large number of
obstacles, removes the points in the cloud that intersect with a given
robot model and allows for limiting the frequency at which this map is
published.

<hr>

@section usage Usage
@verbatim
$ world_3d_map robot_model [standard ROS args]
@endverbatim


@par Example

@verbatim
$ world_3d_map robotdesc/pr2 scan:=tilt_scan
@endverbatim

@par Notes

@b robot_model is the name (string) of a robot description to be used
when building the map. This allows for removing parts of the laser
scan that see the robot itself. If no robot model is to be used,
specify '-' for the robot_model argument.

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b scan/LaserScan : scan data received from a laser
- @b cloud/PointCloudFloat32 : cloud data 

Additional subscriptions due to inheritance from NodeRobotModel:
- @b localizedpose/RobotBase2DOdom : localized position of the robot base

Publishes to (name/type):
- @b "world_3d_map"/PointCloudFloat32 : point cloud describing the 3D environment

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

#include <planning_node_util/knode.h>

#include <rosthread/member_thread.h>
#include <rosthread/mutex.h>

#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/LaserScan.h>

#include <collision_space/util.h>
#include <random_utils/random_utils.h>

// Laser projection
#include "laser_scan_utils/laser_scan.h"

#include <deque>
#include <cmath>

class World3DMap : public ros::node,
		   public planning_node_util::NodeRobotModel
{
public:
    
    World3DMap(const std::string &robot_model) : ros::node("world_3d_map"),
						 planning_node_util::NodeRobotModel(dynamic_cast<ros::node*>(this), robot_model)
    {
	advertise<std_msgs::PointCloudFloat32>("world_3d_map", 1);

	param("world_3d_map/max_publish_frequency", m_maxPublishFrequency, 0.5);
	param("world_3d_map/retain_pointcloud_duration", m_retainPointcloudDuration, 1.0);
	param("world_3d_map/retain_pointcloud_fraction", m_retainPointcloudFraction, 0.25);
	
	param("world_3d_map/retain_above_ground_threshold", m_retainAboveGroundThreshold, 0.01);
	param("world_3d_map/verbosity_level", m_verbose, 1);

	m_active = true;
	m_shouldPublish = false;
	m_acceptScans = false;
	random_utils::init(&m_rng);

        /// @todo Find a way to make this work for pr2, with mechanism control,
        /// but not break for STAIR.  Somebody needs to be periodically
        /// publishing the base->base_laser Tx.  Or else we need a more standard
        /// way of retrieving such Txs;
	/* Set up the transform client */
	double laser_x_offset;
	param("laser_x_offset", laser_x_offset, 0.05);
	m_tf.setWithEulers("base_laser", "base", laser_x_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

	/* create a thread that handles the publishing of the data */	
	m_publishingThread = ros::thread::member_thread::startMemberFunctionThread<World3DMap>(this, &World3DMap::publishDataThread);

	subscribe("scan",  m_inputScan,  &World3DMap::scanCallback, 1);
	subscribe("cloud", m_inputCloud, &World3DMap::pointCloudCallback, 1);
	subscribe("base_scan", m_baseScanMsg, &World3DMap::baseScanCallback, 1);
    }
    
    ~World3DMap(void)
    {
	/* terminate spawned threads */
	m_active = false;
	
	pthread_join(*m_publishingThread, NULL);
	for (unsigned int i = 0 ; i < m_currentWorld.size() ; ++i)
	    delete m_currentWorld[i];

	for (unsigned int i = 0 ; i < m_selfSeeParts.size() ; ++i)
	    delete m_selfSeeParts[i].body;
    }
    
    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	planning_node_util::NodeRobotModel::setRobotDescription(file);
	addSelfSeeBodies();
    }
    
    void setAcceptScans(bool acceptScans)
    {
	m_acceptScans = acceptScans;
	if (m_verbose)
	    printf("Accepting input scans...\n");
    }
    
private:
    
    struct RobotPart
    {
	collision_space::bodies::Shape        *body;
	planning_models::KinematicModel::Link *link;	
    };
  
    void stateUpdate(void)
    {
      //m_robotState->print();
	planning_node_util::NodeRobotModel::stateUpdate();
	if (m_kmodel)
	    m_kmodel->computeTransforms(m_robotState->getParams());
	if (m_kmodelSimple)
	    m_kmodelSimple->computeTransforms(m_robotStateSimple->getParams());
    }
    
    void pointCloudCallback(void)
    {
	/* If we're not ready to accept scans yet, discard the data */
	if (!m_acceptScans)
	    return;

	if (m_verbose) 
	    fprintf(stdout, "Received point cloud with %d points in frame %s\n", m_inputCloud.get_pts_size(), m_inputCloud.header.frame_id.c_str());
	
	bool success = false;
	
	if (m_inputCloud.header.frame_id == "map")
	{
	    m_toProcess = m_inputCloud;
	    success = true;
	}
	else
	{
	    try
	    {
		m_tf.transformPointCloud("map", m_toProcess, m_inputCloud);
		success = true;
	    }
	    catch(libTF::TransformReference::LookupException& ex)
	    {
		fprintf(stderr, "Discarding pointcloud: Transform reference lookup exception: %s\n", ex.what());
	    }
	    catch(libTF::TransformReference::ExtrapolateException& ex)
	    {
		fprintf(stderr, "Discarding pointcloud: Extrapolation exception: %s\n", ex.what());
	    }
	    catch(libTF::TransformReference::ConnectivityException& ex)
	    {
		fprintf(stderr, "Discarding pointcloud: Connectivity exception: %s\n", ex.what());
	    }
	    catch(...)
	    {
		fprintf(stderr, "Discarding pointcloud: Exception in point cloud computation.\n");
	    }
	}
	
	if (success)
	    processData();
    }
    
    void scanCallback(void)
    {
	/* If we're not ready to accept scans yet, discard the data */
      if (!m_acceptScans){
	std::cout << "Bailing\n";
	return;
      }
	
	if (m_verbose) 
	    fprintf(stdout, "Received laser scan with %d points in frame %s\n", m_inputScan.get_ranges_size(), m_inputScan.header.frame_id.c_str());
	
	/* copy data to a place where incoming messages do not affect it */
	bool success = false;
	try
	{
	    m_tf.transformLaserScanToPointCloud("map", m_toProcess, m_inputScan);
	    success = true;
	}
	catch(libTF::TransformReference::LookupException& ex)
	{
            fprintf(stderr, "Discarding laser scan: Transform reference lookup exception: %s\n", ex.what());
	}
	catch(libTF::TransformReference::ExtrapolateException& ex)
	{
	    fprintf(stderr, "Discarding laser scan: Extrapolation exception: %s\n", ex.what());
	}
	catch(libTF::TransformReference::ConnectivityException& ex)
	{
	    fprintf(stderr, "Discarding laser scan: Connectivity exception: %s\n", ex.what());
	}
	catch(...)
	{
	    fprintf(stderr, "Discarding laser scan: Exception in point cloud computation\n");
	}  
	if (success)
	    processData();
    }
    
    void baseScanCallback(void)
    {
      if (!m_acceptScans){
	return;
      }
	
      // Assemble a point cloud, in the laser's frame
      std_msgs::PointCloudFloat32 local_cloud;
      const double LASER_MAX_RANGE = 4.0;
      m_projector.projectLaser(m_baseScanMsg, local_cloud, LASER_MAX_RANGE);

	if (m_verbose) 
	    fprintf(stdout, "Received laser scan with %d points in frame %s\n", local_cloud.get_pts_size(), local_cloud.header.frame_id.c_str());
	
	/* copy data to a place where incoming messages do not affect it */
	bool success = false;
	try
	{
	    m_tf.transformPointCloud("map", m_toProcess, local_cloud);
	    success = true;
	}
	catch(libTF::TransformReference::LookupException& ex)
	{
            fprintf(stderr, "Discarding laser scan: Transform reference lookup exception: %s\n", ex.what());
	}
	catch(libTF::TransformReference::ExtrapolateException& ex)
	{
	    fprintf(stderr, "Discarding laser scan: Extrapolation exception: %s\n", ex.what());
	}
	catch(libTF::TransformReference::ConnectivityException& ex)
	{
	    fprintf(stderr, "Discarding laser scan: Connectivity exception: %s\n", ex.what());
	}
	catch(...)
	{
	    fprintf(stderr, "Discarding laser scan: Exception in point cloud computation\n");
	}  
	if (success)
	    processData();
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
	if (ss && ss->hasFlag("collision"))
	{
	    for (unsigned int i = 0 ; i < ss->linkNames.size() ; ++i)
	    {
		planning_models::KinematicModel::Link *link = m_kmodel->getLink(ss->linkNames[i]);
		if (link)
		{
		    RobotPart rp = { NULL, link };    
		    
		    switch (link->shape->type)
		    {
		    case planning_models::KinematicModel::Shape::BOX:
			rp.body = new collision_space::bodies::Box();
			{
			    const double* size = static_cast<planning_models::KinematicModel::Box*>(link->shape)->size;
			    rp.body->setDimensions(size);
			}
			break;
		    case planning_models::KinematicModel::Shape::SPHERE:
			rp.body = new collision_space::bodies::Sphere();
			{
			    double size[1];
			    size[0] = static_cast<planning_models::KinematicModel::Sphere*>(link->shape)->radius;
			    rp.body->setDimensions(size);
			}
			break;
		    case planning_models::KinematicModel::Shape::CYLINDER:
			rp.body = new collision_space::bodies::Cylinder();
			{
			    double size[2];
			    size[0] = static_cast<planning_models::KinematicModel::Cylinder*>(link->shape)->length;
			    size[1] = static_cast<planning_models::KinematicModel::Cylinder*>(link->shape)->radius;
			    rp.body->setDimensions(size);
			}
			break;
		    default:
			break;
		    }
		    
		    if (!rp.body)
		    {
			fprintf(stderr, "Unknown body type: %d\n", link->shape->type);
			continue;
		    }
		    
		    rp.body->setScale(1.75);
		    
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

	m_worldDataMutex.lock();

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
	
	/* let the publishing thread know there is new data to publish */
	m_shouldPublish = true;
	
	if (m_verbose)
	{
	    double window = m_currentWorld.size() > 0 ? (m_currentWorld.back()->header.stamp - m_currentWorld.front()->header.stamp).to_double() : 0.0;
	    printf("World map containing %d point clouds (window size = %f seconds)\n", m_currentWorld.size(), window);
	}

	m_worldDataMutex.unlock();
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
    
    /** Remove invalid floating point values and strip channel
     *  iformation.  Also keep a certain ratio of the cloud information
     *  only. Works with pointclouds in robot or map frames */
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
	
	if (m_verbose)
	    printf("Filter 0 discarded %d points (%d left) \n", n - j, j);

	return copy;	
    }    
    
    /** Remove points from the cloud if the robot sees parts of
	itself. Works for pointclouds in the robot frame. \todo make the
	comment true, separate function in 2.*/
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
	    
	    if (z > m_retainAboveGroundThreshold)
	    {
		bool keep = true;
		for (int i = m_selfSeeParts.size() - 1 ; keep && i >= 0 ; --i)
		    keep = !m_selfSeeParts[i].body->containsPoint(x, y, z);
		
		if (keep)
		    copy->pts[j++] = cloud.pts[k];
	    }
	}
	if (m_verbose)
	    printf("Filter 1 discarded %d points (%d left) \n", n - j, j);
	
	copy->set_pts_size(j);

	return copy;
    }
    
    std::vector<RobotPart>                   m_selfSeeParts;
    std::deque<std_msgs::PointCloudFloat32*> m_currentWorld;// Pointers to saved clouds

    
    double                           m_maxPublishFrequency;
    double                           m_retainPointcloudDuration;
    double                           m_retainPointcloudFraction;    
    double                           m_retainAboveGroundThreshold;
    int                              m_verbose;
    
    std_msgs::LaserScan              m_inputScan;  //Buffer for recieving scan
    std_msgs::PointCloudFloat32      m_inputCloud; //Buffer for recieving cloud
    std_msgs::LaserScan              m_baseScanMsg;  //Buffer for recieving base scan
    std_msgs::PointCloudFloat32      m_toProcess; 
    
    pthread_t                       *m_publishingThread;
    ros::thread::mutex               m_worldDataMutex;
    bool                             m_active, m_shouldPublish, m_acceptScans;
    random_utils::rngState           m_rng;

  laser_scan::LaserProjection m_projector; /**< Used to project laser scans */

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
	World3DMap *map = new World3DMap(argv[1]);
	map->loadRobotDescription();
	map->waitForState();
	map->setAcceptScans(true);
	map->spin();
	map->shutdown();
	delete map;
    }
    else
	usage(argv[0]);
    
    return 0;    
}
