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
   - @b cloud/PointCloud : cloud data 

   Additional subscriptions due to inheritance from NodeRobotModel:
   - @b localizedpose/RobotBase2DOdom : localized position of the robot base

   Publishes to (name/type):
   - @b "world_3d_map"/PointCloud : point cloud describing the 3D environment

   <hr>

   Uses (name/type):
   - None

   Provides (name/type):
   - None

   <hr>

   @section parameters ROS parameters
   - @b "world_3d_map/max_publish_frequency" : @b [double] the maximum frequency (Hz) at which the data in the built 3D map is to be sent (default 10)
   - @b "world_3d_map/base_laser_range : @b [double] the max range setting for base laser projection (default 10)
   - @b "world_3d_map/tilt_laser_range : @b [double] the max range setting for tilt laser projection (default 4)
   - @b "world_3d_map/retain_pointcloud_fraction : @b [double] the time for which a point cloud is retained as part of the current world information (default 2)
   - @b "world_3d_map/retain_above_ground_threshold : @b [double] the vertical distance from the ground for which points are considered in the ground plane (default .03)
   - @b "world_3d_map/verbosity_level" : @b [int] sets the verbosity level (default 1)
**/

#include <ros/node.h>
#include <tf/transform_listener.h>
#include <rosthread/member_thread.h>
#include <rosthread/mutex.h>

#include <std_msgs/PointCloud.h>
#include <std_msgs/LaserScan.h>

#include <random_utils/random_utils.h>
#include <robot_filter/RobotFilter.h>
#include <tf/message_notifier.h>

// Laser projection
#include "laser_scan/laser_scan.h"

#include <deque>
#include <cmath>

class World3DMap : public ros::node
{
public:
    
  World3DMap(const std::string &robot_model_name) :
    ros::node("world_3d_map"), m_tf(*this, true, 1000000000ULL)
  {
    m_tf.setExtrapolationLimit(ros::Duration().fromSec(10));

    advertise<std_msgs::PointCloud>("world_3d_map", 1);

    param("world_3d_map/max_publish_frequency", m_maxPublishFrequency, 10.0);
    param("world_3d_map/base_laser_range", m_baseLaserMaxRange, 10.0);
    param("world_3d_map/tilt_laser_range", m_tiltLaserMaxRange, 4.0);
    param("world_3d_map/retain_pointcloud_fraction", m_retainPointcloudFraction, 0.25);
    param("world_3d_map/retain_above_ground_threshold", m_retainAboveGroundThreshold, 0.03);
    param("world_3d_map/verbosity_level", m_verbose, 1);

    double bodyPartScale;
    param("world_3d_map/body_part_scale", bodyPartScale, 1.95);

    m_active = true;
    m_acceptScans = false;
    random_utils::init(&m_rng);

    /* create a thread that handles the publishing of the data */	
    m_publishingThread = ros::thread::member_thread::startMemberFunctionThread<World3DMap>(this, &World3DMap::publishDataThread);

    m_scanNotifier = new tf::MessageNotifier<std_msgs::LaserScan>(&m_tf, this, 
				 boost::bind(&World3DMap::scanCallback, this, _1),
				 "scan", "map", 50);
    m_baseScanNotifier = new tf::MessageNotifier<std_msgs::LaserScan>(&m_tf, this, 
				 boost::bind(&World3DMap::baseScanCallback, this, _1),
				 "cloud", "map", 50);
    m_cloudNotifier = new tf::MessageNotifier<std_msgs::PointCloud>(&m_tf, this, 
				 boost::bind(&World3DMap::pointCloudCallback, this, _1),
				 "base_scan", "map", 50);

    m_robotFilter = new robot_filter::RobotFilter(this, robot_model_name, m_verbose, bodyPartScale);
  }
    
  ~World3DMap(void)
  {
    delete m_robotFilter;
    delete m_scanNotifier;
    delete m_baseScanNotifier;
    delete m_cloudNotifier;

    /* terminate spawned threads */
    m_active = false;
	
    pthread_join(*m_publishingThread, NULL);
    for (unsigned int i = 0 ; i < m_currentWorld.size() ; ++i)
      delete m_currentWorld[i];

  }
  void setAcceptScans(bool acceptScans)
  {
    m_acceptScans = acceptScans;
  }


  void loadRobotDescription() {
    m_robotFilter->loadRobotDescription();
  }

  void waitForState() {
    m_robotFilter->waitForState();
  }
    
    
private:
  
  void pointCloudCallback(const tf::MessageNotifier<std_msgs::PointCloud>::MessagePtr& message)
  {
    processData(*message);
  }
    
  void scanCallback(const tf::MessageNotifier<std_msgs::LaserScan>::MessagePtr& message)
  {
    // Project laser into point cloud
    std_msgs::PointCloud local_cloud;
    m_projector.projectLaser(*message, local_cloud, m_tiltLaserMaxRange);
    processData(local_cloud);
  }
    
  void baseScanCallback(const tf::MessageNotifier<std_msgs::LaserScan>::MessagePtr& message)
  {
    // Project laser into point cloud
    std_msgs::PointCloud local_cloud;
    m_projector.projectLaser(*message, local_cloud, m_baseLaserMaxRange);
    processData(local_cloud);
  }


  void publishDataThread(void)
  {
    ros::Duration *d = new ros::Duration();
    d->fromSec(1.0/m_maxPublishFrequency);
    
    /* while everything else is running (map building) check if
       there are any updates to send, but do so at most at the
       maximally allowed frequency of sending data */
    while (m_active)
    {
      d->sleep();
	
      m_worldDataMutex.lock();
      if (m_active && m_currentWorld.size() > 0)
      {
	std_msgs::PointCloud toPublish;
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
	    ROS_INFO("Publishing a point cloud with %u points\n", toPublish.get_pts_size());
	  publish("world_3d_map", toPublish);
	}
      }
      m_worldDataMutex.unlock();
    }
    delete d;
  }
    
  void processData(const std_msgs::PointCloud& local_cloud)
  {
    if (!m_acceptScans){
      ROS_INFO("Rejecting scans\n");
      return;
    }

    m_worldDataMutex.lock();

    point_clouds_.push_back(local_cloud);

    while(!point_clouds_.empty()){

      const std_msgs::PointCloud& point_cloud = point_clouds_.front();


      std_msgs::PointCloud map_cloud;
	
      /* Transform to the map frame */
      try
	{
	  m_tf.transformPointCloud("map", point_cloud, map_cloud);
	}
      catch(tf::LookupException& ex)
	{
	  ROS_ERROR("Lookup exception: %s\n", ex.what());
	  break;
	}
      catch(tf::ExtrapolationException& ex)
	{
	  ROS_ERROR("Extrapolation exception: %s\n", ex.what());
	  break;
	}
      catch(tf::ConnectivityException& ex)
	{
	  ROS_ERROR("Connectivity exception: %s\n", ex.what());
	  break;
	}
      catch(std::runtime_error &ex)
	{
	  ROS_ERROR("Exception in point cloud computation \n %s", ex.what());
	  break;
	}

      point_clouds_.pop_front();

      /* add new data */

      ROS_DEBUG( "Received laser scan with %d points in frame %s\n", 
		local_cloud.get_pts_size(), local_cloud.header.frame_id.c_str());

      std_msgs::PointCloud *newData = runFilters(map_cloud);

      if (newData){
	if (newData->get_pts_size() == 0)
	  delete newData;
	else{
	  m_currentWorld.push_back(newData);
	}
      }
    }

    m_worldDataMutex.unlock();
  }
    
  std_msgs::PointCloud* runFilters(const std_msgs::PointCloud &cloud)
  {
    std_msgs::PointCloud *cloudF = filter0(cloud, m_retainPointcloudFraction);
    
    if (cloudF)
      {
	std_msgs::PointCloud *temp = filter1(*cloudF);
	delete cloudF;
	cloudF = temp;
      }

    if (cloudF)
      {
	std_msgs::PointCloud *temp = m_robotFilter->filter(*cloudF);
	delete cloudF;
	cloudF = temp;
      }
    
    return cloudF;
  }

  /** Remove invalid floating point values and strip channel
   *  iformation.  Also keep a certain ratio of the cloud information
   *  only. Works with pointclouds in robot or map frames */
  std_msgs::PointCloud* filter0(const std_msgs::PointCloud &cloud, double frac = 1.0)
  {
    std_msgs::PointCloud *copy = new std_msgs::PointCloud();
    copy->header = cloud.header;

    unsigned int n = cloud.get_pts_size();
    unsigned int j = 0;
    copy->set_pts_size(n);	
    for (unsigned int k = 0 ; k < n ; ++k)
      if (random_utils::uniform(&m_rng, 0.0, 1.0) < frac)
	if (std::isfinite(cloud.pts[k].x) && std::isfinite(cloud.pts[k].y) && std::isfinite(cloud.pts[k].z))
	  copy->pts[j++] = cloud.pts[k];
    copy->set_pts_size(j);
	
    ROS_INFO("Filter 0 discarded %d points (%d left) \n", n - j, j);

    return copy;	
  } 



  /** Remove points in the ground plane. Note that
   in the future this could use Sachin's ransac
   ground plane extract. */
  std_msgs::PointCloud* filter1(const std_msgs::PointCloud &cloud)
  {

    if (cloud.header.frame_id != "map") {
      ROS_ERROR("Cloud not in the robot frame in filter1. It is in the %s frame.", 
		cloud.header.frame_id.c_str());
    }

    std_msgs::PointCloud *copy = new std_msgs::PointCloud();
    copy->header = cloud.header;

    unsigned int n = cloud.get_pts_size();
    unsigned int j = 0;
    copy->set_pts_size(n);	
    for (unsigned int k = 0 ; k < n ; ++k)
      if (cloud.pts[k].z > m_retainAboveGroundThreshold)
	copy->pts[j++] = cloud.pts[k];
    copy->set_pts_size(j);
	
    ROS_INFO("Filter 1 discarded %d points (%d left) \n", n - j, j);

    return copy;	
  }       

    
  std::vector<std_msgs::PointCloud*> m_currentWorld;// Pointers to saved clouds

  double                           m_maxPublishFrequency;
  double                           m_baseLaserMaxRange;
  double                           m_tiltLaserMaxRange;
  double                           m_retainPointcloudFraction;    
  double                           m_retainAboveGroundThreshold;
  int                              m_verbose;
    
  robot_filter::RobotFilter       *m_robotFilter;
  tf::TransformListener            m_tf; 
  std_msgs::LaserScan              m_inputScan;  //Buffer for recieving scan
  std_msgs::PointCloud             m_inputCloud; //Buffer for recieving cloud
  std_msgs::LaserScan              m_baseScanMsg;  //Buffer for recieving base scan    
  pthread_t                       *m_publishingThread;
  ros::thread::mutex               m_worldDataMutex;
  bool                             m_active, m_acceptScans;
  random_utils::rngState           m_rng;

  laser_scan::LaserProjection m_projector; /**< Used to project laser scans */
  std::deque<std_msgs::PointCloud> point_clouds_;

  tf::MessageNotifier<std_msgs::LaserScan>* m_scanNotifier;
  tf::MessageNotifier<std_msgs::LaserScan>* m_baseScanNotifier;
  tf::MessageNotifier<std_msgs::PointCloud>* m_cloudNotifier;
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
      ROS_DEBUG("Waiting until I have mechanism state and a localized pose message");
      map->waitForState();
      ROS_DEBUG("Got state and a localized pose message");
      map->setAcceptScans(true);
      map->spin();
      map->shutdown();
      delete map;
    }
  else
    usage(argv[0]);
    
  return 0;    
}
