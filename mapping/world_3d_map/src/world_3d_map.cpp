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
#include <rostools/Log.h>
#include <deque>
#include <cmath>
using namespace std_msgs;
using namespace ros::thread::member_thread;

class World3DMap : public ros::node
{
public:

    World3DMap(void) : ros::node("world_3d_map")
    {
	advertise<PointCloudFloat32>("world_3d_map");
	advertise<rostools::Log>("roserr");

	// NOTE: subscribe to stereo vision point cloud as well... when it becomes available
	subscribe("full_cloud", inputCloud, &World3DMap::pointCloudCallback);
	
	param("world_3d_map/max_publish_frequency", maxPublishFrequency, 0.5);
	param("world_3d_map/retain_pointcloud_duration", retainPointcloudDuration, 2.0);
	param("world_3d_map/verbosity_level", verbose, 1);
	
	/* create a thread that does the processing of the input data.
	 * and one that handles the publishing of the data */
	active = true;
	working = false;
	shouldPublish = false;
	
	processMutex.lock();
	publishMutex.lock();
	processingThread = startMemberFunctionThread<World3DMap>(this, &World3DMap::processDataThread);
	publishingThread = startMemberFunctionThread<World3DMap>(this, &World3DMap::publishDataThread);
    }
    
    ~World3DMap(void)
    {
	/* terminate spawned threads */
	active = false;
	processMutex.unlock();
	
	pthread_join(*publishingThread, NULL);
	pthread_join(*processingThread, NULL);
	for (unsigned int i = 0 ; i < currentWorld.size() ; ++i)
	    delete currentWorld[i];
    }
    
    void pointCloudCallback(void)
    {
	/* The idea is that if processing of previous input data is
	   not done, data will be discarded. Hopefully this discarding
	   of data will not happen, but we don't want the node to
	   postpone processing latest data just because it is not done
	   with older data. */
	
	flagMutex.lock();
	bool discard = working;
	if (!discard)
	    working = true;
	
	if (discard)
	{
	    /* log the fact that input was discarded */
	    rostools::Log l;
	    l.level = 20;
	    l.name  = get_name();
	    l.msg   = "Discarded point cloud data (previous input set not done processing)";
	    publish("roserr", l);
	}
	else
	{
	    toProcess = inputCloud;  /* copy data to a place where incoming messages do not affect it */
	    processMutex.unlock();   /* let the processing thread know that there is data to process */
	}
	flagMutex.unlock();
    }
    
    void processDataThread(void)
    {
	while (active)
	{
	    /* This mutex acts as a condition, but is safer (condition
	       messages may get lost or interrupted by signals) */
	    processMutex.lock();
	    if (active)
	    {
		worldDataMutex.lock();
		processData();
		worldDataMutex.unlock();
		
		/* notify the publishing thread that it can send data */
		shouldPublish = true;
	    }
	    
	    /* make a note that there is no active processing */
	    flagMutex.lock();
	    working = false;
	    flagMutex.unlock();
	}
    }
    

    void publishDataThread(void)
    {
	ros::Duration *d = new ros::Duration(1.0/maxPublishFrequency);
	
	/* while everything else is running (map building) check if
	   there are any updates to send, but do so at most at the
	   maximally allowed frequency of sending data */
	while (active)
	{
	    d->sleep();

	    if (shouldPublish)
	    {
		worldDataMutex.lock();
		if (active)
		{
		    PointCloudFloat32 toPublish;
		    toPublish.header = currentWorld.back()->header;
		    
		    unsigned int      npts  = 0;
		    for (unsigned int i = 0 ; i < currentWorld.size() ; ++i)
			npts += currentWorld[i]->get_pts_size();
		    
		    toPublish.set_pts_size(npts);
		    
		    unsigned int j = 0;
		    for (unsigned int i = 0 ; i < currentWorld.size() ; ++i)
		    {
			unsigned int n = currentWorld[i]->get_pts_size();			
			for (unsigned int k = 0 ; k < n ; ++k)
			    toPublish.pts[j++] =  currentWorld[i]->pts[k];
		    }
		    
		    toPublish.set_pts_size(j);
		    if (ok())
		    {
			if (verbose > 0)
			    printf("Publishing a point cloud with %u points\n", toPublish.get_pts_size());
			publish("world_3d_map", toPublish);
		    }
		}
		shouldPublish = false;
		worldDataMutex.unlock();
	    }
	}
	delete d;
    }
    
    /* remove invalid floating point values and strip channel iformation */
    PointCloudFloat32* filterInvalidValues(const PointCloudFloat32 &cloud)
    {
	PointCloudFloat32 *copy = new PointCloudFloat32();
	copy->header = cloud.header;

	unsigned int n = cloud.get_pts_size();
	unsigned int j = 0;
	copy->set_pts_size(n);	
	for (unsigned int k = 0 ; k < n ; ++k)
	    if (isfinite(cloud.pts[k].x) && isfinite(cloud.pts[k].y) && isfinite(cloud.pts[k].z))
		copy->pts[j++] = cloud.pts[k];
	copy->set_pts_size(j);

	return copy;	
    }

    PointCloudFloat32* runFilters(const PointCloudFloat32 &cloud)
    {
	return filterInvalidValues(cloud);
    }
    
    void processData(void)
    {
	/* remove old data */
	ros::Time &time = toProcess.header.stamp;
	while (!currentWorld.empty() && (time - currentWorld.front()->header.stamp).to_double() < retainPointcloudDuration)
	{
	    PointCloudFloat32* old = currentWorld.front();
	    currentWorld.pop_front();
	    delete old;
	}
	/* add new data */
	currentWorld.push_back(runFilters(toProcess));
	if (verbose > 0)
	    printf("World map containing %d point clouds\n", currentWorld.size());	
    }
    
private:
    
    PointCloudFloat32              inputCloud;
    PointCloudFloat32              toProcess;
    std::deque<PointCloudFloat32*> currentWorld;
    
    double             maxPublishFrequency;
    double             retainPointcloudDuration;
    int                verbose;
    
    pthread_t         *processingThread;
    pthread_t         *publishingThread;
    ros::thread::mutex processMutex, publishMutex, worldDataMutex, flagMutex;
    bool               active, working, shouldPublish;
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);

    World3DMap map;
    map.spin();
    map.shutdown();
    
    return 0;    
}
