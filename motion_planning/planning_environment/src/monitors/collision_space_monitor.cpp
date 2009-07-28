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

#include "planning_environment/monitors/collision_space_monitor.h"
#include "planning_environment/util/construct_object.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/bind.hpp>
#include <sstream>
#include <climits>

namespace planning_environment
{
    
    static inline double maxCoord(const geometry_msgs::Point32 &point)
    {
	return std::max(std::max(point.x, point.y), point.z);
    }
}

void planning_environment::CollisionSpaceMonitor::setupCSM(void)
{
    onBeforeMapUpdate_ = NULL;
    onAfterMapUpdate_  = NULL;
    onObjectInMapUpdate_ = NULL;
    
    collisionMapNotifier_ = NULL;
    collisionMapUpdateNotifier_ = NULL;
    objectInMapNotifier_ = NULL;
    
    haveMap_ = false;

    collisionSpace_ = cm_->getODECollisionModel().get();
    
    // a list of static planes bounding the environment
    std::string planes;
    nh_.param<std::string>("~bounding_planes", planes, std::string());
    nh_.param<double>("~pointcloud_padd", pointcloud_padd_, 0.01);

    std::stringstream ss(planes);
    std::vector<double> planeValues;
    if (!planes.empty())
	while (ss.good() && !ss.eof())
	{
	    double value;
	    ss >> value;
	    planeValues.push_back(value);
	}
    if (planeValues.size() % 4 != 0)
	ROS_WARN("~bounding_planes must be a list of 4-tuples (a b c d) that define planes ax+by+cz+d=0");
    else
	for (unsigned int i = 0 ; i < planeValues.size() / 4 ; ++i)
	{
	    collisionSpace_->addPlane("bounds", planeValues[i * 4], planeValues[i * 4 + 1], planeValues[i * 4 + 2], planeValues[i * 4 + 3]);
	    ROS_INFO("Added static plane %fx + %fy + %fz + %f = 0", planeValues[i * 4], planeValues[i * 4 + 1], planeValues[i * 4 + 2], planeValues[i * 4 + 3]);
	}
    
    collisionMapNotifier_ = new tf::MessageNotifier<mapping_msgs::CollisionMap>(*tf_, boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1), "collision_map", getFrameId(), 1);
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapNotifier_->getTargetFramesString().c_str());

    collisionMapUpdateNotifier_ = new tf::MessageNotifier<mapping_msgs::CollisionMap>(*tf_, boost::bind(&CollisionSpaceMonitor::collisionMapUpdateCallback, this, _1), "collision_map_update", getFrameId(), 1);
    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateNotifier_->getTargetFramesString().c_str());

    objectInMapNotifier_ = new tf::MessageNotifier<mapping_msgs::ObjectInMap>(*tf_, boost::bind(&CollisionSpaceMonitor::objectInMapCallback, this, _1), "object_in_map", getFrameId(), 1024);
    ROS_DEBUG("Listening to object_in_map using message notifier with target frame %s", collisionMapUpdateNotifier_->getTargetFramesString().c_str());
}

bool planning_environment::CollisionSpaceMonitor::isMapUpdated(double sec) const
{
    if (!haveMap_)
	return false;
    
    // less than 10us is considered 0 
    if (sec > 1e-5 && lastMapUpdate_ < ros::Time::now() - ros::Duration(sec))
	return false;
    else
	return true;
}

void planning_environment::CollisionSpaceMonitor::waitForMap(void) const
{
    int s = 0;
    while (nh_.ok() && !haveMap())
    {
	if (s == 0)
	    ROS_INFO("Waiting for map ...");
	s = (s + 1) % 40;
	ros::spinOnce();
	ros::Duration().fromSec(0.05).sleep();
    }
    if (haveMap())
	ROS_INFO("Map received!");
}

void planning_environment::CollisionSpaceMonitor::collisionMapUpdateCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap)
{
    if (collisionMap->boxes.size() > 0)
       updateCollisionSpace(collisionMap, false);
}

void planning_environment::CollisionSpaceMonitor::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap)
{
    updateCollisionSpace(collisionMap, true);
}

void planning_environment::CollisionSpaceMonitor::updateCollisionSpace(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear)
{
    boost::mutex::scoped_lock lock(mapUpdateLock_);
    
    int n = collisionMap->get_boxes_size();
    
    ROS_DEBUG("Received collision map with %d points that is %f seconds old", n, (ros::Time::now() - collisionMap->header.stamp).toSec());
    
    if (onBeforeMapUpdate_ != NULL)
	onBeforeMapUpdate_(collisionMap, clear);

    // we want to make sure the frame the robot model is kept in is the same as the frame of the collisionMap
    bool transform = !frame_id_.empty() && collisionMap->header.frame_id != frame_id_;

    ros::WallTime startTime = ros::WallTime::now();
    double *data = n > 0 ? new double[4 * n] : NULL;	

    if (transform)
    {
	std::string target = frame_id_;
	bool err = false;
	
#pragma omp parallel for
	for (int i = 0 ; i < n ; ++i)
	{
	    int i4 = i * 4;
	    geometry_msgs::PointStamped psi;
	    psi.header  = collisionMap->header;
	    psi.point.x = collisionMap->boxes[i].center.x;
	    psi.point.y = collisionMap->boxes[i].center.y;
	    psi.point.z = collisionMap->boxes[i].center.z;
	    
	    geometry_msgs::PointStamped pso;
	    try
	    {
		tf_->transformPoint(target, psi, pso);
	    }
	    catch(...)
	    {
		err = true;
		pso = psi;
	    }
	    
	    data[i4    ] = pso.point.x;
	    data[i4 + 1] = pso.point.y;
	    data[i4 + 2] = pso.point.z;
	    
	    data[i4 + 3] = maxCoord(collisionMap->boxes[i].extents) * 0.867 + pointcloud_padd_;
	}
	
	if (err)
	    ROS_ERROR("Some errors encountered in transforming the collision map to frame '%s' from frame '%s'", target.c_str(), collisionMap->header.frame_id.c_str());
    }
    else
    {

#pragma omp parallel for
	for (int i = 0 ; i < n ; ++i)
	{
	    int i4 = i * 4;
	    data[i4    ] = collisionMap->boxes[i].center.x;
	    data[i4 + 1] = collisionMap->boxes[i].center.y;
	    data[i4 + 2] = collisionMap->boxes[i].center.z;
	    
	    data[i4 + 3] = maxCoord(collisionMap->boxes[i].extents) * 0.867 + pointcloud_padd_;
	}
    }
    
    collisionSpace_->lock();

    if (clear)
	collisionSpace_->clearObstacles("points");
    if (n > 0)
        collisionSpace_->addPointCloudSpheres("points", n, data);

    collisionSpace_->unlock();
    
    if (data)
       delete[] data;
    
    double tupd = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Updated map model in %f seconds", tupd);
    lastMapUpdate_ = collisionMap->header.stamp;
    haveMap_ = true;
    
    if (onAfterMapUpdate_ != NULL)
	onAfterMapUpdate_(collisionMap, clear);
}

void planning_environment::CollisionSpaceMonitor::objectInMapCallback(const mapping_msgs::ObjectInMapConstPtr &objectInMap)
{
    if (objectInMap->action == mapping_msgs::ObjectInMap::ADD)
    {
	// add the object to the map
	shapes::Shape *shape = construct_object(objectInMap->object);
	if (shape)
	{
	    bool err = false;
	    geometry_msgs::PoseStamped psi;
	    geometry_msgs::PoseStamped pso;
	    psi.pose = objectInMap->pose;
	    psi.header = objectInMap->header;
	    try
	    {
		tf_->transformPose(getFrameId(), psi, pso);
	    }
	    catch(...)
	    {
		err = true;
	    }
	    
	    if (err)
		ROS_ERROR("Unable to transform object '%s' in frame '%s' to frame '%s'", objectInMap->id.c_str(), objectInMap->header.frame_id.c_str(), getFrameId().c_str());
	    else
	    {
		btTransform pose;
		tf::poseMsgToTF(pso.pose, pose);
		collisionSpace_->lock();
		collisionSpace_->clearObstacles(objectInMap->id);
		collisionSpace_->addObject(objectInMap->id, shape, pose);
		collisionSpace_->unlock();
		ROS_INFO("Added object '%s' to collision space", objectInMap->id.c_str());
	    }
	    delete shape;
	}
    }
    else
    {
	// remove the object from the map
	collisionSpace_->lock();
	collisionSpace_->clearObstacles(objectInMap->id);
	collisionSpace_->unlock();
	ROS_INFO("Removed object '%s' from collision space", objectInMap->id.c_str());
    }
    
    if (onObjectInMapUpdate_)
	onObjectInMapUpdate_(objectInMap);
}

bool planning_environment::CollisionSpaceMonitor::attachObject(const mapping_msgs::AttachedObjectConstPtr &attachedObject)
{
    collisionSpace_->lock();
    // call the same code as in the kinematic model state monitor, but disable the callback
    boost::function<void(planning_models::KinematicModel::Link*, const mapping_msgs::AttachedObjectConstPtr &attachedObject)> backup = onAfterAttachBody_;
    onAfterAttachBody_ = NULL;
    bool result = KinematicModelStateMonitor::attachObject(attachedObject);

    // restore the callback
    onAfterAttachBody_ = backup;
    if (result)
    {
	kmodel_->lock();	    
	collisionSpace_->updateAttachedBodies();
	kmodel_->unlock();	    
	ROS_INFO("Attached bodies have been updated");
    }
    collisionSpace_->unlock();
    
    // call the event, if needed
    if (result && (onAfterAttachBody_ != NULL))
	onAfterAttachBody_(kmodel_->getLink(attachedObject->link_name), attachedObject); 
    
    return result;
}
