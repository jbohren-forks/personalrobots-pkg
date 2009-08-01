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
    envMonitorStarted_ = false;
    onBeforeMapUpdate_ = NULL;
    onAfterMapUpdate_  = NULL;
    onObjectInMapUpdate_ = NULL;
    
    collisionMapNotifier_ = NULL;
    collisionMapUpdateNotifier_ = NULL;
    objectInMapNotifier_ = NULL;
    
    haveMap_ = false;

    collisionSpace_ = cm_->getODECollisionModel().get();
    nh_.param<double>("~pointcloud_padd", pointcloud_padd_, 0.01);
    
    startEnvironmentMonitor();
}

void planning_environment::CollisionSpaceMonitor::startEnvironmentMonitor(void)
{
    if (envMonitorStarted_)
	return;
    
    collisionMapNotifier_ = new tf::MessageNotifier<mapping_msgs::CollisionMap>(*tf_, boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1), "collision_map", getFrameId(), 1);
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapNotifier_->getTargetFramesString().c_str());

    collisionMapUpdateNotifier_ = new tf::MessageNotifier<mapping_msgs::CollisionMap>(*tf_, boost::bind(&CollisionSpaceMonitor::collisionMapUpdateCallback, this, _1), "collision_map_update", getFrameId(), 1);
    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateNotifier_->getTargetFramesString().c_str());

    objectInMapNotifier_ = new tf::MessageNotifier<mapping_msgs::ObjectInMap>(*tf_, boost::bind(&CollisionSpaceMonitor::objectInMapCallback, this, _1), "object_in_map", getFrameId(), 1024);
    ROS_DEBUG("Listening to object_in_map using message notifier with target frame %s", collisionMapUpdateNotifier_->getTargetFramesString().c_str());

    envMonitorStarted_ = true;
}

void planning_environment::CollisionSpaceMonitor::stopEnvironmentMonitor(void)
{
    if (!envMonitorStarted_)
	return;
    
    delete collisionMapUpdateNotifier_;
    collisionMapUpdateNotifier_ = NULL;
    
    delete collisionMapNotifier_;
    collisionMapNotifier_ = NULL;
    
    delete objectInMapNotifier_;
    objectInMapNotifier_ = NULL;

    ROS_DEBUG("Environment state is no longer being monitored");

    envMonitorStarted_ = false;
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

void planning_environment::CollisionSpaceMonitor::collisionMapAsSpheres(const mapping_msgs::CollisionMapConstPtr &collisionMap,
									std::vector<shapes::Shape*> &spheres, std::vector<btTransform> &poses)
{
    // we want to make sure the frame the robot model is kept in is the same as the frame of the collisionMap
    bool transform = !frame_id_.empty() && collisionMap->header.frame_id != frame_id_;
    const int n = collisionMap->get_boxes_size();
    
    spheres.resize(n);
    poses.resize(n);
    
    if (transform)
    {
	std::string target = frame_id_;
	bool err = false;
	
#pragma omp parallel for
	for (int i = 0 ; i < n ; ++i)
	{
	    robot_msgs::PointStamped psi;
	    psi.header  = collisionMap->header;
	    psi.point.x = collisionMap->boxes[i].center.x;
	    psi.point.y = collisionMap->boxes[i].center.y;
	    psi.point.z = collisionMap->boxes[i].center.z;
	    
	    robot_msgs::PointStamped pso;
	    try
	    {
		tf_->transformPoint(target, psi, pso);
	    }
	    catch(...)
	    {
		err = true;
		pso = psi;
	    }
	    
	    poses[i].setIdentity();
	    poses[i].setOrigin(btVector3(pso.point.x, pso.point.y, pso.point.z));
	    spheres[i] = new shapes::Sphere(maxCoord(collisionMap->boxes[i].extents) * 0.867 + pointcloud_padd_);
	}
	
	if (err)
	    ROS_ERROR("Some errors encountered in transforming the collision map to frame '%s' from frame '%s'", target.c_str(), collisionMap->header.frame_id.c_str());
    }
    else
    {

#pragma omp parallel for
	for (int i = 0 ; i < n ; ++i)
	{
	    poses[i].setIdentity();
	    poses[i].setOrigin(btVector3(collisionMap->boxes[i].center.x, collisionMap->boxes[i].center.y, collisionMap->boxes[i].center.z));
	    spheres[i] = new shapes::Sphere(maxCoord(collisionMap->boxes[i].extents) * 0.867 + pointcloud_padd_);
	}
    }
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsBoxes(const mapping_msgs::CollisionMapConstPtr &collisionMap,
								      std::vector<shapes::Shape*> &boxes, std::vector<btTransform> &poses)
{
    // we want to make sure the frame the robot model is kept in is the same as the frame of the collisionMap
    bool transform = !frame_id_.empty() && collisionMap->header.frame_id != frame_id_;
    const int n = collisionMap->get_boxes_size();
    
    double pd = 2.0 * pointcloud_padd_;
    
    boxes.resize(n);
    poses.resize(n);
    
    if (transform)
    {
	std::string target = frame_id_;
	bool err = false;
	
#pragma omp parallel for
	for (int i = 0 ; i < n ; ++i)
	{
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
	    
	    poses[i].setRotation(btQuaternion(btVector3(collisionMap->boxes[i].axis.x, collisionMap->boxes[i].axis.y, collisionMap->boxes[i].axis.z), collisionMap->boxes[i].angle));
	    poses[i].setOrigin(btVector3(pso.point.x, pso.point.y, pso.point.z));
	    boxes[i] = new shapes::Box(collisionMap->boxes[i].extents.x + pd, collisionMap->boxes[i].extents.y + pd, collisionMap->boxes[i].extents.z + pd);
	}
	
	if (err)
	    ROS_ERROR("Some errors encountered in transforming the collision map to frame '%s' from frame '%s'", target.c_str(), collisionMap->header.frame_id.c_str());
    }
    else
    {

#pragma omp parallel for
	for (int i = 0 ; i < n ; ++i)
	{
	    poses[i].setRotation(btQuaternion(btVector3(collisionMap->boxes[i].axis.x, collisionMap->boxes[i].axis.y, collisionMap->boxes[i].axis.z), collisionMap->boxes[i].angle));
	    poses[i].setOrigin(btVector3(collisionMap->boxes[i].center.x, collisionMap->boxes[i].center.y, collisionMap->boxes[i].center.z));
	    boxes[i] = new shapes::Box(collisionMap->boxes[i].extents.x + pd, collisionMap->boxes[i].extents.y + pd, collisionMap->boxes[i].extents.z + pd);
	}
    }
}

void planning_environment::CollisionSpaceMonitor::updateCollisionSpace(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear)
{
    boost::mutex::scoped_lock lock(mapUpdateLock_);
    
    ROS_DEBUG("Received collision map with %d points that is %f seconds old", collisionMap->get_boxes_size(), (ros::Time::now() - collisionMap->header.stamp).toSec());
    
    if (onBeforeMapUpdate_ != NULL)
	onBeforeMapUpdate_(collisionMap, clear);

    ros::WallTime startTime = ros::WallTime::now();
    
    std::vector<shapes::Shape*> spheres;
    std::vector<btTransform>    poses;
    collisionMapAsBoxes(collisionMap, spheres, poses);

    collisionSpace_->lock();
    if (clear)
	collisionSpace_->clearObjects("points");
    collisionSpace_->addObjects("points", spheres, poses);
    collisionSpace_->unlock();
    
    double tupd = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Updated map model in %f seconds", tupd);
    
    if (clear)
    {
	lastMapUpdate_ = collisionMap->header.stamp;
	haveMap_ = true;
    }
    
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
		collisionSpace_->clearObjects(objectInMap->id);
		collisionSpace_->addObject(objectInMap->id, shape, pose);
		collisionSpace_->unlock();
		ROS_INFO("Added object '%s' to collision space", objectInMap->id.c_str());
	    }
	}
    }
    else
    {
	// remove the object from the map
	collisionSpace_->lock();
	collisionSpace_->clearObjects(objectInMap->id);
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

void planning_environment::CollisionSpaceMonitor::recoverCollisionMap(const collision_space::EnvironmentModel *env, mapping_msgs::CollisionMap &cmap)
{
    cmap.header.frame_id = getFrameId();
    cmap.header.stamp = ros::Time::now();
    cmap.boxes.clear();
    
    const collision_space::EnvironmentObjects::NamespaceObjects &no = env->getObjects()->getObjects("points");
    const unsigned int n = no.shape.size();
    double pd = pointcloud_padd_ * 2.0;    
    for (unsigned int i = 0 ; i < n ; ++i)
	if (no.shape[i]->type == shapes::BOX)
	{
	    const shapes::Box* box = static_cast<const shapes::Box*>(no.shape[i]);
	    mapping_msgs::OrientedBoundingBox obb;
	    obb.extents.x = box->size[0] - pd;
	    obb.extents.y = box->size[1] - pd;
	    obb.extents.z = box->size[2] - pd;
	    const btVector3 &c = no.shapePose[i].getOrigin();
	    obb.center.x = c.x();
	    obb.center.y = c.y();
	    obb.center.z = c.z();
	    const btQuaternion q = no.shapePose[i].getRotation();
	    obb.angle = q.getAngle();
	    const btVector3 axis = q.getAxis();
	    obb.axis.x = axis.x();
	    obb.axis.y = axis.y();
	    obb.axis.z = axis.z();
	    cmap.boxes.push_back(obb);
	}
}
