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

#include "planning_environment/collision_space_monitor.h"
#include <robot_msgs/PointStamped.h>
#include <boost/bind.hpp>
#include <sstream>
#include <climits>

namespace planning_environment
{
    
    static inline double maxCoord(const robot_msgs::Point32 &point)
    {
	return std::max(std::max(point.x, point.y), point.z);
    }
}

void planning_environment::CollisionSpaceMonitor::setupCSM(void)
{
    onBeforeMapUpdate_ = NULL;
    onAfterMapUpdate_  = NULL;
    onAfterAttachBody_ = NULL;
    
    attachedBodyNotifier_   = NULL;
    collisionMapNotifier_ = NULL;
    
    haveMap_ = false;

    if (!tf_)
	tf_ = new tf::TransformListener();
    
    collisionSpace_ = cm_->getODECollisionModel().get();

    // the factor by which a boxes maximum extent is multiplied to get the radius of the sphere to be placed in the collision space
    nh_.param<double>("~box_scale", boxScale_, 2.0);
    
    // a list of static planes bounding the environment
    std::string planes;
    nh_.param<std::string>("~bounding_planes", planes, std::string());
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
	    collisionSpace_->addStaticPlane(planeValues[i * 4], planeValues[i * 4 + 1], planeValues[i * 4 + 2], planeValues[i * 4 + 3]);
	    ROS_INFO("Added static plane %fx + %fy + %fz + %f = 0", planeValues[i * 4], planeValues[i * 4 + 1], planeValues[i * 4 + 2], planeValues[i * 4 + 3]);
	}
    
    collisionMapNotifier_ = new tf::MessageNotifier<robot_msgs::CollisionMap>(*tf_, boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1), "collision_map", getFrameId(), 1);
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapNotifier_->getTargetFramesString().c_str());

    collisionMapUpdateNotifier_ = new tf::MessageNotifier<robot_msgs::CollisionMap>(*tf_, boost::bind(&CollisionSpaceMonitor::collisionMapUpdateCallback, this, _1), "collision_map_update", getFrameId(), 1);
    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateNotifier_->getTargetFramesString().c_str());

    if (cm_->loadedModels())
    {
	attachedBodyNotifier_ = new tf::MessageNotifier<robot_msgs::AttachedObject>(*tf_, boost::bind(&CollisionSpaceMonitor::attachObjectCallback, this, _1), "attach_object", "", 1);
	attachedBodyNotifier_->setTargetFrame(cm_->getCollisionCheckLinks());
	ROS_DEBUG("Listening to attach_object using message notifier with target frame %s", attachedBodyNotifier_->getTargetFramesString().c_str());
    }
}

bool planning_environment::CollisionSpaceMonitor::isMapUpdated(double sec) const
{
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

void planning_environment::CollisionSpaceMonitor::collisionMapUpdateCallback(const robot_msgs::CollisionMapConstPtr &collisionMap)
{
    if (collisionMap->boxes.size() > 0)
       updateCollisionSpace(collisionMap, false);
}

void planning_environment::CollisionSpaceMonitor::collisionMapCallback(const robot_msgs::CollisionMapConstPtr &collisionMap)
{
    updateCollisionSpace(collisionMap, true);
}

void planning_environment::CollisionSpaceMonitor::updateCollisionSpace(const robot_msgs::CollisionMapConstPtr &collisionMap, bool clear)
{
    boost::mutex::scoped_lock lock(mapUpdateLock_);
    
    int n = collisionMap->get_boxes_size();
    
    ROS_DEBUG("Received %d points (collision map)", n);
    
    if (onBeforeMapUpdate_ != NULL)
	onBeforeMapUpdate_(collisionMap);

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
	    
	    data[i4    ] = pso.point.x;
	    data[i4 + 1] = pso.point.y;
	    data[i4 + 2] = pso.point.z;
	    
	    data[i4 + 3] = maxCoord(collisionMap->boxes[i].extents) * boxScale_;
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
	    
	    data[i4 + 3] = maxCoord(collisionMap->boxes[i].extents) * boxScale_;
	}
    }
    
    collisionSpace_->lock();

    if (clear)
	collisionSpace_->clearObstacles();
    if (n > 0)
        collisionSpace_->addPointCloud(n, data);

    collisionSpace_->unlock();
    
    if (data)
       delete[] data;
    
    double tupd = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Updated map model in %f seconds", tupd);
    lastMapUpdate_ = collisionMap->header.stamp;
    haveMap_ = true;
    
    if (onAfterMapUpdate_ != NULL)
	onAfterMapUpdate_(collisionMap);
}

void planning_environment::CollisionSpaceMonitor::attachObjectCallback(const robot_msgs::AttachedObjectConstPtr &attachedObject)
{
    collisionSpace_->lock();
    planning_models::KinematicModel::Link *link = kmodel_->getLink(attachedObject->link_name);
    
    if (link)
    {	
	// clear the previously attached bodies 
	for (unsigned int i = 0 ; i < link->attachedBodies.size() ; ++i)
	    delete link->attachedBodies[i];
	unsigned int n = attachedObject->get_objects_size();
	link->attachedBodies.resize(0);
	
	// create the new ones
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    robot_msgs::PointStamped center;
	    robot_msgs::PointStamped centerP;
	    center.point.x = attachedObject->objects[i].center.x;
	    center.point.y = attachedObject->objects[i].center.y;
	    center.point.z = attachedObject->objects[i].center.z;
	    center.header  = attachedObject->header;
	    bool err = false;
	    try
	    {
		tf_->transformPoint(attachedObject->link_name, center, centerP);
	    }
	    catch(...)
	    {
		err = true;
		ROS_ERROR("Unable to transform object to be attached from frame '%s' to frame '%s'", attachedObject->header.frame_id.c_str(), attachedObject->link_name.c_str());
	    }
	    if (err)
		continue;
	    
	    unsigned int j = link->attachedBodies.size();
	    link->attachedBodies.push_back(new planning_models::KinematicModel::AttachedBody());
	    link->attachedBodies[j]->attachTrans.setOrigin(btVector3(centerP.point.x, centerP.point.y, centerP.point.z));
	    
	    // this is a HACK! we should have orientation
	    shapes::Box *box = new shapes::Box();
	    box->size[0] = attachedObject->objects[i].max_bound.x - attachedObject->objects[i].min_bound.x;
	    box->size[1] = attachedObject->objects[i].max_bound.y - attachedObject->objects[i].min_bound.y;
	    box->size[2] = attachedObject->objects[i].max_bound.z - attachedObject->objects[i].min_bound.z;
	    link->attachedBodies[j]->shape = box;
	}
	
	// update the collision model
	collisionSpace_->updateAttachedBodies();
	ROS_DEBUG("Link '%s' on '%s' has %d objects attached", attachedObject->link_name.c_str(), attachedObject->robot_name.c_str(), n);
    }
    else
	ROS_WARN("Unable to attach object to link '%s' on '%s'", attachedObject->link_name.c_str(), attachedObject->robot_name.c_str());
    collisionSpace_->unlock();
    if (link && (onAfterAttachBody_ != NULL))
	onAfterAttachBody_(link);
}
