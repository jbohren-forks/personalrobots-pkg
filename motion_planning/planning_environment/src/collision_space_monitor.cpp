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

namespace planning_environment
{
    
    static inline double radiusOfBox(const robot_msgs::Point32 &point)
    {
	return std::max(std::max(point.x, point.y), point.z) * 1.73;
    }
}

void planning_environment::CollisionSpaceMonitor::setupCSM(void)
{
    onBeforeMapUpdate_ = NULL;
    onAfterMapUpdate_  = NULL;
    onAfterAttachBody_ = NULL;
    haveMap_ = false;
    collisionSpace_ = cm_->getODECollisionModel().get();
    
    collisionMapSubscriber_ = nh_.subscribe("collision_map", 1, &CollisionSpaceMonitor::collisionMapCallback, this);
    ROS_DEBUG("Listening to collision_map");

    if (cm_->loadedModels())
    {
	attachBodySubscriber_ = nh_.subscribe("attach_object", 1, &CollisionSpaceMonitor::attachObjectCallback, this);
	ROS_DEBUG("Listening to attach_object");
    }
}

bool planning_environment::CollisionSpaceMonitor::isMapUpdated(double sec) const
{
    if (sec > 0 && lastMapUpdate_ < ros::Time::now() - ros::Duration(sec))
	return false;
    else
	return true;
}

void planning_environment::CollisionSpaceMonitor::collisionMapCallback(const robot_msgs::CollisionMapConstPtr &collisionMap)
{
    unsigned int n = collisionMap->get_boxes_size();
    ROS_DEBUG("Received %u points (collision map)", n);
    
    if (onBeforeMapUpdate_ != NULL)
	onBeforeMapUpdate_(collisionMap);
    
    ros::WallTime startTime = ros::WallTime::now();
    double *data = new double[4 * n];	
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	unsigned int i4 = i * 4;	    
	data[i4    ] = collisionMap->boxes[i].center.x;
	data[i4 + 1] = collisionMap->boxes[i].center.y;
	data[i4 + 2] = collisionMap->boxes[i].center.z;
	
	data[i4 + 3] = radiusOfBox(collisionMap->boxes[i].extents);
    }
    
    collisionSpace_->lock();
    collisionSpace_->clearObstacles();
    collisionSpace_->addPointCloud(n, data);
    collisionSpace_->unlock();
    
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
	link->attachedBodies.resize(n);
	
	// create the new ones
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    link->attachedBodies[i] = new planning_models::KinematicModel::AttachedBody();
	    
	    robot_msgs::PointStamped center;
	    robot_msgs::PointStamped centerP;
	    center.point.x = attachedObject->objects[i].center.x;
	    center.point.y = attachedObject->objects[i].center.y;
	    center.point.z = attachedObject->objects[i].center.z;
	    center.header  = attachedObject->header;
	    tf_.transformPoint(attachedObject->link_name, center, centerP);
	    
	    link->attachedBodies[i]->attachTrans.setOrigin(btVector3(centerP.point.x, centerP.point.y, centerP.point.z));
	    
	    // this is a HACK! we should have orientation
	    planning_models::shapes::Box *box = new planning_models::shapes::Box();
	    box->size[0] = attachedObject->objects[i].max_bound.x - attachedObject->objects[i].min_bound.x;
	    box->size[1] = attachedObject->objects[i].max_bound.y - attachedObject->objects[i].min_bound.y;
	    box->size[2] = attachedObject->objects[i].max_bound.z - attachedObject->objects[i].min_bound.z;
	    link->attachedBodies[i]->shape = box;
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
