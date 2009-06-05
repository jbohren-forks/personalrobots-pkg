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

#include "kinematic_planning/CollisionSpaceMonitor.h"

namespace kinematic_planning
{  	
    static inline double radiusOfBox(const robot_msgs::Point32 &point)
    {
	return std::max(std::max(point.x, point.y), point.z) * 1.73;
    }
}

void kinematic_planning::CollisionSpaceMonitor::collisionSpaceSubscribe(void)
{
    m_collisionMapSubscriber   = m_nodeHandle.subscribe("collision_map", 1, &CollisionSpaceMonitor::collisionMapCallback, this);
    m_attachedObjectSubscriber = m_nodeHandle.subscribe("attach_object", 1, &CollisionSpaceMonitor::attachObjectCallback, this);
    m_setCollisionStateService = m_nodeHandle.advertiseService("set_collision_state", &CollisionSpaceMonitor::setCollisionState, this);
}

void kinematic_planning::CollisionSpaceMonitor::attachObjectCallback(const robot_msgs::AttachedObjectConstPtr &attachedObject)
{
    m_collisionSpace->lock();
    int model_id = m_collisionSpace->getModelID(attachedObject->robot_name);
    planning_models::KinematicModel::Link *link = model_id >= 0 ? m_kmodel->getLink(attachedObject->link_name) : NULL;
    
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
	    m_tf.transformPoint(attachedObject->link_name, center, centerP);
	    
	    link->attachedBodies[i]->attachTrans.setOrigin(btVector3(centerP.point.x, centerP.point.y, centerP.point.z));
	    
	    // this is a HACK! we should have orientation
	    planning_models::shapes::Box *box = new planning_models::shapes::Box();
	    box->size[0] = attachedObject->objects[i].max_bound.x - attachedObject->objects[i].min_bound.x;
	    box->size[1] = attachedObject->objects[i].max_bound.y - attachedObject->objects[i].min_bound.y;
	    box->size[2] = attachedObject->objects[i].max_bound.z - attachedObject->objects[i].min_bound.z;
	    link->attachedBodies[i]->shape = box;
	}
	
	// update the collision model
	m_collisionSpace->updateAttachedBodies(model_id);
	ROS_INFO("Link '%s' on '%s' has %d objects attached", attachedObject->link_name.c_str(), attachedObject->robot_name.c_str(), n);
    }
    else
	ROS_WARN("Unable to attach object to link '%s' on '%s'", attachedObject->link_name.c_str(), attachedObject->robot_name.c_str());
    m_collisionSpace->unlock();
    if (link)
	afterAttachBody(attachedObject, link);
}

bool kinematic_planning::CollisionSpaceMonitor::setCollisionState(motion_planning_srvs::CollisionCheckState::Request &req, motion_planning_srvs::CollisionCheckState::Response &res)
{
    m_collisionSpace->lock();
    int model_id = m_collisionSpace->getModelID(req.robot_name);
    if (model_id >= 0)
	res.value = m_collisionSpace->setCollisionCheck(model_id, req.link_name, req.value ? true : false);
    else
	res.value = -1;
    m_collisionSpace->unlock();
    if (res.value == -1)
	ROS_WARN("Unable to change collision checking state for link '%s' on '%s'", req.link_name.c_str(), req.robot_name.c_str());
    else
	ROS_INFO("Collision checking for link '%s' on '%s' is now %s", req.link_name.c_str(), req.robot_name.c_str(), res.value ? "enabled" : "disabled");
    return true;	    
}

void kinematic_planning::CollisionSpaceMonitor::loadRobotDescription(void)
{
    KinematicStateMonitor::loadRobotDescription();
    if (m_kmodel)
	m_collisionSpace = m_envModels->getODECollisionModel();
}

bool kinematic_planning::CollisionSpaceMonitor::isMapUpdated(double sec)
{
    if (sec > 0 && m_lastMapUpdate < ros::Time::now() - ros::Duration(sec))
	return false;
    else
	return true;
}

void kinematic_planning::CollisionSpaceMonitor::collisionMapCallback(const robot_msgs::CollisionMapConstPtr &collisionMap)
{
    unsigned int n = collisionMap->get_boxes_size();
    ROS_DEBUG("Received %u points (collision map)", n);
    
    beforeWorldUpdate(collisionMap);
    
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
    
    m_collisionSpace->lock();
    m_collisionSpace->clearObstacles();
    m_collisionSpace->addPointCloud(n, data);
    m_collisionSpace->unlock();
    
    delete[] data;
    
    double tupd = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Updated world model in %f seconds", tupd);
    m_lastMapUpdate = ros::Time::now();
    
    afterWorldUpdate(collisionMap);
}

void kinematic_planning::CollisionSpaceMonitor::beforeWorldUpdate(const robot_msgs::CollisionMapConstPtr &collisionMap)
{
}

void kinematic_planning::CollisionSpaceMonitor::afterWorldUpdate(const robot_msgs::CollisionMapConstPtr &collisionMap)
{
}

void kinematic_planning::CollisionSpaceMonitor::afterAttachBody(const robot_msgs::AttachedObjectConstPtr &attachedObject,
								planning_models::KinematicModel::Link *link)
{
}
