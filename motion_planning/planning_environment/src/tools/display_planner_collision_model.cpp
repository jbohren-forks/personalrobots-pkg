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

@b DisplayPlannerCollisionModel is a node that displays the state of
the robot's collision space, as seen by the planner

**/

#include <planning_environment/monitors/collision_space_monitor.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

class DisplayPlannerCollisionModel
{
public:

    DisplayPlannerCollisionModel(void)
    {
	visualizationMarkerPublisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
	collisionModels_ = new planning_environment::CollisionModels("robot_description");
	nh_.param<bool>("~skip_collision_map", skip_collision_map_, false);
	
	if (collisionModels_->loadedModels())
	{
	    collisionSpaceMonitor_ = new planning_environment::CollisionSpaceMonitor(collisionModels_, &tf_);
	    if (!skip_collision_map_)
		collisionSpaceMonitor_->setOnAfterMapUpdateCallback(boost::bind(&DisplayPlannerCollisionModel::afterWorldUpdate, this, _1, _2));
	    collisionSpaceMonitor_->setOnAfterAttachBodyCallback(boost::bind(&DisplayPlannerCollisionModel::afterAttachBody, this, _1, _2));
	    collisionSpaceMonitor_->setOnObjectInMapUpdateCallback(boost::bind(&DisplayPlannerCollisionModel::objectInMapUpdate, this, _1));
	}
	else
	    collisionSpaceMonitor_ = NULL;
    }

    virtual ~DisplayPlannerCollisionModel(void)
    {
	if (collisionSpaceMonitor_)
	    delete collisionSpaceMonitor_;
	if (collisionModels_)
	    delete collisionModels_;
    }

protected:

    void objectInMapUpdate(const mapping_msgs::ObjectInMapConstPtr &objectInMap)
    {
	visualization_msgs::Marker mk;
	mk.ns = nh_.getName() + "_" + objectInMap->id;
	mk.id = 0;
	mk.header = objectInMap->header;
	
	if (objectInMap->action == mapping_msgs::ObjectInMap::ADD)
	{
	    mk.action = visualization_msgs::Marker::ADD;
	    setObject(objectInMap->object, mk);
	    mk.pose = objectInMap->pose;
	}
	else
	    mk.action = visualization_msgs::Marker::DELETE;
	mk.color.a = 0.5;
	mk.color.r = 0.1;
	mk.color.g = 0.8;
	mk.color.b = 0.3;
	visualizationMarkerPublisher_.publish(mk);
    }
    
    void afterWorldUpdate(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear)
    {
	if (!clear)
	    return;
	
	unsigned int n = collisionMap->get_boxes_size();
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    const robot_msgs::Point32 &point = collisionMap->boxes[i].center;
	    const robot_msgs::Point32 &extents = collisionMap->boxes[i].extents;
	    sendPoint(i, nh_.getName(), point.x, point.y, point.z,
		      std::max(std::max(extents.x, extents.y), extents.z) * 0.867,
		      collisionMap->header);
	}
    }
    
    void afterAttachBody(planning_models::KinematicModel::Link *link, const mapping_msgs::AttachedObjectConstPtr &attachedObject)
    {
	/*	roslib::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = link->name;
	for (unsigned int i = 0 ; i < link->attachedBodies.size() ; ++i)
        {
            shapes::Box *box = dynamic_cast<shapes::Box*>(link->attachedBodies[i]->shape);
            if (box)
            {
                btVector3 &v = link->attachedBodies[i]->attachTrans.getOrigin();
                sendPoint(v.x(), v.y(), v.z(), std::max(std::max(box->size[0], box->size[1]), box->size[2] / 2.0), header, 0);
            }
	    } */
	ROS_INFO("should display attached body");	
    }

private:

    void setObject(const mapping_msgs::Object &obj, visualization_msgs::Marker &mk)
    {
	switch (obj.type)
	{
	case mapping_msgs::Object::SPHERE:
	    mk.type = visualization_msgs::Marker::SPHERE;
	    mk.scale.x = mk.scale.y = mk.scale.z = obj.dimensions[0] * 2.0;
	    break;
	    
	case mapping_msgs::Object::BOX:
	    mk.type = visualization_msgs::Marker::CUBE;
	    mk.scale.x = obj.dimensions[0];
	    mk.scale.y = obj.dimensions[1];
	    mk.scale.z = obj.dimensions[2];
	    break;

	case mapping_msgs::Object::CYLINDER:
	    mk.type = visualization_msgs::Marker::CYLINDER;
	    mk.scale.x = obj.dimensions[0] * 2.0;
	    mk.scale.y = obj.dimensions[0] * 2.0;
	    mk.scale.z = obj.dimensions[1];
	    break;

	case mapping_msgs::Object::MESH:
	    mk.type = visualization_msgs::Marker::LINE_LIST;
	    {
		unsigned int nt = obj.triangles.size() / 3;
		for (unsigned int i = 0 ; i < nt ; ++i)
		{
		    mk.points.push_back(obj.vertices[obj.triangles[3*i]]);
		    mk.points.push_back(obj.vertices[obj.triangles[3*i+1]]);
		    mk.points.push_back(obj.vertices[obj.triangles[3*i]]);
		    mk.points.push_back(obj.vertices[obj.triangles[3*i+2]]);
		    mk.points.push_back(obj.vertices[obj.triangles[3*i+1]]);
		    mk.points.push_back(obj.vertices[obj.triangles[3*i+2]]);
		}
	    }
	    
	    break;
	    
	default:
	    ROS_ERROR("Unknown object type: %d", (int)obj.type);
	}
    }
    
    void sendPoint(int id, const std::string &ns, double x, double y, double z, double radius, const roslib::Header &header)
    {
	visualization_msgs::Marker mk;
	mk.header = header;

	mk.ns = ns;
	mk.id = id;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;

	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;

	mk.scale.x = mk.scale.y = mk.scale.z = radius * 2.0;

	mk.color.a = 1.0;
	mk.color.r = 0.9;
	mk.color.g = 0.1;
	mk.color.b = 0.1;
	mk.lifetime = ros::Duration(10.0);
	
	visualizationMarkerPublisher_.publish(mk);
    }

    ros::NodeHandle                              nh_;
    tf::TransformListener                        tf_;
    planning_environment::CollisionModels       *collisionModels_;
    planning_environment::CollisionSpaceMonitor *collisionSpaceMonitor_;
    ros::Publisher                               visualizationMarkerPublisher_;
    bool                                         skip_collision_map_;

    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_planner_collision_model");

    DisplayPlannerCollisionModel disp;
    ros::spin();
    
    return 0;
}
