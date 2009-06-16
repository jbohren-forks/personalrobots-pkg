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

<hr>

@section usage Usage
@verbatim
$ display_planner_collision_model [standard ROS args]
@endverbatim

@par Example

@verbatim
$ display_planner_collision_model robot_description:=robotdesc/pr2
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name/type):
- @b "visualization_marker"/visualization_msgs::Marker

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):
- None


<hr>

@section parameters ROS parameters
- None

**/

#include <planning_environment/collision_space_monitor.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Byte.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

class DisplayPlannerCollisionModel
{
public:

    DisplayPlannerCollisionModel(void)
    {
	id_ = 0;
	visualizationMarkerPublisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
	collisionModels_ = new planning_environment::CollisionModels("robot_description");
	collisionSpaceMonitor_ = new planning_environment::CollisionSpaceMonitor(collisionModels_);
	if (collisionModels_->loadedModels())
	{
	    collisionSpaceMonitor_->setOnAfterMapUpdateCallback(boost::bind(&DisplayPlannerCollisionModel::afterWorldUpdate, this, _1));
	    collisionSpaceMonitor_->setOnAfterAttachBodyCallback(boost::bind(&DisplayPlannerCollisionModel::afterAttachBody, this, _1));
	}
    }

    virtual ~DisplayPlannerCollisionModel(void)
    {
	delete collisionSpaceMonitor_;
	delete collisionModels_;
    }

protected:

    void afterWorldUpdate(const robot_msgs::CollisionMapConstPtr &collisionMap)
    {
	unsigned int n = collisionMap->get_boxes_size();
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    const robot_msgs::Point32 &point = collisionMap->boxes[i].center;
	    const robot_msgs::Point32 &extents = collisionMap->boxes[i].extents;
	    sendPoint(point.x, point.y, point.z,
		      std::max(std::max(extents.x, extents.y), extents.z) * 1.73,
		      collisionMap->header, 1);
	}
    }

    void afterAttachBody(planning_models::KinematicModel::Link *link)
    {
	roslib::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = link->name;
	for (unsigned int i = 0 ; i < link->attachedBodies.size() ; ++i)
        {
            planning_models::shapes::Box *box = dynamic_cast<planning_models::shapes::Box*>(link->attachedBodies[i]->shape);
            if (box)
            {
                btVector3 &v = link->attachedBodies[i]->attachTrans.getOrigin();
                sendPoint(v.x(), v.y(), v.z(), std::max(std::max(box->size[0], box->size[1]), box->size[2] / 2.0), header, 0);
            }
        }
    }

private:

    void sendPoint(double x, double y, double z, double radius, const roslib::Header &header, int color)
    {
	visualization_msgs::Marker mk;
	mk.header = header;

	mk.ns = "display_planner_collision_model";
	mk.id = id_++;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;

	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;

	mk.scale.x = mk.scale.y = mk.scale.z = radius * 2.0;

	mk.color.a = 1.0;

	if (color == 0)
	{
	    mk.color.r = 1.0;
	    mk.color.g = 0.04;
	    mk.color.b = 0.04;
	}
	else
	{
	    mk.color.r = 0.04;
	    mk.color.g = 1.0;
	    mk.color.b = 0.04;
	}
	
	visualizationMarkerPublisher_.publish(mk);
    }

    int                                          id_;
    ros::Publisher                               visualizationMarkerPublisher_;
    ros::NodeHandle                              nh_;
    planning_environment::CollisionModels       *collisionModels_;
    planning_environment::CollisionSpaceMonitor *collisionSpaceMonitor_;
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_planner_collision_model");

    DisplayPlannerCollisionModel disp;
    ros::spin();
    
    return 0;
}
