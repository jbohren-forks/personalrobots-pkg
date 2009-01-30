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

@htmlinclude ../manifest.html

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
- @b "visualizationMarker"/VisualizationMarker 

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

#include "kinematic_planning/CollisionSpaceMonitor.h"

#include <std_msgs/Byte.h>
#include <robot_msgs/VisualizationMarker.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>
using namespace kinematic_planning;

class DisplayPlannerCollisionModel : public ros::Node,
				     public CollisionSpaceMonitor
{
public:
    
    DisplayPlannerCollisionModel(void) : ros::Node("display_planner_collision_model"),
					 CollisionSpaceMonitor(dynamic_cast<ros::Node*>(this)),
					 id_(0)
    {
	advertise<robot_msgs::VisualizationMarker>("visualizationMarker", 10240);

	advertise<robot_msgs::AttachedObject>("attach_object", 1);
    }
    
    virtual ~DisplayPlannerCollisionModel(void)
    {
    }
    
protected:
    
    void afterWorldUpdate(void)
    {
	CollisionSpaceMonitor::afterWorldUpdate();
	
	unsigned int n = m_collisionMap.get_boxes_size();	
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    sendPoint(m_collisionMap.boxes[i].center.x,
		      m_collisionMap.boxes[i].center.y,
		      m_collisionMap.boxes[i].center.z,
		      radiusOfBox(m_collisionMap.boxes[i].extents),
		      m_collisionMap.header, 1);
	}
    }
    
    void afterAttachBody(planning_models::KinematicModel::Link *link)
    {
	rostools::Header header = m_attachedObject.header;
	header.frame_id = link->name;
	for (unsigned int i = 0 ; i < link->attachedBodies.size() ; ++i)
        {
            planning_models::KinematicModel::Box *box = dynamic_cast<planning_models::KinematicModel::Box*>(link->attachedBodies[i]->shape);
            if (box)
            {
                btVector3 &v = link->attachedBodies[i]->attachTrans.getOrigin();
                sendPoint(v.x(), v.y(), v.z(), std::max(std::max(box->size[0], box->size[1]), box->size[2] / 2.0), header, 0);
            }
        }
    }

    
private:
    
    void sendPoint(double x, double y, double z, double radius, const rostools::Header &header, int color)
    {
	robot_msgs::VisualizationMarker mk;
	mk.header = header;
	
	mk.id = id_++;
	mk.type = robot_msgs::VisualizationMarker::SPHERE;
	mk.action = robot_msgs::VisualizationMarker::ADD;
	mk.x = x;
	mk.y = y;
	mk.z = z;

	mk.roll = 0;
	mk.pitch = 0;
	mk.yaw = 0;
	
	mk.xScale = radius * 2.0;
	mk.yScale = radius * 2.0;
	mk.zScale = radius * 2.0;
		
	mk.alpha = 255;

	if (color == 0)
	{
	    mk.r = 255;
	    mk.g = 10;
	    mk.b = 10;
	}
	else
	{
	    mk.r = 10;
	    mk.g = 255;
	    mk.b = 10;
	}
	
	publish("visualizationMarker", mk);
    }

    int id_;
    
};

int main(int argc, char **argv)
{ 
    ros::init(argc, argv);
    
    DisplayPlannerCollisionModel *disp = new DisplayPlannerCollisionModel();
    disp->loadRobotDescription();
    
    disp->spin();
    disp->shutdown();

    
    delete disp;	

    return 0;    
}
