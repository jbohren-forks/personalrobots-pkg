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

#include "kinematic_planning/KinematicStateMonitor.h"
#include <collision_space/environmentODE.h>

#include <robot_msgs/PointCloud.h>
#include <robot_msgs/CollisionMap.h>
#include <robot_msgs/AttachedObject.h>
#include <motion_planning_srvs/CollisionCheckState.h>

/** Main namespace */
namespace kinematic_planning
{
    
    /**
       
       @b CollisionSpaceMonitor is a class which in addition to being aware of a robot model,
       is also aware of a collision space.
       
       <hr>
       
       @section topic ROS topics
       
       Subscribes to (name/type):
       - @b "collision_map"/CollisionMap : data describing the 3D environment
       - @b "attach_object"/AttachedObject : data describing an object to be attached to a link
       
       Publishes to (name/type):
       - None
       
       <hr>
       
       @section services ROS services
       
       Uses (name/type):
       - None
       
       Provides (name/type):
       - @b "set_collision_state"/CollisionCheckState : service to allow enabling and disabling collision checking for links
       
       <hr>
       
       @section notes Notes

       This class inherits from KinematicStateMonitor. Additional
       relevant topics, services and parameters are documented in
       KinematicStateMonitor.

       This class uses the following special groups (from the URDF document)
       
       - "collision_check" with the flag "collision": if present, this is used
       to define the links of the robot that are to be checked for
       collision. If not present, NO COLLISION CHECKING IS PERFORMED!
       
       - any group name with the flag "self_collision": checks the links in
       each group for collision against every other link in the same group.
       
    **/

    class CollisionSpaceMonitor : public KinematicStateMonitor
    {

    public:
	
	CollisionSpaceMonitor(void) : KinematicStateMonitor()
	{
	    m_collisionSpace = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelODE());
	    m_collisionSpace->setSelfCollision(true);
	    
	    // hack for having ground plane
	    m_collisionSpace->addStaticPlane(0.0, 0.0, 1.0, -0.01);
	    collisionSpaceSubscribe();
	}
	
        CollisionSpaceMonitor(boost::shared_ptr<collision_space::EnvironmentModel> collisionSpace) : KinematicStateMonitor() 
	{
	    // use a given collision space
	    m_collisionSpace = collisionSpace;

	    collisionSpaceSubscribe();
	}

	virtual ~CollisionSpaceMonitor(void)
	{
	}
	
	
	bool setCollisionState(motion_planning_srvs::CollisionCheckState::Request &req, motion_planning_srvs::CollisionCheckState::Response &res);

	virtual void loadRobotDescription(void);
	
	bool isMapUpdated(double sec);

    protected:
	
	void collisionSpaceSubscribe(void);
	
	void attachObjectCallback(const robot_msgs::AttachedObjectConstPtr &attachedObject);
	void collisionMapCallback(const robot_msgs::CollisionMapConstPtr &collisionMap);
	
	virtual void beforeWorldUpdate(const robot_msgs::CollisionMapConstPtr &collisionMap);
	virtual void afterWorldUpdate(const robot_msgs::CollisionMapConstPtr &collisionMap);
	virtual void afterAttachBody(const robot_msgs::AttachedObjectConstPtr &attachedObject, planning_models::KinematicModel::Link *link);
	
	boost::shared_ptr<collision_space::EnvironmentModel> m_collisionSpace;
	ros::Subscriber                                      m_collisionMapSubscriber;
	ros::Subscriber                                      m_attachedObjectSubscriber;
	ros::ServiceServer                                   m_setCollisionStateService;
	ros::Time                                            m_lastMapUpdate;
    };
     
} // kinematic_planning
