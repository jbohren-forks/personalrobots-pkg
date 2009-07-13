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

@b ClearKnownObjects is a node that removes known objects from a
collision map.

**/

#include "planning_environment/models/collision_models.h"
#include "planning_environment/monitors/kinematic_model_state_monitor.h"
#include "planning_environment/util/construct_object.h"

#include <tf/message_notifier.h>
#include <mapping_msgs/CollisionMap.h>
#include <mapping_msgs/AttachedObject.h>

class ClearKnownObjects
{
public:

    ClearKnownObjects(void)
    {    
	cm_ = new planning_environment::CollisionModels("robot_description");
	if (cm_->loadedModels())
	{
	    kmsm_ = new planning_environment::KinematicModelStateMonitor(static_cast<planning_environment::RobotModels*>(cm_), &tf_);
	
	    collisionMapNotifier_ = new tf::MessageNotifier<mapping_msgs::CollisionMap>(tf_, boost::bind(&ClearKnownObjects::collisionMapCallback, this, _1), "collision_map", kmsm_->getFrameId(), 1);
	    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapNotifier_->getTargetFramesString().c_str());
	    
	    collisionMapUpdateNotifier_ = new tf::MessageNotifier<mapping_msgs::CollisionMap>(tf_, boost::bind(&ClearKnownObjects::collisionMapUpdateCallback, this, _1), "collision_map_update", kmsm_->getFrameId(), 1);
	    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateNotifier_->getTargetFramesString().c_str());
	    
	    attachedBodyNotifier_ = new tf::MessageNotifier<mapping_msgs::AttachedObject>(tf_, boost::bind(&ClearKnownObjects::attachObjectCallback, this, _1), "attach_object", "", 1);
	    attachedBodyNotifier_->setTargetFrame(cm_->getCollisionCheckLinks());
	    ROS_DEBUG("Listening to attach_object using message notifier with target frame %s", attachedBodyNotifier_->getTargetFramesString().c_str());
	}
	else
	{
	    kmsm_ = NULL;
	    collisionMapNotifier_ = NULL;
	    collisionMapUpdateNotifier_ = NULL;
	    attachedBodyNotifier_ = NULL;
	}
    }

    ~ClearKnownObjects(void)
    {
	if (collisionMapNotifier_)
	    delete collisionMapNotifier_;
	if (collisionMapUpdateNotifier_)
	    delete collisionMapUpdateNotifier_;
	if (kmsm_)
	    delete kmsm_;
	if (cm_)
	    delete cm_;
    }
    
    void run(void)
    {   
	ros::spin();
    }
    
private:

    void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap)
    {
    }
    
    void collisionMapUpdateCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap)
    {
    }
    
    void attachObjectCallback(const mapping_msgs::AttachedObjectConstPtr &attachedObject)
    {
    }
    

    tf::TransformListener       tf_;
    planning_environment::CollisionModels            *cm_;
    planning_environment::KinematicModelStateMonitor *kmsm_;

    tf::MessageNotifier<mapping_msgs::CollisionMap>     *collisionMapNotifier_;
    tf::MessageNotifier<mapping_msgs::CollisionMap>     *collisionMapUpdateNotifier_;
    tf::MessageNotifier<mapping_msgs::AttachedObject>   *attachedBodyNotifier_;

};



    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "clear_known_objects", ros::init_options::AnonymousName);

    ClearKnownObjects cko;
    cko.run();
    
    return 0;
}
