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

#ifndef PLANNING_ENVIRONMENT_MONITORS_COLLISION_SPACE_MONITOR_
#define PLANNING_ENVIRONMENT_MONITORS_COLLISION_SPACE_MONITOR_

#include "planning_environment/models/collision_models.h"
#include "planning_environment/monitors/kinematic_model_state_monitor.h"

#include <mapping_msgs/CollisionMap.h>
#include <mapping_msgs/ObjectInMap.h>
#include <boost/thread/mutex.hpp>

namespace planning_environment
{

    /** \brief @b CollisionSpaceMonitor is a class which in addition to being aware of a robot model,
	is also aware of a collision space.
    */
    class CollisionSpaceMonitor : public KinematicModelStateMonitor
    {
    public:
	
	CollisionSpaceMonitor(CollisionModels *cm, tf::TransformListener *tf, std::string frame_id) : KinematicModelStateMonitor(static_cast<RobotModels*>(cm), tf, frame_id)
	{
	    cm_ = cm;
	    setupCSM();
	}
	
	CollisionSpaceMonitor(CollisionModels *cm, tf::TransformListener *tf) : KinematicModelStateMonitor(static_cast<RobotModels*>(cm), tf)
	{
	    cm_ = cm;
	    setupCSM();
	}
	
	virtual ~CollisionSpaceMonitor(void)
	{
	    if (objectInMapNotifier_)
		delete objectInMapNotifier_;
	    if (collisionMapNotifier_)
		delete collisionMapNotifier_;
	    if (collisionMapUpdateNotifier_)
		delete collisionMapUpdateNotifier_;
	}

	/** \brief Return the instance of the environment model maintained */
	collision_space::EnvironmentModel* getEnvironmentModel(void) const
	{
	    return collisionSpace_;
	}
	
	/** \brief Return the instance of collision models that is being used */
	CollisionModels* getCollisionModels(void) const
	{
	    return cm_;
	}
	
	/** \brief Define a callback for before updating a map */
	void setOnBeforeMapUpdateCallback(const boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> &callback)
	{
	    onBeforeMapUpdate_ = callback;
	}

	/** \brief Define a callback for after updating a map */
	void setOnAfterMapUpdateCallback(const boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> &callback)
	{
	    onAfterMapUpdate_ = callback;
	}

	/** \brief Define a callback for updates to the objects in the map */
	void setOnObjectInMapUpdateCallback(const boost::function<void(const mapping_msgs::ObjectInMapConstPtr)> &callback)
	{
	    onObjectInMapUpdate_ = callback;
	}

	/** \brief Return true if  map has been received */
	bool haveMap(void) const
	{
	    return haveMap_;
	}
	
	/** \brief Return true if a map update has been received in the last sec seconds. If sec < 10us, this function always returns true. */
	bool isMapUpdated(double sec) const;
	

	/** \brief Wait until a map is received */
	void waitForMap(void) const;	

	/** \brief Return the last update time for the map */
	const ros::Time& lastMapUpdate(void) const
	{
	    return lastMapUpdate_;
	}
	
    protected:
	
	void setupCSM(void);
	void updateCollisionSpace(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear);
	void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap);
	void collisionMapUpdateCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap);
	void objectInMapCallback(const mapping_msgs::ObjectInMapConstPtr &objectInMap);
	virtual bool attachObject(const mapping_msgs::AttachedObjectConstPtr &attachedObject);

	CollisionModels                                                *cm_;
	collision_space::EnvironmentModel                              *collisionSpace_;
	boost::mutex                                                    mapUpdateLock_;
	
	bool                                                            haveMap_;
	ros::Time                                                       lastMapUpdate_;	
	tf::MessageNotifier<mapping_msgs::CollisionMap>                *collisionMapNotifier_;
	tf::MessageNotifier<mapping_msgs::CollisionMap>                *collisionMapUpdateNotifier_;
	tf::MessageNotifier<mapping_msgs::ObjectInMap>                 *objectInMapNotifier_;
	
	boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> onBeforeMapUpdate_;
	boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> onAfterMapUpdate_;
	boost::function<void(const mapping_msgs::ObjectInMapConstPtr)>        onObjectInMapUpdate_;
    
    };
    
	
}

#endif

