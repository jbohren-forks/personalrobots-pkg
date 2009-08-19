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

/** \Author: Benjamin Cohen  **/

#include <pm_wrapper/pm_wrapper.h>


pm_wrapper::pm_wrapper()
{
	node_.param("~publish_internal_collision_map", visualize_map_,false);
  node_.param<std::string>("~planning_frame_id", planning_frame_, "base_link");
  std::string pr2_urdf_param;
  node_.searchParam("robot_description",pr2_urdf_param);
  node_.param<std::string>(pr2_urdf_param, robot_description_,"robot_description");
	node_.param<std::string>("~group_name", group_name_,"right_arm");
};

pm_wrapper::~pm_wrapper()
{
  delete collision_model_;
  delete planning_monitor_;
};

bool pm_wrapper::initPlanningMonitor(const std::vector<std::string> &links, tf::TransformListener * tfl, std::string frame_id)
{
  collision_check_links_ = links;
	planning_frame_ = frame_id;
	// these links will be checked for self collision against the links in the yaml file
	collision_model_ = new planning_environment::CollisionModels("robot_description", collision_check_links_);
  planning_monitor_ = new planning_environment::PlanningMonitor(collision_model_, tfl, planning_frame_);

	//error checking
  groupID_ = planning_monitor_->getKinematicModel()->getGroupID(group_name_);
	ROS_DEBUG("groupID = %i", groupID_);
  std::vector<std::string> joint_names;
  planning_monitor_->getKinematicModel()->getJointsInGroup(joint_names, groupID_);
  if(joint_names.empty())
  {
    ROS_WARN("[pm_wrapper] joint group %i is empty", groupID_);
    return false;
  }
  ROS_DEBUG("[pm_wrapper] joint names in group #%i",groupID_);
  for(unsigned int i = 0; i < joint_names.size(); i++)
		ROS_DEBUG("[pm_wrapper]%i: %s", i, joint_names[i].c_str());

	if(visualize_map_)
	{
		if (collision_model_->loadedModels())
		{
			col_map_publisher_ = node_.advertise<mapping_msgs::CollisionMap>("edited_collision_map", 1);
		//	planning_monitor_->setOnAfterMapUpdateCallback(boost::bind(&pm_wrapper::publishInternalCollisionMap, this, _1, _2));
		}
		else
			ROS_DEBUG("collision_model_ didn't properly load the robot model");
	}
	
	ROS_DEBUG("initialized planning monitor");
  return true;
}

planning_models::StateParams* pm_wrapper::fillStartState(const std::vector<motion_planning_msgs::KinematicJoint> &given)
{
  planning_models::StateParams *s = planning_monitor_->getKinematicModel()->newStateParams();
  for (unsigned int i = 0 ; i < given.size() ; ++i)
  {	
    if (!planning_monitor_->getTransformListener()->frameExists(given[i].header.frame_id))
    {
      ROS_ERROR("Frame '%s' for joint '%s' in starting state is unknown", given[i].header.frame_id.c_str(), given[i].joint_name.c_str());
      continue;
    }
    motion_planning_msgs::KinematicJoint kj = given[i];
    if (planning_monitor_->transformJointToFrame(kj, planning_monitor_->getFrameId()))
      s->setParamsJoint(kj.value, kj.joint_name);
  }

  if (s->seenAll())
    return s;
  else
  {
    if (planning_monitor_->haveState())
    {
      ROS_INFO("Using the current state to fill in the starting state for the motion plan");
      std::vector<planning_models::KinematicModel::Joint*> joints;
      planning_monitor_->getKinematicModel()->getJoints(joints);
      for (unsigned int i = 0 ; i < joints.size() ; ++i)
				if (!s->seenJoint(joints[i]->name))
				  s->setParamsJoint(planning_monitor_->getRobotState()->getParamsJoint(joints[i]->name), joints[i]->name);
    	  return s;
    }
  }
  delete s;
  ROS_WARN("fillstartState is returning NULL");
  return NULL;
}

void pm_wrapper::updateRobotState(const std::vector <motion_planning_msgs::KinematicJoint> &robot_state)
{
	planning_models::StateParams *start_state = fillStartState(robot_state);
	
	if(start_state == NULL)
		ROS_WARN("start_state_ == NULL");

	planning_monitor_->getKinematicModel()->computeTransforms(start_state->getParams());
	planning_monitor_->getEnvironmentModel()->updateRobotModel();

	planning_monitor_->getEnvironmentModel()->lock();
	planning_monitor_->getKinematicModel()->lock();

	delete start_state;
	
	ROS_DEBUG("updatedRobotState()");
}

void pm_wrapper::setRobotJointStates(const std::vector<std::string> &joint_names, const std::vector<double> &params)
{
	/** planar links have 3 params {x,y,theta} and joints have 1 param */
	
	planning_models::StateParams robot_state = *(planning_monitor_->getRobotState());
	
	if(!robot_state.setParamsJoints(params, joint_names))
	{
		ROS_WARN("No changes were made to the robot state");
		return;
	}
	
	planning_monitor_->getKinematicModel()->computeTransforms(robot_state.getParams());
	planning_monitor_->getEnvironmentModel()->updateRobotModel();

	planning_monitor_->getEnvironmentModel()->lock();
	planning_monitor_->getKinematicModel()->lock();

	ROS_DEBUG("setJointState()");
}

bool pm_wrapper::areLinksValid(const double * angles)
{
//   ROS_INFO("checking %.3f %.3f %.3f %.3f %.3f %.3f %.3f",angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);

  planning_monitor_->getKinematicModel()->computeTransformsGroup(angles, groupID_);
  planning_monitor_->getEnvironmentModel()->updateRobotModel();

  return !(planning_monitor_->getEnvironmentModel()->isCollision());
}

void pm_wrapper::unlockPM()
{
  // after done planning
  planning_monitor_->getEnvironmentModel()->unlock();
  planning_monitor_->getKinematicModel()->unlock();
}

void pm_wrapper::setObject(shapes::Shape *object, btTransform pose)
{
	object_ = object;
	object_pose_ = pose;
}

void pm_wrapper::removeObject(shapes::Shape *object, const btTransform &pose)
{
	// at this point, the environment model has the collision map inside it
	// get exclusive access
	planning_monitor_->getEnvironmentModel()->lock();
	
	// remove the objects colliding with the box
	ROS_DEBUG("object removed has type %i (origin: %.2f %.2f %.2f) in frame %s", object->type, pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getOrigin().getZ(), planning_monitor_->getFrameId().c_str());
	
	planning_monitor_->getEnvironmentModel()->removeCollidingObjects(object, pose);	
	
	if(visualize_map_)
		publishInternalCollisionMap();

	// release our hold
	planning_monitor_->getEnvironmentModel()->unlock();
}

void pm_wrapper::addObject(const std::string &ns, shapes::Shape *object, const btTransform &pose)
{
	// at this point, the environment model has the collision map inside it
	// get exclusive access
	planning_monitor_->getEnvironmentModel()->lock();
	
	// remove the objects colliding with the box
	ROS_DEBUG("object removed has type %i (origin: %.2f %.2f %.2f) in frame %s", object->type, pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getOrigin().getZ(), planning_monitor_->getFrameId().c_str());
	
	planning_monitor_->getEnvironmentModel()->addObject(ns, object, pose);
	
	if(visualize_map_)
		publishInternalCollisionMap();

	// release our hold
	planning_monitor_->getEnvironmentModel()->unlock();
}

void pm_wrapper::publishInternalCollisionMap()
{
	//publish the collision map
	planning_monitor_->recoverCollisionMap(planning_monitor_->getEnvironmentModel(), col_map_);
	col_map_publisher_.publish(col_map_);	
}

//copied straight from remove_object_example.cpp in planning_environment
 void pm_wrapper::publishMapWithoutObject(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear)
{
	ROS_DEBUG("in publishMapWithoutObject()");
	if(!visualize_map_)
		return;
	
	// we do not care about incremental updates, only re-writes of the map
	if (!clear)
		return;
	
	// at this point, the environment model has the collision map inside it
	// get exclusive access
	planning_monitor_->getEnvironmentModel()->lock();
	
	// get a copy of our own, to play with :)
	collision_space::EnvironmentModel *env = planning_monitor_->getEnvironmentModel()->clone();
	
	// release our hold
	planning_monitor_->getEnvironmentModel()->unlock();

	// remove the objects colliding with the box
	ROS_DEBUG("object removed has type %i (origin: %.2f %.2f %.2f) in frame %s", object_->type, object_pose_.getOrigin().getX(), object_pose_.getOrigin().getY(), object_pose_.getOrigin().getZ(), planning_monitor_->getFrameId().c_str());
	env->removeCollidingObjects(object_, object_pose_);	
	
	// forward the updated map	
	planning_monitor_->recoverCollisionMap(env, col_map_);
	col_map_publisher_.publish(col_map_);

	// throw away our copy
	delete env;
	
	ROS_INFO("Received collision map with %d points and published one with %d points", 
					 (int)collisionMap->get_boxes_size(), (int)col_map_.get_boxes_size());
}

 void pm_wrapper::publishMapWithObject(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear)
{
	ROS_DEBUG("in publishMapWithObject()");
	if(!visualize_map_)
		return;
	
	// we do not care about incremental updates, only re-writes of the map
	if (!clear)
		return;
	
	// at this point, the environment model has the collision map inside it
	// get exclusive access
	planning_monitor_->getEnvironmentModel()->lock();
	
	// get a copy of our own, to play with :)
	collision_space::EnvironmentModel *env = planning_monitor_->getEnvironmentModel()->clone();
	
	// release our hold
	planning_monitor_->getEnvironmentModel()->unlock();

	// remove the objects colliding with the box
	ROS_DEBUG("object added has type %i (origin: %.2f %.2f %.2f) in frame %s", object_->type, object_pose_.getOrigin().getX(), object_pose_.getOrigin().getY(), object_pose_.getOrigin().getZ(), planning_monitor_->getFrameId().c_str());
	
	std::vector<shapes::Shape*> objects(1);
	objects[1] = object_;
	std::vector<btTransform> poses(1);
	poses[0] = object_pose_;
	
	env->addObjects(std::string("added_objects"), objects, poses);	
	
	// forward the updated map
	planning_monitor_->recoverCollisionMap(env, col_map_);
	col_map_publisher_.publish(col_map_);

	// throw away our copy
	delete env;
	
	ROS_INFO("Received collision map with %d points and published one with %d points", 
					 (int)collisionMap->get_boxes_size(), (int)col_map_.get_boxes_size());
}


/*


create a ModelKinematic in ompl_ros and pass into the constructor a planning environment. then create a smooother and pass into the constructor the model kinematic. then use setParams 
to organize the joint angles in the right order */

