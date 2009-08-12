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

#include <robot_voxelizer/robot_voxelizer.h>


RobotVoxelizer::RobotVoxelizer()
{
	node_.param("~padding", padding_, .01);
	node_.param("~visualize", bVisualize_, true);
	node_.param("~resolution", resolution_, .02);
	node_.param<std::string>("~working_frame", working_frame_, "base_link");
	node_.param<std::string>("~robot_description_param", robot_description_, "robot_description");

	inv_resolution_ = 1.0 / resolution_;
	
	if(bVisualize_)
		marker_publisher_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	
	joint_states_subscriber_ = node_.subscribe("/joint_states", 1, &RobotVoxelizer::jointStatesCallback, this);

	robot_model_ = new planning_environment::RobotModels(robot_description_);
	monitor_ = new planning_environment::KinematicModelStateMonitor(robot_model_, &tf_, working_frame_);
}

RobotVoxelizer::~RobotVoxelizer()
{
	delete robot_model_;
	delete monitor_;
}

bool RobotVoxelizer::init()
{
	link_names_ = robot_model_->getCollisionCheckLinks();
	kmodel_ = robot_model_->getKinematicModel();
	bodies_.resize(link_names_.size());

	
	for(unsigned int i = 0; i < link_names_.size(); ++i)
	{
		bodies_[i] = bodies::createBodyFromShape(monitor_->getKinematicModel()->getLink(link_names_[i])->shape);
		bodies_[i]->setPadding(padding_);
	}
	
	ROS_DEBUG("Pose is %sincluded in the robot state.", monitor_->isPoseIncluded() ? "" : "not ");
	ROS_DEBUG("Pose is %supdated.", monitor_->isPoseUpdated(1) ? "" : "not ");
	ROS_DEBUG("Robot state monitor is %sstarted.", monitor_->isStateMonitorStarted() ? "" : "not ");

	updateSelfCollisionBodies();

	if(bVisualize_)
	{
		self_collision_voxels_.clear();
		getVoxelsInSelfCollisionBodies(self_collision_voxels_);
		updateVisualizations(self_collision_voxels_);
	}
	
	return true;
}

void RobotVoxelizer::updateSelfCollisionBodies()
{
	monitor_->getKinematicModel()->lock();
	monitor_->getKinematicModel()->computeTransforms(monitor_->getRobotState()->getParams());

	for(unsigned int i = 0; i < link_names_.size(); ++i)
	{
		bodies_[i]->setPose(monitor_->getKinematicModel()->getLink(link_names_[i])->globalTrans);
	
		ROS_DEBUG("%s: pose: %.3f %.3f %.3f %.3f  padding: %.3f scale: %.2f", link_names_[i].c_str(), bodies_[i]->getPose().getOrigin().x(), bodies_[i]->getPose().getOrigin().y(), 
							bodies_[i]->getPose().getOrigin().z(), bodies_[i]->getPose().getOrigin().w(), bodies_[i]->getPadding(), bodies_[i]->getScale());
	}

	monitor_->getKinematicModel()->unlock();
}

void RobotVoxelizer::getVoxelsInSelfCollisionBodies(std::vector<btVector3> &voxels)
{
	monitor_->getKinematicModel()->lock();

	for(unsigned int i = 0; i < link_names_.size(); ++i)
	{
		getVoxelsInBody(*(bodies_[i]), voxels);
	}

	monitor_->getKinematicModel()->unlock();
}

void RobotVoxelizer::jointStatesCallback(const mechanism_msgs::JointStatesConstPtr &joint_states)
{
	updateSelfCollisionBodies();

	if(bVisualize_)
	{
		self_collision_voxels_.clear();
		getVoxelsInSelfCollisionBodies(self_collision_voxels_);
		updateVisualizations(self_collision_voxels_);
	}
}

void RobotVoxelizer::getVoxelsInBody(const bodies::Body &body, std::vector<std::vector<double> > &voxels)
{
	body.computeBoundingSphere(bounding_sphere_);
	int x_min,x_max,y_min,y_max,z_min,z_max;
	std::vector<double> xyz_world(3);
	
	worldToGrid(bounding_sphere_.center,bounding_sphere_.center.x()-bounding_sphere_.radius,bounding_sphere_.center.y()-bounding_sphere_.radius,	
							bounding_sphere_.center.z()-bounding_sphere_.radius, x_min,y_min,z_min);
	worldToGrid(bounding_sphere_.center,bounding_sphere_.center.x()+bounding_sphere_.radius,bounding_sphere_.center.y()+bounding_sphere_.radius,	
							bounding_sphere_.center.z()+bounding_sphere_.radius, x_max,y_max,z_max);
	
	ROS_INFO("xyz_min: %i %i %i xyz_max: %i %i %i",x_min,y_min,z_min,x_max,y_max,z_max);
	
	voxels.reserve(30000);
	for(int x = x_min; x <= x_max; ++x)
	{
		for(int y = y_min; y <= y_max; ++y)
		{
			for(int z = z_min; z <= z_max; ++z)
			{
				gridToWorld(bounding_sphere_.center,x,y,z,xyz_world[0], xyz_world[1], xyz_world[2]);
				if(body.containsPoint(xyz_world[0], xyz_world[1], xyz_world[2]))
					voxels.push_back(xyz_world);
			}
		}
	}
	
// 	ROS_INFO("number of occupied cells in bounding sphere: %i", cells.size());
}

void RobotVoxelizer::getVoxelsInBody(const bodies::Body &body, std::vector<btVector3> &voxels)
{
	body.computeBoundingSphere(bounding_sphere_);
	int x,y,z,x_min,x_max,y_min,y_max,z_min,z_max;
	double xw,yw,zw;
	btVector3 v;
	
	worldToGrid(bounding_sphere_.center,bounding_sphere_.center.x()-bounding_sphere_.radius,bounding_sphere_.center.y()-bounding_sphere_.radius,	
							bounding_sphere_.center.z()-bounding_sphere_.radius, x_min,y_min,z_min);
	worldToGrid(bounding_sphere_.center,bounding_sphere_.center.x()+bounding_sphere_.radius,bounding_sphere_.center.y()+bounding_sphere_.radius,	
							bounding_sphere_.center.z()+bounding_sphere_.radius, x_max,y_max,z_max);
	
	ROS_INFO("xyz_min: %i %i %i xyz_max: %i %i %i",x_min,y_min,z_min,x_max,y_max,z_max);
	
	voxels.reserve(30000);
	for(x = x_min; x <= x_max; ++x)
	{
		for(y = y_min; y <= y_max; ++y)
		{
			for(z = z_min; z <= z_max; ++z)
			{
				gridToWorld(bounding_sphere_.center,x,y,z,xw,yw,zw);
				if(body.containsPoint(xw,yw,zw))
				{	
					v.setX(xw);
					v.setY(yw);
					v.setZ(zw);
					voxels.push_back(v);
				}
			}
		}
	}
	
// 	ROS_INFO("number of occupied voxels in bounding sphere: %i", voxels.size());
}

void RobotVoxelizer::updateVisualizations(const std::vector<std::vector<double> > &voxels)
{
	std::string frame_id = "base_link";
	visualization_msgs::Marker obs_marker;
	obs_marker.header.frame_id = frame_id;
	obs_marker.header.stamp = ros::Time::now();
	obs_marker.ns = "mesh_voxelizer";
	obs_marker.id = 0;
	obs_marker.type = visualization_msgs::Marker::CUBE_LIST;
	obs_marker.action = 0;
	obs_marker.scale.x = resolution_;
	obs_marker.scale.y = resolution_;
	obs_marker.scale.z = resolution_;
	obs_marker.color.r = 0.0;
	obs_marker.color.g = 0.25;
	obs_marker.color.b = 0.75;
	obs_marker.color.a = 0.3;
	obs_marker.lifetime = ros::Duration(30.0);

	obs_marker.points.resize(voxels.size());
	for (unsigned int k = 0; k < voxels.size(); ++k) 
	{
		int last = obs_marker.points.size();
		obs_marker.points.resize(last + 1);
		obs_marker.points[last].x = voxels[k][0];
		obs_marker.points[last].y = voxels[k][1];
		obs_marker.points[last].z = voxels[k][2];
	}
	
	ROS_DEBUG("publishing markers: %d obstacles", obs_marker.points.size());
	marker_publisher_.publish(obs_marker);
}

void RobotVoxelizer::updateVisualizations(const std::vector<btVector3> &voxels)
{
	std::string frame_id = "base_link";
	visualization_msgs::Marker obs_marker;
	obs_marker.header.frame_id = frame_id;
	obs_marker.header.stamp = ros::Time::now();
	obs_marker.ns = "mesh_voxelizer";
	obs_marker.id = 0;
	obs_marker.type = visualization_msgs::Marker::CUBE_LIST;
	obs_marker.action = 0;
	obs_marker.scale.x = resolution_;
	obs_marker.scale.y = resolution_;
	obs_marker.scale.z = resolution_;
	obs_marker.color.r = 0.0;
	obs_marker.color.g = 0.25;
	obs_marker.color.b = 0.75;
	obs_marker.color.a = 0.3;
	obs_marker.lifetime = ros::Duration(30.0);

	obs_marker.points.resize(voxels.size());
	for (unsigned int k = 0; k < voxels.size(); ++k) 
	{
		int last = obs_marker.points.size();
		obs_marker.points.resize(last + 1);
		obs_marker.points[last].x = voxels[k].x();
		obs_marker.points[last].y = voxels[k].y();
		obs_marker.points[last].z = voxels[k].z();
	}
	
	ROS_DEBUG("publishing markers: %d obstacles", obs_marker.points.size());
	marker_publisher_.publish(obs_marker);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mesh_voxelizer_node");
  RobotVoxelizer mesh_voxelizer;
  if(!mesh_voxelizer.init())
    return 1;

  ros::spin();
  return 0;
}

/**
me:  i just want to confirm that if I use the kinematicModelStateMonitor, then the bodies are  updated by mechanism state
 Ioan:  yes
you get that and it maintains the current state, it does not do fwd kinematics automatically though
when you need the poses you need to do something like:
monitor->getKinematicModel()->lock();
monitor->getKinematicModel()->computeTransforms(monitor->getRobotState()->getParams());
and then you get the pose of a link by doing smth like:
btTransform pose = monitor->getKinematicModel()->getLink("my_link")->globalTransf;
and then:
body->setPose(pose);
where body has been created as:
bodies::Body *body = bodies::createBodyFromShape(monitor->getKinematicModel()->getLink("my_link")->shape);
of course when you are done: 
monitor->getKinematicModel()->unlock();
the locking is needed only if you want to make sure you don't get changes on the datastructure you are working on
say... you receive an atached_object message

when you do this you have access to all the attached objects as well for instance, grasped objects
as well you should voxelize those as well 
check the monitor->getKinematicModel()->getLink("my_link")->attachedObjects
(i may not remember the correct member variable)
but it is a vector of AttachedObject *
which again has shape & pose
the pose of that is also updated by computeTransforms()
this should be a minor addition
and at least you know you can deal with attached objects
 me:  ok very cool
I also want this to be able to handle other meshes
not only the robot
lets say a trimesh of a glass
 Ioan:  yep
same code will work
if the shape is an instance of a trimesh
 */

