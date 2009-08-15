/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <chomp_motion_planner/chomp_collision_space.h>

namespace chomp
{

ChompCollisionSpace::ChompCollisionSpace():
  distance_field_(NULL)
{
}

ChompCollisionSpace::~ChompCollisionSpace()
{
  if (distance_field_)
    delete distance_field_;
  delete collision_map_notifier_;
}

void ChompCollisionSpace::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr& collision_map)
{
  // @TODO transform the collision map to the required frame!!
  if (mutex_.try_lock())
  {
    ros::WallTime start = ros::WallTime::now();
    distance_field_->reset();
    ROS_INFO("Reset prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());
    start = ros::WallTime::now();
    distance_field_->addCollisionMapToField(*collision_map);
    mutex_.unlock();
    ROS_INFO("Updated prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());

    distance_field_->visualize(0.14, 0.15, collision_map->header.frame_id, collision_map->header.stamp);

  }
  else
  {
    ROS_INFO("Skipped collision map update because planning is in progress.");
  }
}

bool ChompCollisionSpace::init()
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  double resolution;

  node_handle_.param("~reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param("~collision_space/size_x", size_x, 2.0);
  node_handle_.param("~collision_space/size_y", size_y, 3.0);
  node_handle_.param("~collision_space/size_z", size_z, 4.0);
  node_handle_.param("~collision_space/origin_x", origin_x, 0.1);
  node_handle_.param("~collision_space/origin_y", origin_y, -1.5);
  node_handle_.param("~collision_space/origin_z", origin_z, -2.0);
  node_handle_.param("~collision_space/resolution", resolution, 0.02);

  distance_field_ = new distance_field::PropagationDistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, 0.16);

  collision_map_notifier_ = new tf::MessageNotifier<mapping_msgs::CollisionMap>(tf_,
      boost::bind(&ChompCollisionSpace::collisionMapCallback, this, _1),
      "collision_map", reference_frame_, 1);
  ROS_INFO("Initialized chomp collision space in %s reference frame.", reference_frame_.c_str());
  return true;
}

}
