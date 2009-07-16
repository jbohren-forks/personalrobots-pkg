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

#ifndef CHOMP_COLLISION_SPACE_H_
#define CHOMP_COLLISION_SPACE_H_

#include <voxel3d/voxel3d.h>
#include <mapping_msgs/CollisionMap.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

namespace chomp
{

class ChompCollisionSpace
{
public:
  ChompCollisionSpace();
  virtual ~ChompCollisionSpace();

  /**
   * \brief Callback for CollisionMap messages
   */
  void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr& collision_map);

  /**
   * \brief Initializes the collision space, listens for messages, etc
   *
   * \return false if not successful
   */
  bool init();

  /**
   * \brief Lock the collision space from updating/reading
   */
  void lock();

  /**
   * \brief Unlock the collision space for updating/reading
   */
  void unlock();

  double getDistanceGradient(double x, double y, double z,
      double& gradient_x, double& gradient_y, double& gradient_z) const;

private:
  Voxel3d* voxel3d_;
  tf::TransformListener tf_;
  tf::MessageNotifier<mapping_msgs::CollisionMap> *collision_map_notifier_;
  //tf::MessageNotifier<mapping_msgs::CollisionMap> *collision_map_update_notifier_;
  std::string reference_frame_;
  ros::NodeHandle node_handle_;
  boost::mutex mutex_;
};

///////////////////////////// inline functions follow ///////////////////////////////////

inline void ChompCollisionSpace::lock()
{
  mutex_.lock();
}

inline void ChompCollisionSpace::unlock()
{
  mutex_.unlock();
}

inline double ChompCollisionSpace::getDistanceGradient(double x, double y, double z,
    double& gradient_x, double& gradient_y, double& gradient_z) const
{
  return voxel3d_->getDistanceGradient(x, y, z, gradient_x, gradient_y, gradient_z);
}

} // namespace chomp

#endif /* CHOMP_COLLISION_SPACE_H_ */
