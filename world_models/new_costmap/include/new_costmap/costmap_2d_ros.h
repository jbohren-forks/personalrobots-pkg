/*********************************************************************
 *
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
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef COSTMAP_COSTMAP_2D_ROS_H_
#define COSTMAP_COSTMAP_2D_ROS_H_

#include <ros/node.h>
#include <ros/console.h>
#include <new_costmap/costmap_2d.h>
#include <new_costmap/observation_buffer.h>
#include <robot_srvs/StaticMap.h>
#include <robot_msgs/Polyline2D.h>
#include <map>
#include <vector>
#include <string>
#include <sstream>

#include <tf/transform_datatypes.h>
#include <tf/message_notifier_base.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

#include <laser_scan/LaserScan.h>
#include <laser_scan/laser_scan.h>

#include <robot_msgs/PointCloud.h>

// Thread suppport
#include <boost/thread.hpp>

using namespace costmap_2d;
using namespace tf;
using namespace robot_msgs;

namespace costmap_2d {

  class Costmap2DROS {
    public:
      Costmap2DROS(ros::Node& ros_node);
      ~Costmap2DROS();

      void laserScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message, ObservationBuffer* buffer);
      void pointCloudCallback(const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& message, ObservationBuffer* buffer);
      void spin();
      void updateMap();
      void resetWindow();
      void publishCostMap();
      Costmap2D getCostMap();
      unsigned char* getCharMap();

    private:
      ros::Node& ros_node_;
      tf::TransformListener tf_; ///< @brief Used for transforming point clouds
      laser_scan::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds
      boost::recursive_mutex observation_lock_; ///< @brief A lock for accessing data in callbacks safely
      boost::recursive_mutex map_lock_; ///< @brief A lock for accessing data in callbacks safely
      Costmap2D* costmap_;
      std::string global_frame_;
      double freq_;
      boost::thread* visualizer_thread_;
      boost::thread* window_reset_thread_;

      std::vector<tf::MessageNotifierBase*> observation_notifiers_; ///< @brief Used to make sure that transforms are available for each sensor
      std::vector<ObservationBuffer*> observation_buffers_; ///< @brief Used to store observations from various sensors

  };
};

#endif

