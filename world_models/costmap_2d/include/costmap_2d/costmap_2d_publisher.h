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
#ifndef COSTMAP_COSTMAP_2D_PUBLISHER_H_
#define COSTMAP_COSTMAP_2D_PUBLISHER_H_
#include <ros/node.h>
#include <ros/console.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/rate.h>
#include <visualization_msgs/Polyline.h>
#include <boost/thread.hpp>

namespace costmap_2d {
  class Costmap2DPublisher {
    public:
      Costmap2DPublisher(ros::Node& ros_node, double publish_frequency, std::string global_frame, std::string topic_prefix = std::string(""));
      ~Costmap2DPublisher();

      void publishCostmap();
      void updateCostmapData(const Costmap2D& costmap);
      bool active() {return active_;}

    private:
      void mapPublishLoop(double frequency);

      ros::Node& ros_node_;
      std::string global_frame_,topic_prefix_;
      boost::thread* visualizer_thread_; ///< @brief A thread for publising to the visualizer
      std::vector< std::pair<double, double> > raw_obstacles_, inflated_obstacles_;
      boost::recursive_mutex lock_; ///< @brief A lock
      bool active_;
  };
};
#endif
