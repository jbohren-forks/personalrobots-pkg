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
#ifndef COSTMAP_OBSERVATION_BUFFER_H_
#define COSTMAP_OBSERVATION_BUFFER_H_

#include <vector>
#include <list>
#include <string>
#include <ros/time.h>
#include <new_costmap/observation.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <robot_msgs/PointCloud.h>

namespace costmap_2d {
  /**
   * @class ObservationBuffer
   * @brief Takes in point clouds from sensors, transforms them to the desired frame, and stores them 
   */
  class ObservationBuffer {
    public:
      ObservationBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate, 
          tf::TransformListener& tf, std::string global_frame, std::string sensor_frame);

      ~ObservationBuffer();

      //burden is on the user to make sure the transform is available... ie they should use a MessageNotifier
      void bufferCloud(const robot_msgs::PointCloud& cloud);

      void getObservations(std::vector<Observation>& observations);

      bool isCurrent() const;

    private:
      void purgeStaleObservations();
      tf::TransformListener& tf_;
      const ros::Duration observation_keep_time_;
      const ros::Duration expected_update_rate_;
      ros::Time last_updated_;
      std::string global_frame_;
      std::string sensor_frame_;
      std::list<Observation> observation_list_;
      std::string topic_name_;
  };
};
#endif
