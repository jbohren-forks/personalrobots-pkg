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

/* Author: Wim Meeussen */

#ifndef __PEOPLE_TRACKING_NODE__
#define __PEOPLE_TRACKING_NODE__

#include <string>
#include <boost/thread/mutex.hpp>

// ros stuff
#include <ros/node.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// people tracking stuff
#include "tracker.h"
#include "detector_particle.h"
#include "gaussian_vector.h"

// messages
#include <std_msgs/PointCloud.h>
#include <robot_msgs/PositionMeasurement.h>
#include <message_sequencing/time_sequencer.h>
 
// log files
#include <fstream>


namespace estimation
{

class PeopleTrackingNode: public ros::node
{
public:
  /// constructor
  PeopleTrackingNode(const std::string& node_name);

  /// destructor
  virtual ~PeopleTrackingNode();

  /// callback for messages
  void callbackRcv(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message);

  /// callback for dropped messages
  void callbackDrop(const boost::shared_ptr<robot_msgs::PositionMeasurement>& message);

  /// tracker loop
  void spin();


private:
  std::string node_name_;

  /// message sequencer
  message_sequencing::TimeSequencer<robot_msgs::PositionMeasurement>  message_sequencer_;

  /// trackers
  std::list<Tracker*> trackers_;

  // tf listener
  tf::TransformListener robot_state_;

  unsigned int tracker_counter_;
  double freq_, start_distance_min_, reliability_threshold_;
  BFL::StatePosVel sys_sigma_;
  std::string fixed_frame_;
  boost::mutex filter_mutex_;

  std::vector<std_msgs::Point32> meas_visualize_;
  unsigned int meas_visualize_counter_;


}; // class

}; // namespace

#endif
