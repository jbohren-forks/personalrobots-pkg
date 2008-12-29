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

#ifndef __PEOPLE_TRACKING_NODE__
#define __PEOPLE_TRACKING_NODE__

#include <string>

// ros stuff
#include <ros/node.h>
#include <tf/tf.h>

// people tracking stuff
#include "tracker.h"
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
  PeopleTrackingNode(const string& node_name);

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
  std::vector<Tracker*> trackers_;
  std::vector<std::string> tracker_names_;

  // messages to send
  std_msgs::PointCloud  point_cloud_; 

  double freq_, time_;
  std::vector<tf::Vector3> meas_, vel_;
  BFL::GaussianVector move_;
  BFL::StatePosVel sys_sigma_;
  tf::Vector3 meas_sigma_;
  BFL::StatePosVel prior_sigma_;


}; // class

}; // namespace

#endif
