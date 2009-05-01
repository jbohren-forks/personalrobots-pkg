
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include "ros/node.h"
#include "boost/scoped_ptr.hpp"
#include "mechanism_model/controller.h"
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include "mechanism_model/chain.h"
#include "control_toolbox/pid.h"
#include <tf/transform_listener.h>
#include "realtime_tools/realtime_publisher.h"
#include "filters/filter_chain.h"

#include "robot_srvs/SetPoseStamped.h"
#include "robot_msgs/TaskFrameFormalism.h"
#include "robot_msgs/CartesianState.h"

namespace controller {

class CartesianHybridController : public Controller
{
public:
  CartesianHybridController();
  ~CartesianHybridController() {}

  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  virtual void update(void);
  virtual bool starting();

  KDL::Twist pose_desi_;
  KDL::Twist twist_desi_;
  KDL::Wrench wrench_desi_;
  KDL::Frame pose_meas_;
  KDL::Twist twist_meas_;
  KDL::Twist twist_meas_filtered_;

  control_toolbox::Pid pose_pids_[6];  // (x,y,z) position, then (x,y,z) rot
  control_toolbox::Pid twist_pids_[6];

  // x, y, z, rx, ry, rz
  int mode_[6];
  double setpoint_[6];

  KDL::Frame task_frame_offset_;  // task frame in the root frame
  KDL::Frame tool_frame_offset_;  // tool frame in the ee frame

  mechanism::Chain chain_;
  KDL::Chain kdl_chain_;
  mechanism::RobotState *robot_;
  double last_time_;

  int initial_mode_;

  double saturated_velocity_, saturated_rot_velocity_;

  bool use_filter_;
  filters::FilterChain<double> twist_filter_;
};

class CartesianHybridControllerNode : public Controller
{
public:
  CartesianHybridControllerNode();
  ~CartesianHybridControllerNode();

  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  virtual void update(void);
  virtual bool starting() { return c_.starting(); }

  void command();

  bool setToolFrame(robot_srvs::SetPoseStamped::Request &req,
                    robot_srvs::SetPoseStamped::Response &resp);

private:
  std::string name_;
  tf::TransformListener TF;
  CartesianHybridController c_;
  ros::Node *node_;
  robot_msgs::TaskFrameFormalism command_msg_;

  std::string task_frame_name_;

  unsigned int loop_count_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<robot_msgs::CartesianState> > pub_state_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > pub_tf_;
};

}
