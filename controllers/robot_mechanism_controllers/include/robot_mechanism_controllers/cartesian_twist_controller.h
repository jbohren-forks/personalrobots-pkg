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

/*
 * Author: Wim Meeussen
 */

#ifndef CARTESIAN_TWIST_CONTROLLER_H
#define CARTESIAN_TWIST_CONTROLLER_H

#include <vector>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <ros/node.h>
#include <robot_msgs/Twist.h>
#include <mechanism_model/controller.h>
#include <tf/transform_datatypes.h>
#include <robot_mechanism_controllers/cartesian_wrench_controller.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>

namespace controller {

class CartesianTwistController
{
public:
  CartesianTwistController();
  ~CartesianTwistController();

  bool init(mechanism::RobotState *robot, const std::string& root_name,
            const std::string& tip_name, const std::string& controller_name);
  bool starting();
  void update();

  // input of the controller
  KDL::Twist twist_desi_, twist_meas_;

private:
  // output of the controller
  KDL::Wrench wrench_out_;

  ros::Node* node_;
  std::string controller_name_;
  double last_time_, ff_trans_, ff_rot_;

  // pid controllers
  std::vector<control_toolbox::Pid>fb_pid_controller_;

  // robot description
  mechanism::RobotState *robot_state_;
  mechanism::Chain chain_;

  // kdl stuff for kinematics
  KDL::Chain             kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
  KDL::JntArrayVel       jnt_posvel_;

  // internal wrench controller
  CartesianWrenchController wrench_controller_;
};






class CartesianTwistControllerNode : public Controller
{
 public:
  CartesianTwistControllerNode();
  ~CartesianTwistControllerNode();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  bool starting();
  void update();
  void command();

 private:
  ros::Node* node_;
  std::string controller_name_;
  CartesianTwistController controller_;

  robot_msgs::Twist twist_msg_;
};

} // namespace


#endif
