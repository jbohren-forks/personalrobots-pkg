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

#ifndef ENDEFFECTOR_TWIST_CONTEROLLER_H
#define ENDEFFECTOR_TWIST_CONTEROLLER_H

#include <vector>
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "ros/node.h"
#include "robot_msgs/Twist.h"
#include "misc_utils/subscription_guard.h"
#include "mechanism_model/controller.h"
#include "tf/transform_datatypes.h"
#include "misc_utils/advertised_service_guard.h"
#include "robot_mechanism_controllers/endeffector_wrench_controller.h"
#include "joy/Joy.h"

namespace controller {

class EndeffectorTwistController : public Controller
{
public:
  EndeffectorTwistController();
  ~EndeffectorTwistController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();

  // input of the controller
  KDL::Twist twist_desi_, twist_meas_;

  // output of the controller
  KDL::Wrench wrench_out_;


private:
  unsigned int  num_joints_, num_segments_;

  // kdl stuff for kinematics
  KDL::Chain             chain_;
  KDL::ChainFkSolverVel* jnt_to_twist_solver_;

  // to get joint positions, velocities, and to set joint torques
  std::vector<mechanism::JointState*> joints_; 

  // internal wrench controller
  EndeffectorWrenchController wrench_controller_;
};






class EndeffectorTwistControllerNode : public Controller
{
 public:
  EndeffectorTwistControllerNode() {};
  ~EndeffectorTwistControllerNode() {};
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();
  void command();

  // callback functions for joystick
  void joystick();
  
 private:
  EndeffectorTwistController controller_;
  SubscriptionGuard guard_command_;

  robot_msgs::Twist twist_msg_;
  joy::Joy joystick_msg_;
};

} // namespace


#endif
