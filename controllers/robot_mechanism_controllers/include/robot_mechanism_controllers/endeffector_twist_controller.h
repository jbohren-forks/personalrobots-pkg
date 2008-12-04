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
 * Example config:

 <controller type="CartesianEffortController" name="controller_name">
   <chain root="root_link" tip="tip_link" offset="0.3 0.1 0.2" />
 </controller>

 * The root is fixed, and all commands are specified in its coordinate
 * frame.
 *
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

namespace controller {

class EndeffectorTwistController : public Controller
{
public:
  EndeffectorTwistController();
  ~EndeffectorTwistController();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();

  KDL::Twist twist_desi_, twist_meas_;
  KDL::Wrench wrench_out_;


private:
  unsigned int print_counter_;

  KDL::Chain             chain_;
  KDL::ChainFkSolverVel* jnt_to_twist_solver_;
  unsigned int  num_joints_, num_segments_;

  std::vector<mechanism::JointState*> joints_;  // root to tip, 1 element smaller than links_   

  EndeffectorWrenchController wrench_controller_;
};






class EndeffectorTwistControllerNode : public Controller
{
 public:
  EndeffectorTwistControllerNode();
  ~EndeffectorTwistControllerNode();
  
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);
  void update();
  
  void command();
  void spacenavPos();
  void spacenavRot();
  
 private:
  EndeffectorTwistController controller_;
  SubscriptionGuard guard_command_;

  robot_msgs::Twist twist_msg_;
  std_msgs::Point spacenav_pos_msg_, spacenav_rot_msg_;
};

} // namespace


#endif
