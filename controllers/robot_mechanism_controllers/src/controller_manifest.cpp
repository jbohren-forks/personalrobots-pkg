/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "pluginlib/class_list_macros.h"
#include "controller_interface/controller.h"

#include "robot_mechanism_controllers/cartesian_pose_controller.h"
#include "robot_mechanism_controllers/cartesian_trajectory_controller.h"
#include "robot_mechanism_controllers/cartesian_twist_controller.h"
#include "robot_mechanism_controllers/cartesian_wrench_controller.h"
#include "robot_mechanism_controllers/joint_effort_controller.h"
#include "robot_mechanism_controllers/joint_pd_controller.h"
#include "robot_mechanism_controllers/joint_position_controller.h"
#include "robot_mechanism_controllers/joint_ud_calibration_controller.h"
#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "robot_mechanism_controllers/trigger_controller.h"

using namespace controller;


PLUGINLIB_REGISTER_CLASS(JointEffortController, JointEffortController, Controller)
PLUGINLIB_REGISTER_CLASS(JointVelocityController, JointVelocityController, Controller)
PLUGINLIB_REGISTER_CLASS(JointPositionController, JointPositionController, Controller)

PLUGINLIB_REGISTER_CLASS(CartesianWrenchController, CartesianWrenchController, Controller)
PLUGINLIB_REGISTER_CLASS(CartesianTwistController, CartesianTwistController, Controller)
PLUGINLIB_REGISTER_CLASS(CartesianPoseController, CartesianPoseController, Controller)

PLUGINLIB_REGISTER_CLASS(JointUDCalibrationController, JointUDCalibrationController, Controller)

PLUGINLIB_REGISTER_CLASS(CartesianTrajectoryController, CartesianTrajectoryController, Controller)

PLUGINLIB_REGISTER_CLASS(TriggerController, TriggerController, Controller)
PLUGINLIB_REGISTER_CLASS(TriggerControllerNode, TriggerControllerNode, Controller)

PLUGINLIB_REGISTER_CLASS(JointPDController, JointPDController, Controller)
PLUGINLIB_REGISTER_CLASS(JointPDControllerNode, JointPDControllerNode, Controller)

