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

#include "experimental_controllers/pid_position_velocity_controller.h"
//#include "experimental_controllers/plug_controller.h"
#include "experimental_controllers/trajectory_controller.h"

#include "experimental_controllers/cartesian_tff_controller.h"

#include "experimental_controllers/cartesian_hybrid_controller.h"
#include "experimental_controllers/cartesian_twist_controller_ik.h"
#include "experimental_controllers/dynamic_loader_controller.h"
//#include "experimental_controllers/endeffector_constraint_controller.h"
#include "experimental_controllers/joint_autotuner.h"
#include "experimental_controllers/joint_blind_calibration_controller.h"
#include "experimental_controllers/joint_calibration_controller.h"
#include "experimental_controllers/joint_chain_constraint_controller.h"
#include "experimental_controllers/joint_chain_sine_controller.h"
#include "experimental_controllers/joint_inverse_dynamics_controller.h"
#include "experimental_controllers/joint_limit_calibration_controller.h"
#include "experimental_controllers/joint_position_smoothing_controller.h"
#include "experimental_controllers/probe.h"

#include "experimental_controllers/head_servoing_controller.h"
#include "experimental_controllers/joint_trajectory_controller.h"
#include "experimental_controllers/arm_trajectory_controller.h"


using namespace controller;


PLUGINLIB_REGISTER_CLASS(PIDPositionVelocityController, PIDPositionVelocityController, Controller)
//PLUGINLIB_REGISTER_CLASS(PlugController, PlugController, Controller)
//PLUGINLIB_REGISTER_CLASS(PlugControllerNode, PlugControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(TrajectoryController, TrajectoryController, Controller)

PLUGINLIB_REGISTER_CLASS(DynamicLoaderController, DynamicLoaderController, Controller)

PLUGINLIB_REGISTER_CLASS(Probe, Probe, Controller)
PLUGINLIB_REGISTER_CLASS(CartesianTFFController, CartesianTFFController, Controller)
PLUGINLIB_REGISTER_CLASS(CartesianHybridController, CartesianHybridController, Controller)
PLUGINLIB_REGISTER_CLASS(CartesianHybridControllerNode, CartesianHybridControllerNode, Controller)
//PLUGINLIB_REGISTER_CLASS(EndeffectorConstraintController, EndeffectorConstraintController, Controller)
//PLUGINLIB_REGISTER_CLASS(EndeffectorConstraintControllerNode, EndeffectorConstraintControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(JointChainConstraintControllerNode, JointChainConstraintControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(JointInverseDynamicsController, JointInverseDynamicsController, Controller);
PLUGINLIB_REGISTER_CLASS(JointPositionSmoothController, JointPositionSmoothController, Controller)
PLUGINLIB_REGISTER_CLASS(JointPositionSmoothControllerNode, JointPositionSmoothControllerNode, Controller)

PLUGINLIB_REGISTER_CLASS(JointAutotuner, JointAutotuner, Controller)
PLUGINLIB_REGISTER_CLASS(JointAutotunerNode, JointAutotunerNode, Controller)
PLUGINLIB_REGISTER_CLASS(JointBlindCalibrationController, JointBlindCalibrationController, Controller)
PLUGINLIB_REGISTER_CLASS(JointBlindCalibrationControllerNode,JointBlindCalibrationControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(JointCalibrationControllerNode,JointCalibrationControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(JointChainSineController,JointChainSineController, Controller)
PLUGINLIB_REGISTER_CLASS(JointLimitCalibrationControllerNode,JointLimitCalibrationControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(CartesianTwistControllerIk,CartesianTwistControllerIk, Controller)

PLUGINLIB_REGISTER_CLASS(HeadServoingController, HeadServoingController, Controller)
PLUGINLIB_REGISTER_CLASS(JointTrajectoryController,JointTrajectoryController, Controller)
PLUGINLIB_REGISTER_CLASS(ArmTrajectoryControllerNode,ArmTrajectoryControllerNode, Controller)


