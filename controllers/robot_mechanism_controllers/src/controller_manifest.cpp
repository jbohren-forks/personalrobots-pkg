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

#include "pluginlib/plugin_macros.h"
#include "mechanism_control/controller.h"

#include "robot_mechanism_controllers/cartesian_hybrid_controller.h"
#include "robot_mechanism_controllers/cartesian_pose_controller.h"
#include "robot_mechanism_controllers/cartesian_tff_controller.h"
#include "robot_mechanism_controllers/cartesian_trajectory_controller.h"
#include "robot_mechanism_controllers/cartesian_twist_controller.h"
#include "robot_mechanism_controllers/cartesian_twist_controller_ik.h"
#include "robot_mechanism_controllers/cartesian_wrench_controller.h"
#include "robot_mechanism_controllers/dynamic_loader_controller.h"
#include "robot_mechanism_controllers/endeffector_constraint_controller.h"
#include "robot_mechanism_controllers/joint_autotuner.h"
#include "robot_mechanism_controllers/joint_blind_calibration_controller.h"
#include "robot_mechanism_controllers/joint_calibration_controller.h"
#include "robot_mechanism_controllers/joint_chain_constraint_controller.h"
#include "robot_mechanism_controllers/joint_chain_sine_controller.h"
#include "robot_mechanism_controllers/joint_effort_controller.h"
#include "robot_mechanism_controllers/joint_inverse_dynamics_controller.h"
#include "robot_mechanism_controllers/joint_limit_calibration_controller.h"
#include "robot_mechanism_controllers/joint_pd_controller.h"
#include "robot_mechanism_controllers/joint_position_controller.h"
#include "robot_mechanism_controllers/joint_position_smoothing_controller.h"
#include "robot_mechanism_controllers/joint_ud_calibration_controller.h"
#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "robot_mechanism_controllers/probe.h"
#include "robot_mechanism_controllers/trigger_controller.h"

using namespace controller;

BEGIN_PLUGIN_LIST(Controller)

REGISTER_PLUGIN(JointEffortController)
REGISTER_PLUGIN(JointVelocityController)
REGISTER_PLUGIN(JointPositionController)

REGISTER_PLUGIN(CartesianWrenchController)
REGISTER_PLUGIN(CartesianTwistController)
REGISTER_PLUGIN(CartesianPoseController)

REGISTER_PLUGIN(JointUDCalibrationController)

REGISTER_PLUGIN(CartesianTrajectoryController)

REGISTER_PLUGIN(DynamicLoaderController)

REGISTER_PLUGIN(TriggerController);
REGISTER_PLUGIN(TriggerControllerNode);

REGISTER_PLUGIN(Probe)
REGISTER_PLUGIN(JointPDController)
REGISTER_PLUGIN(JointPDControllerNode)
REGISTER_PLUGIN(CartesianTwistControllerIk)
REGISTER_PLUGIN(CartesianTFFController)
REGISTER_PLUGIN(CartesianHybridController)
REGISTER_PLUGIN(CartesianHybridControllerNode)
REGISTER_PLUGIN(EndeffectorConstraintController)
REGISTER_PLUGIN(EndeffectorConstraintControllerNode)
REGISTER_PLUGIN(JointChainConstraintControllerNode)
REGISTER_PLUGIN(JointInverseDynamicsController);
REGISTER_PLUGIN(JointPositionSmoothController)
REGISTER_PLUGIN(JointPositionSmoothControllerNode)

REGISTER_PLUGIN(JointAutotuner)
REGISTER_PLUGIN(JointAutotunerNode)
REGISTER_PLUGIN(JointBlindCalibrationController)
REGISTER_PLUGIN(JointBlindCalibrationControllerNode)
REGISTER_PLUGIN(JointCalibrationControllerNode)
REGISTER_PLUGIN(JointChainSineController)
REGISTER_PLUGIN(JointLimitCalibrationControllerNode)

END_PLUGIN_LIST
