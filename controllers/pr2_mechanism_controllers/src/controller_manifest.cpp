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

#include "pr2_mechanism_controllers/caster_calibration_controller.h"
#include "pr2_mechanism_controllers/caster_controller.h"
#include "pr2_mechanism_controllers/gripper_calibration_controller.h"
#include "pr2_mechanism_controllers/head_position_controller.h"
#include "pr2_mechanism_controllers/laser_scanner_traj_controller.h"
#include "pr2_mechanism_controllers/pr2_base_controller.h"
//#include "pr2_mechanism_controllers/pr2_gripper_controller.h"
#include "pr2_mechanism_controllers/pr2_odometry.h"
#include "pr2_mechanism_controllers/wrist_calibration_controller.h"


using namespace controller;


PLUGINLIB_REGISTER_CLASS(CasterCalibrationController, CasterCalibrationController, Controller)
PLUGINLIB_REGISTER_CLASS(CasterController, CasterController, Controller)
PLUGINLIB_REGISTER_CLASS(GripperCalibrationController, GripperCalibrationController, Controller)
PLUGINLIB_REGISTER_CLASS(GripperCalibrationControllerNode, GripperCalibrationControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(HeadPositionController, HeadPositionController, Controller)
PLUGINLIB_REGISTER_CLASS(LaserScannerTrajController, LaserScannerTrajController, Controller)
PLUGINLIB_REGISTER_CLASS(LaserScannerTrajControllerNode, LaserScannerTrajControllerNode, Controller)
PLUGINLIB_REGISTER_CLASS(Pr2BaseController, Pr2BaseController, Controller)
//PLUGINLIB_REGISTER_CLASS(Pr2GripperController, Pr2GripperController, Controller)
PLUGINLIB_REGISTER_CLASS(Pr2Odometry, Pr2Odometry, Controller)
PLUGINLIB_REGISTER_CLASS(WristCalibrationController, WristCalibrationController, Controller)
PLUGINLIB_REGISTER_CLASS(WristCalibrationControllerNode, WristCalibrationControllerNode, Controller)


