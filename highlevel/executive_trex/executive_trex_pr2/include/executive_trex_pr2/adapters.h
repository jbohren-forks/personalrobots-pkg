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

/**
 * @file This file provides shared includes used for stubs and adapters
 *
 * @author Conor McGann
 */

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <robot_msgs/Pose.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/BatteryState.h>
#include <robot_msgs/PlugStow.h>
#include <robot_actions/action_runner.h>
#include <robot_actions/action.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <pr2_robot_actions/CheckPathState.h>
#include <pr2_robot_actions/NotifyDoorBlockedState.h>
#include <pr2_robot_actions/ShellCommandState.h>
#include <pr2_robot_actions/StopActionState.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <pr2_robot_actions/RechargeState.h>
#include <pr2_robot_actions/DetectPlugOnBaseState.h>
#include <pr2_robot_actions/MoveAndGraspPlugState.h>
#include <pr2_robot_actions/StowPlugState.h>
#include <pr2_robot_actions/SwitchControllers.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <pr2_robot_actions/DetectOutletState.h>
