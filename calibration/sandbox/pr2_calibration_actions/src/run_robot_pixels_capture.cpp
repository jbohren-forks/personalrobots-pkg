/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include "pr2_calibration_actions/robot_pixels_capture.h"

using namespace pr2_calibration_actions;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pixel_capture_node");

  RobotPixelsConfig config;

  config.joint_states_config.joint_states_topic = "joint_states";
  config.joint_states_config.joint_names.resize(9);
  config.joint_states_config.joint_names[0] = "head_pan_joint";
  config.joint_states_config.joint_names[1] = "head_tilt_joint";
  config.joint_states_config.joint_names[2] = "r_shoulder_pan_joint";
  config.joint_states_config.joint_names[3] = "r_shoulder_lift_joint";
  config.joint_states_config.joint_names[4] = "r_upper_arm_roll_joint";
  config.joint_states_config.joint_names[5] = "r_elbow_flex_joint";
  config.joint_states_config.joint_names[6] = "r_forearm_roll_joint";
  config.joint_states_config.joint_names[7] = "r_wrist_flex_joint";
  config.joint_states_config.joint_names[8] = "r_wrist_roll_joint";

  config.joint_states_config.tolerances.resize(9);
  config.joint_states_config.tolerances[0] = 0.0;
  config.joint_states_config.tolerances[1] = 0.0;
  config.joint_states_config.tolerances[2] = 0.0;
  config.joint_states_config.tolerances[3] = 0.0;
  config.joint_states_config.tolerances[4] = 0.0;
  config.joint_states_config.tolerances[5] = 0.0;
  config.joint_states_config.tolerances[6] = 0.0;
  config.joint_states_config.tolerances[7] = 0.0;
  config.joint_states_config.tolerances[8] = 0.0;

  config.joint_states_config.padding = ros::Duration(1,0);
  config.joint_states_config.min_samples = 30;
  config.joint_states_config.cache_size = 450;

  // *************************************

  config.pixel_configs.resize(2);
  config.pixel_configs[0].pixel_topic = "/stereo/left/led";
  config.pixel_configs[0].image_topic = "/stereo/left/image_rect";
  config.pixel_configs[0].channel_name = "Left_Rect";
  config.pixel_configs[0].tolerance = 1.5;
  config.pixel_configs[0].padding = ros::Duration().fromSec(.5);
  config.pixel_configs[0].min_samples = 4;

  config.pixel_configs[1].pixel_topic = "/stereo/right/led";
  config.pixel_configs[1].image_topic = "/stereo/right/image_rect";
  config.pixel_configs[1].channel_name = "Right_Rect";
  config.pixel_configs[1].tolerance = 1.5;
  config.pixel_configs[1].padding = ros::Duration().fromSec(.5);
  config.pixel_configs[1].min_samples = 4;

  // *************************************

  config.joint_states_timeshift = ros::Duration().fromSec(.25);

  // *************************************

  config.pixel_timeshifts.resize(1);
  config.pixel_timeshifts[0].chan1 = 0;
  config.pixel_timeshifts[0].chan2 = 1;
  config.pixel_timeshifts[0].max_timeshift = ros::Duration(0,0);

  RobotPixelsCapture capture(config);

  ros::spin();

}


