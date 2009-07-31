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

#include "ros/ros.h"

#include "robot_msgs/PoseStamped.h"
#include "nav_robot_actions/MoveBaseState.h"

#include "move_base/MoveBaseAction.h"
#include "actionlib/client/action_client.h"
#include "action_translator/action_translator.h"
#include "robot_actions/action_runner.h"
#include "boost/thread.hpp"

using namespace robot_msgs;

move_base::MoveBaseGoal fromOldGoal(const PoseStamped& old_goal)
{
  move_base::MoveBaseGoal new_goal;
  new_goal.target_pose = old_goal;
  return new_goal;
}

PoseStamped fromActionFeedback(const move_base::MoveBaseFeedback& action_feedback)
{
  return action_feedback.cur_pose;
}

PoseStamped fromActionResult(const move_base::MoveBaseResult& action_result)
{
  return action_result.final_pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_action_translator");

  ros::NodeHandle n;

  action_translator::ActionTranslator<move_base::MoveBaseAction, PoseStamped, PoseStamped>
                              translator("move_base", &fromOldGoal, &fromActionFeedback, &fromActionResult);

  robot_actions::ActionRunner runner(10.0);
  runner.connect<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped>(translator);

  runner.run();

  ros::spin();

  return 0;
}
