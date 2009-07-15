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

#ifndef ACTION_TOOLS_ACTION_CLIENT_H_
#define ACTION_TOOLS_ACTION_CLIENT_H_

#include "action_tools/GoalStatus.h"
#include "action_tools/Preempt.h"

namespace action_tools
{


namespace TerminalStatuses
{
  enum TerminalStatus  { REJECTED, PREEMPTED, SUCCEEDED, ABORTED, TIMED_OUT, UNKNOWN_STATE } ;
}

template <class ActionGoal, class Goal, class ActionResult, class Result>
class ActionClient
{
public:
  typedef boost::function<void (const TerminalStatuses::TerminalStatus&, const Result&)> ResultCallback;

  ActionClient(std::string name, ros::NodeHandle nh = ros::NodeHandle()) : nh_(nh, name)
  {
    ROS_INFO("ClientState: Setting to IDLE");
    client_state_ = IDLE;

    goal_pub_    = nh_.advertise<ActionGoal> ("goal", 1);
    preempt_pub_ = nh_.advertise<Preempt> ("preempt", 1);

    status_sub_  = nh_.subscribe("status", 1, &ActionClient<ActionGoal, Goal, ActionResult, Result>::status_callback, this);
    result_sub_  = nh_.subscribe("result", 1, &ActionClient<ActionGoal, Goal, ActionResult, Result>::result_callback, this);
  }

  void execute(const Goal& goal, ResultCallback result_callback)
  {
    cur_goal_.goal_id.id = ros::Time::now();
    cur_goal_.goal = goal;
    goal_pub_.publish(cur_goal_);
    ROS_INFO("ClientState: Setting to PURSUING_GOAL");
    client_state_ = PURSUING_GOAL;
    result_callback_ = result_callback;
    cur_status_.status = GoalStatus::IDLE;
    cur_status_.goal_id.id = cur_goal_.goal_id.id;
  }

  Result getResult()
  {
    return latest_result_->result;
  }

private:
  enum ClientState {IDLE, PURSUING_GOAL, WAITING_FOR_RESULT, WAITING_FOR_TERMINAL_STATE };
  ClientState client_state_;
  ActionGoal cur_goal_;
  GoalStatus cur_status_;
  boost::shared_ptr<const ActionResult> latest_result_;

  ResultCallback result_callback_;

  ros::NodeHandle nh_;
  ros::Publisher goal_pub_;
  ros::Publisher preempt_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  void status_callback(const GoalStatusConstPtr& status)
  {
    // Check error condition: See if we're pursuing a goal in the future
    if (status->goal_id.id > cur_goal_.header.stamp)
    {
      if (status->status == GoalStatus::PREEMPTED ||
          status->status == GoalStatus::SUCCEEDED ||
          status->status == GoalStatus::ABORTED)
      {
        ROS_INFO("Action has moved on to a new goal");
      }
      else
      {
        ROS_WARN("Switched to a new goal without giving feedback");
      }
      ROS_INFO("ClientState: Setting to IDLE");
      client_state_ = IDLE;
      return;
    }

    if (status->goal_id.id == cur_goal_.goal_id.id)
    {
      // Check if we did transition to a different goal status
      if (cur_status_.status != status->status)
      {
        if (status->status == GoalStatus::PREEMPTED ||
            status->status == GoalStatus::SUCCEEDED ||
            status->status == GoalStatus::ABORTED)
        {
          ROS_INFO("In terminal state");
          if (client_state_ == PURSUING_GOAL || client_state_ == IDLE)
          {
            ROS_INFO("ClientState: Setting to WAITING_FOR_RESULT");
            client_state_ = WAITING_FOR_RESULT;
          }
          else if(client_state_ == WAITING_FOR_TERMINAL_STATE)
          {
            ROS_INFO("Calling Result Callback");
            result_callback_(goalStatusToTerminalStatus(*status), latest_result_->result);
            ROS_INFO("ClientState: Setting to IDLE");
            client_state_ = IDLE;
          }
        }
        else if (status->status     == GoalStatus::ACTIVE &&
                 cur_status_.status == GoalStatus::IDLE)
        {
          ROS_INFO("Detected goal transition from IDLE to ACTIVE");
          ROS_INFO("ClientState: Setting to PURSUING_GOAL");
          client_state_ = PURSUING_GOAL;
        }
        else
        {
          ROS_WARN("Detected a funncy transition: from (%u) to (%u)", cur_status_.status, status->status);
        }
        cur_status_ = *status;
      }
    }
  }

  void result_callback(const boost::shared_ptr<const ActionResult>& result)
  {
    ROS_INFO("Result Callback");
    if (cur_goal_.goal_id.id == result->goal_id.id)
    {
      if (client_state_ == PURSUING_GOAL ||
          client_state_ == IDLE)
      {
        latest_result_ = result;
        ROS_INFO("ClientState: Setting to WAITING_FOR_TERMINAL_STATE");
        client_state_ = WAITING_FOR_TERMINAL_STATE;
      }
      else if (client_state_ == WAITING_FOR_RESULT)
      {
        latest_result_ = result;
        ROS_INFO("Calling Result Callback");
        result_callback_(goalStatusToTerminalStatus(cur_status_), latest_result_->result);
        ROS_INFO("ClientState: Setting to IDLE");
        client_state_ = IDLE;
      }
    }
    else
      ROS_INFO("Got result for a different goal");
  }

  TerminalStatuses::TerminalStatus goalStatusToTerminalStatus(const GoalStatus& goal_status)
  {
    switch(goal_status.status)
    {
      case GoalStatus::IDLE:   ROS_WARN("IDLE is not terminal"); return TerminalStatuses::UNKNOWN_STATE;
      case GoalStatus::ACTIVE: ROS_WARN("ACTIVE is not terminal"); return TerminalStatuses::UNKNOWN_STATE;
      case GoalStatus::PREEMPTED: return TerminalStatuses::PREEMPTED;
      case GoalStatus::SUCCEEDED: return TerminalStatuses::SUCCEEDED;
      case GoalStatus::ABORTED:   return TerminalStatuses::ABORTED;
      default: ROS_WARN("Got a weird GoalStatus"); return TerminalStatuses::UNKNOWN_STATE;
    }
    return TerminalStatuses::UNKNOWN_STATE;
  }
};

}

#endif // ACTION_TOOLS_ACTION_CLIENT_H_
