/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ACTION_TRANSLATOR_ACTION_TRANSLATOR_H_
#define ACTION_TRANSLATOR_ACTION_TRANSLATOR_H_

#include "actionlib/client/simple_action_client.h"
#include "robot_actions/action.h"

namespace action_translator
{

template<class ActionSpec, class OldGoal, class OldFeedback>
class ActionTranslator : public robot_actions::Action<OldGoal, OldFeedback>
{
private:
  ACTION_DEFINITION(ActionSpec);
  typedef ActionTranslator<ActionSpec, OldGoal, OldFeedback> ActionTranslatorT;

public:
  typedef boost::function< Goal(const OldGoal&)> FromOldGoalFunc;
  typedef boost::function< OldFeedback(const Feedback&)> FromActionFeedbackFunc;
  typedef boost::function< OldFeedback(const Result&)> FromActionResultFunc;

  ActionTranslator(const std::string& new_action_name,
                   FromOldGoalFunc from_old_goal_func,
                   FromActionFeedbackFunc from_action_feedback_func = FromActionFeedbackFunc(),
                   FromActionResultFunc from_action_result_func     = FromActionResultFunc()) :
    robot_actions::Action<OldGoal, OldFeedback>(new_action_name + "_old"),
    ac_(new_action_name),
    from_old_goal_func_(from_old_goal_func),
    from_action_feedback_func_(from_action_feedback_func),
    from_action_result_func_(from_action_result_func)
  {


  }

  robot_actions::ResultStatus execute(const OldGoal& old_goal, OldFeedback& old_feedback)
  {
    ac_.stopTrackingGoal();

    old_feedback_ptr_ = &old_feedback;

    // Convert from the old goal type to the new goal type
    Goal new_goal = from_old_goal_func_(old_goal);

    // Send the goal to the action server
    ac_.sendGoal(new_goal,
                 boost::bind(&ActionTranslatorT::handleDone, this, _1, _2),
                 NULL,
                 boost::bind(&ActionTranslatorT::handleFeedback, this, _1));

    // Keep looping until the ActionServer is in a Done state

    ros::Duration sleep_dur = ros::Duration().fromSec(.5);
    bool sent_preempt = false;
    while (n_.ok() && ac_.getGoalState() != actionlib::SimpleGoalState::DONE)
    {
      if (!sent_preempt && robot_actions::Action<OldGoal, OldFeedback>::isPreemptRequested())
      {
        ROS_DEBUG("Sending Cancel Message to ActionServer");
        ac_.cancelGoal();
        sent_preempt = true;
      }
      sleep_dur.sleep();
    }

    if (!n_.ok())
      return robot_actions::ABORTED;

    // Copy the Result into the feedback, if this function is registered
    if (from_action_result_func_)
    {
      ResultConstPtr result = ac_.getResult();
      if (result)
        old_feedback = from_action_result_func_( *result ) ;
    }

    // Convert the TerminalState type into the old ResultStatus type
    switch (ac_.getTerminalState().state_)
    {
      case actionlib::TerminalState::RECALLED:
        return robot_actions::PREEMPTED;
      case actionlib::TerminalState::REJECTED:
        return robot_actions::ABORTED;
      case actionlib::TerminalState::PREEMPTED:
        return robot_actions::PREEMPTED;
      case actionlib::TerminalState::ABORTED:
        return robot_actions::ABORTED;
      case actionlib::TerminalState::SUCCEEDED:
        return robot_actions::SUCCESS;
      case actionlib::TerminalState::LOST:
        return robot_actions::ABORTED;
      default:
        ROS_ERROR("Got an unknown Terminal Status: [%u]", ac_.getTerminalState().state_);
        break;
    }
    return robot_actions::ABORTED;
  }

private:
  ros::NodeHandle n_;
  actionlib::SimpleActionClient<ActionSpec> ac_;
  FromOldGoalFunc from_old_goal_func_;
  FromActionFeedbackFunc from_action_feedback_func_;
  FromActionResultFunc from_action_result_func_;

  OldFeedback* old_feedback_ptr_;

  void handleDone(const actionlib::TerminalState& terminal_state, const ResultConstPtr& result)
  {

  }

  void handleFeedback(const FeedbackConstPtr& new_feedback)
  {
    // Copy the received feedback into the old feedback type
    if (from_action_feedback_func_)
      *old_feedback_ptr_ = from_action_feedback_func_(*new_feedback);
  }


};

}

#endif
