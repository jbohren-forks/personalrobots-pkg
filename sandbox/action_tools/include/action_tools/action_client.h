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

#ifndef ACTION_TOOLS_ROBUST_ACTION_CLIENT_H_
#define ACTION_TOOLS_ROBUST_ACTION_CLIENT_H_

#include <boost/thread.hpp>
#include "action_tools/GoalStatus.h"
#include "action_tools/Preempt.h"
#include "action_tools/EnclosureDeleter.h"

#define setState(next_state) \
{ \
  ROS_DEBUG("Setting ClientState to " #next_state);\
  client_state_ = next_state;\
}

namespace action_tools
{


namespace TerminalStatuses
{
  enum TerminalStatus  { REJECTED, PREEMPTED, SUCCEEDED, ABORTED, TIMED_OUT, IGNORED, LOST } ;
}

//! Horrible hack until ROS Supports this (ROS Trac #1387)
class OneShotTimer
{
public:
  OneShotTimer() : active_(false)  { }

  void cb(const ros::TimerEvent& e)
  {
    if (active_)
    {
      active_ = false;

      if (callback_)
        callback_(e);
      else
        ROS_ERROR("Got a NULL Timer OneShotTimer Callback");
    }
  }

  boost::function<void (const ros::TimerEvent& e)> getCb()
  {
    return boost::bind(&OneShotTimer::cb, this, _1);
  }

  void registerOneShotCb(boost::function<void (const ros::TimerEvent& e)> callback)
  {
    callback_ = callback;
  }

  void stop()
  {
    //timer_.stop();
    active_ = false;
  }

  const ros::Timer& operator=(const ros::Timer& rhs)
  {
    active_ = true;
    timer_ = rhs;
    return timer_;
  }
private:
  ros::Timer timer_;
  bool active_;
  boost::function<void (const ros::TimerEvent& e)> callback_;
};

template <class ActionGoal, class Goal, class ActionResult, class Result>
class ActionClient
{
public:
  typedef boost::function<void (const TerminalStatuses::TerminalStatus&, const boost::shared_ptr<const Result>&)> CompletionCallback;
  typedef ActionClient<ActionGoal, Goal, ActionResult, Result> ActionClientT;
  typedef boost::function<void (void)> FilledCompletionCallback;
  typedef boost::shared_ptr<const Result> ResultConstPtr;
  //typedef boost::function<void (const ros::TimerEvent& e)> AckTimeoutCallback;

  ActionClient(std::string name, ros::NodeHandle nh = ros::NodeHandle(), bool expecting_result=false ) : nh_(nh, name)
  {
    // Initialize all One Shot Timers
    ack_timer_.registerOneShotCb(boost::bind(&ActionClientT::ackTimeoutCallback, this, _1));
    runtime_timer_.registerOneShotCb(boost::bind(&ActionClientT::runtimeTimeoutCallback, this, _1));
    wait_for_preempted_timer_.registerOneShotCb(boost::bind(&ActionClientT::waitForPreemptedTimeoutCallback, this, _1));

    setState(IDLE);
    expecting_result_ = expecting_result;

    goal_pub_    = nh_.advertise<ActionGoal> ("goal", 1);
    preempt_pub_ = nh_.advertise<Preempt> ("preempt", 1);

    status_sub_  = nh_.subscribe("status", 1, &ActionClientT::statusCallback, this);
    //result_sub_  = nh_.subscribe("result", 1, &ActionClient<ActionGoal, Goal, ActionResult, Result>::result_callback, this);
  }

  void execute(const Goal& goal, CompletionCallback completion_callback,
               const ros::Duration& runtime_timeout            = ros::Duration(0,0),
               const ros::Duration& ack_timeout                = ros::Duration(5,0),
               const ros::Duration& wait_for_preempted_timeout = ros::Duration(5,0),
               const ros::Duration& comm_sync_timeout          = ros::Duration(5,0))
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    completion_callback_ = completion_callback;
    cur_goal_.header.stamp = ros::Time::now();
    cur_goal_.goal_id.id  = cur_goal_.header.stamp;
    cur_goal_.goal = goal;
    got_terminal_status_ = false;
    result_.reset();
    goal_pub_.publish(cur_goal_);
    setState(WAITING_FOR_ACK);
    runtime_timeout_ = runtime_timeout;
    wait_for_preempted_timeout_ = wait_for_preempted_timeout;
    comm_sync_timeout_ = comm_sync_timeout;


    // don't set an ACK timeout for the special case: duration==0
    if (ack_timeout == ros::Duration(0,0))
      ROS_DEBUG("Not setting a timeout for ACK");
    else
    {
      // Set/reset the timeout for WAITING_FOR_GOAL_ACK
      ROS_DEBUG("Starting the [%.2fs] timer for ACK timeout callback", ack_timeout.toSec());
      ack_timer_ = nh_.createTimer(ack_timeout, ack_timer_.getCb());
    }
  }

private:

  boost::recursive_mutex client_state_mutex_;
  // ***** Lockset for client_state_mutex_ *****

  enum ClientState {SERVER_INACTIVE,IDLE, WAITING_FOR_ACK, PURSUING_GOAL, WAITING_FOR_PREEMPTED, WAITING_FOR_TERMINAL_STATE, WAITING_FOR_RESULT };
  ClientState client_state_;
  bool server_active_;
  ActionGoal cur_goal_;
  uint8_t terminal_status_;
  bool got_terminal_status_;
  ros::Duration runtime_timeout_;            // Maximum time we're willing to stay in {PURSUING_GOAL, WAITING_FOR_TERMINAL_STATE, WAITING_FOR_RESULT} before Preempting
  ros::Duration wait_for_preempted_timeout_; // Maximum time we're willing to stay in WAITING_FOR_PREEMPTED until we release the goal as TIMED_OUT
  ros::Duration comm_sync_timeout_;          // Maximum time we're willing to stay in WAITING_FOR_TERMINAL_STATE or WAITING_FOR_RESULT until we release the goal as TIMED_OUT
  bool expecting_result_;
  boost::shared_ptr<const ActionResult> result_;

  // *******************************************

  //boost::recursive_mutex result_mutex_;

  // Various Timers
  OneShotTimer ack_timer_;
  OneShotTimer runtime_timer_;
  OneShotTimer wait_for_preempted_timer_;

  CompletionCallback completion_callback_;

  ros::NodeHandle nh_;
  ros::Publisher goal_pub_;
  ros::Publisher preempt_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  void ackTimeoutCallback(const ros::TimerEvent& e)
  {
    FilledCompletionCallback callback;
    {
      boost::mutex::scoped_lock(client_state_mutex_);
      if ( client_state_ == WAITING_FOR_ACK )
      {
        ROS_WARN("Timed out waiting for ACK");
        callback = boost::bind(completion_callback_, TerminalStatuses::IGNORED, ResultConstPtr());
        setState(IDLE);
      }
    }

    if (callback)
      callback();
  }

  void runtimeTimeoutCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    if ( client_state_ == PURSUING_GOAL )
    {
      ROS_WARN("Timed out waiting to finish PURSUING_GOAL");
      preemptGoal();
    }
  }

  void waitForPreemptedTimeoutCallback(const ros::TimerEvent& e)
  {
    FilledCompletionCallback callback;
    {
      boost::mutex::scoped_lock(client_state_mutex_);
      if ( client_state_ == WAITING_FOR_PREEMPTED )
      {
        ROS_WARN("Timed out waiting to finish WAITING_FOR_PREEMPTED");
        setState(IDLE);
        callback = boost::bind(completion_callback_, TerminalStatuses::TIMED_OUT, boost::shared_ptr<Result>());
      }
    }

    if (callback)
      callback();
  }

  void preemptGoal()
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    if (client_state_ == PURSUING_GOAL || client_state_ == WAITING_FOR_ACK)
    {
      Preempt preempt;
      preempt.goal_id = cur_goal_.goal_id;
      preempt.header.stamp = cur_goal_.goal_id.id;
      preempt_pub_.publish(preempt);
      if (wait_for_preempted_timeout_ != ros::Duration(0,0))
      {
        ROS_DEBUG("Starting the [%.2fs] timer for the WAIT_FOR_PREEMPTED timeout", wait_for_preempted_timeout_.toSec());
        wait_for_preempted_timer_ = nh_.createTimer(wait_for_preempted_timeout_, wait_for_preempted_timer_.getCb());
      }
      else
        ROS_DEBUG("Infinte timeout for WAIT_FOR_PREEMPTED timeout");
      setState(WAITING_FOR_PREEMPTED);
    }
    else
      ROS_DEBUG("Not in a preemptable state (ClientState=%u)", client_state_);
  }

  void statusCallback(const GoalStatusConstPtr& msg)
  {
    // ***** ADD STATUS PING ****
    //gotStatusPing();

    // Don't do any processing on idle messages
    if (msg->status == GoalStatus::IDLE)
      return;

    // Don't care about status if we're not even trying for a goal
    if (client_state_ == IDLE || client_state_ == SERVER_INACTIVE )
      return;

    // Used to call callback outside the locks
    FilledCompletionCallback callback;
    {
      boost::mutex::scoped_lock(client_state_mutex_);
      if ( isFutureGoal(msg->goal_id) )
      {
        if (msg->status != GoalStatus::REJECTED)
        {
          ROS_DEBUG("Saw a future goal. Therefore, our goal somehow got lost during execution");
          setState(IDLE);
          // Save the callback that we want to call, and call it later (outside the lock).
          callback = boost::bind(completion_callback_, TerminalStatuses::LOST, boost::shared_ptr<Result>());
        }
        else
        {
          ROS_DEBUG("Saw a future goal be Rejected. Ignoring it, since we're ok with rejected future goals");
        }
      }
      else if( isCurrentGoal(msg->goal_id) )
      {
        if (client_state_ == WAITING_FOR_ACK)
        {
          switch (msg->status)
          {
            case GoalStatus::ACTIVE :
              ack_timer_.stop();
              setState(PURSUING_GOAL);
              if (runtime_timeout_ != ros::Duration(0,0))
              {
                ROS_DEBUG("Starting the [(%.2fs] timer for the PURSUING_GOAL timeout", runtime_timeout_.toSec());
                runtime_timer_ = nh_.createTimer(runtime_timeout_, runtime_timer_.getCb());
              }
              else
                ROS_DEBUG("Infinte timeout for PURSUING_GOAL timeout");
              break;
            case GoalStatus::PREEMPTED :
            case GoalStatus::SUCCEEDED :
            case GoalStatus::ABORTED :
            case GoalStatus::REJECTED :
              callback = transitionToTerminalState(msg->status); break;
            default:
              ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
          }
        }
        else if (client_state_ == PURSUING_GOAL)
        {
          switch (msg->status)
          {
            case GoalStatus::ACTIVE :
              break;
            case GoalStatus::PREEMPTED :
            case GoalStatus::SUCCEEDED :
            case GoalStatus::ABORTED :
            case GoalStatus::REJECTED :
              runtime_timer_.stop();
              callback = transitionToTerminalState(msg->status); break;
            default:
              ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
          }
        }
        else if (client_state_ == WAITING_FOR_PREEMPTED)
        {
          switch (msg->status)
          {
            case GoalStatus::ACTIVE :
              break;
            case GoalStatus::PREEMPTED :
            case GoalStatus::SUCCEEDED :
            case GoalStatus::ABORTED :
            case GoalStatus::REJECTED :
              wait_for_preempted_timer_.stop();
              callback = transitionToTerminalState(msg->status); break;
            default:
              ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
          }
        }
        else if (client_state_ == WAITING_FOR_TERMINAL_STATE)
        {
          switch (msg->status)
          {
            case GoalStatus::PREEMPTED :
            case GoalStatus::SUCCEEDED :
            case GoalStatus::ABORTED :
            case GoalStatus::REJECTED :
              callback = transitionToTerminalState(msg->status); break;
            case GoalStatus::ACTIVE : break;
            default:
              ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
          }
        }
        else
        {
          ROS_DEBUG("In ClientState [%u]. Got goal status [%u], Ignoring it", client_state_, msg->status);
        }
      }
    }
    // Call the completion callback if we need to
    if (callback)
      callback();
  }

  bool isFutureGoal(const GoalID& goal_id)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    return goal_id.id > cur_goal_.goal_id.id;
  }

  bool isCurrentGoal(const GoalID& goal_id)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    return goal_id.id == cur_goal_.goal_id.id;
  }

  FilledCompletionCallback transitionToTerminalState(uint8_t status)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    got_terminal_status_ = true;
    terminal_status_ = status;

    if(expecting_result_ && !result_)
    {
      switch(status)
      {
        case GoalStatus::PREEMPTED:
        case GoalStatus::SUCCEEDED:
        case GoalStatus::ABORTED:
        case GoalStatus::REJECTED:
          setState(WAITING_FOR_RESULT);
          // *********** ADD TIMER HERE **************
          break;
        default:
          ROS_WARN("BUG: Tried to go to a terminal status without receiving a terminal status. [GoalStatus.status==%u]", status);
          break;
      }
    }
    else
    {
      ResultConstPtr unwrapped_result;
      if (expecting_result_ && result_)
      {
        EnclosureDeleter<const ActionResult> d(result_);
        unwrapped_result = ResultConstPtr(&(result_->result), d);
      }
      else
        unwrapped_result.reset(); // If they weren't expecting a result or we never got one, then don't give them one

      switch(status)
      {
        case GoalStatus::PREEMPTED:
          setState(IDLE);
          return boost::bind(completion_callback_, TerminalStatuses::PREEMPTED, unwrapped_result);
        case GoalStatus::SUCCEEDED:
          setState(IDLE);
          return boost::bind(completion_callback_, TerminalStatuses::SUCCEEDED, unwrapped_result);
        case GoalStatus::ABORTED:
          setState(IDLE);
          return boost::bind(completion_callback_, TerminalStatuses::ABORTED, unwrapped_result);
        case GoalStatus::REJECTED:
          setState(IDLE);
          return boost::bind(completion_callback_, TerminalStatuses::REJECTED, unwrapped_result);
        default:
          ROS_WARN("BUG: Tried to go to a terminal status without receiving a terminal status. [GoalStatus.status==%u]", status);
          break;
      }
    }
    return FilledCompletionCallback(); // Null callback
  }

};

}

#endif // ACTION_TOOLS_ACTION_CLIENT_H_
