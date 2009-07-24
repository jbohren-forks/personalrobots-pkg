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

#include <list>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "action_tools/one_shot_timer.h"
#include "action_tools/goal_id_generator.h"
#include "action_tools/client_goal_status.h"
#include "action_tools/one_shot_timer.h"
#include "action_tools/EnclosureDeleter.h"

// Messages
//#include "action_tools/ActionHeader.h"
#include "action_tools/GoalStatusArray.h"

namespace action_tools
{

//! \todo figure out why I get compile errors trying to use boost::mutex::scoped_lock()
class ScopedLock
{
public:
  ScopedLock(boost::recursive_mutex& mutex) : mutex_(mutex)
  {
    mutex_.lock();
  }
  ~ScopedLock()
  {
    mutex_.unlock();
  }
private:
  boost::recursive_mutex& mutex_;

};


template<class ActionGoal, class Goal, class ActionResult, class Result, class ActionFeedback, class Feedback>
class ActionClient
{
public:
  // Need forward declaration for typedefs
  class GoalHandle;

  typedef ActionClient<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback> ActionClientT;
  typedef boost::shared_ptr<const ActionGoal> ActionGoalConstPtr;
  typedef boost::shared_ptr<const Goal> GoalConstPtr;

  typedef boost::shared_ptr<const ActionResult> ActionResultConstPtr;
  typedef boost::shared_ptr<const Result> ResultConstPtr;

  typedef boost::shared_ptr<const ActionFeedback> ActionFeedbackConstPtr;
  typedef boost::shared_ptr<const Feedback> FeedbackConstPtr;

  typedef boost::function<void (const GoalHandle&) > CompletionCallback;

  class GoalManager
  {
    public:
      //! \brief Builds the goal manager and then sends the goal over the wire
      GoalManager(const GoalID& goal_id, const Goal& goal, CompletionCallback cb, ActionClientT* ac, const ros::Duration& runtime_timeout);
      void preemptGoal();


    private:
      enum CommState {WAITING_FOR_ACK, PENDING, PURSUING_GOAL, WAITING_FOR_PREEMPTED, WAITING_FOR_RESULT, DONE};
      std::string commStateToString(const CommState& state)
      {
        switch (state)
        {
          case WAITING_FOR_ACK:
            return "WAITING_FOR_ACK";
          case PENDING:
            return "PENDING";
          case PURSUING_GOAL:
            return "PURSUING_GOAL";
          case WAITING_FOR_PREEMPTED:
            return "PREEMPTED";
          case WAITING_FOR_RESULT:
            return "WAITING_FOR_RESULT";
          case DONE:
            return "DONE";
          default:
            ROS_ERROR("Trying to lookup unknown CommState");
            break;
        }
        return "BUG-UNKNOWN";
      }

      void setCommState(const CommState& state);
      void setClientGoalStatus(const ClientGoalStatus::StateEnum& next_status_enum);
      void setClientGoalStatus(const ClientGoalStatus& next_status);
      void updateStatus(const GoalStatusArrayConstPtr& status_array);
      void updateFeedback(const ActionFeedbackConstPtr& feedback);
      void updateResult(const ActionResultConstPtr& result);

      void startWaitingForAck();
      void startPending();
      void startPursuingGoal();
      void startWaitingForPreempted();
      void startWaitingForResult(const ClientGoalStatus& next_client_goal_status);
      void startDone(const ActionResultConstPtr& result);
      void processLost();               // No explicit state transition, but looks like a state transition

      void finishWaitingForAck();
      void finishPending();
      void finishPursuingGoal();
      void finishWaitingForPreempted();
      void finishWaitingForResult();

      void waitingForAckTimeoutCallback(const ros::TimerEvent& e);
      void runtimeTimeoutCallback(const ros::TimerEvent& e);
      void waitingForPreemptedTimeoutCallback(const ros::TimerEvent& e);
      void waitingForResultTimeoutCallback(const ros::TimerEvent& e);

      const GoalStatus* findGoalStatus(const std::vector<GoalStatus>& status_vec);


      //boost::mutex mutex_;
      ClientGoalStatus client_goal_status_;
      CommState comm_state_;                        //!< The internal state if the communication protocol
      GoalID goal_id_;                              //!< ID associated with this goal
      ResultConstPtr result_;

      ActionClientT* ac_;                           //!< The action client we'll use for sending msgs over the wire
      boost::weak_ptr<void> handle_tracker_;        //!< Refcounts the # of goalsHandles for this GoalManager
      typename std::list<boost::shared_ptr<GoalManager> >::iterator it_; //!< Needed to construct a GoalHandle later

      // All the various state timers
      OneShotTimer  waiting_for_ack_timer_;       //!< Tracks timeout in WAITING_FOR_ACK state
      OneShotTimer  runtime_timer_;               //!< Tracks timeout in PURSUING_GOAL state
      OneShotTimer  waiting_for_preempted_timer_; //!< Tracks timeout in WAITING_FOR_PREEMPTED state
      OneShotTimer  waiting_for_result_timer_;    //!< Tracks timeout in WAITING_FOR_RESULT or WAITING_FOR_TERMINAL_STATE
      ros::Duration runtime_timeout_;

      CompletionCallback cb_;

      friend class ActionClient;
  };

  typedef boost::shared_ptr<GoalManager> GoalManagerPtr;

  // Used to clean up the GoalManager list in the ActionClient, once there are no more goal handles pointing to the status
  class HandleTrackerDeleter
  {
    public:
      HandleTrackerDeleter(ActionClientT* ac, typename std::list<GoalManagerPtr>::iterator it) : it_(it), ac_(ac)
      {  }

      void operator() (void* ptr)
      {
        ROS_DEBUG("About to delete a GoalManager");
        ScopedLock(ac_->manager_list_mutex_);
        ac_->manager_list_.erase(it_);
      }

    private:
      typename std::list<GoalManagerPtr>::iterator it_;
      ActionClientT* ac_;
  };

  // Object provided to user to make queries against a specific goal
  class GoalHandle
  {
    private:
      GoalHandle(typename std::list<GoalManagerPtr>::iterator it)
       : status_it_(it), handle_tracker_((*it)->handle_tracker_.lock())
      {  }
    public:

      ClientGoalStatus getStatus()
      {
        boost::mutex::scoped_lock(status_it_->mutex_);
        return (*status_it_)->getClientGoalState_();
      }

    private:
      typename std::list<GoalManagerPtr>::iterator status_it_;
      boost::shared_ptr<void> handle_tracker_;

      friend class ActionClient;
  };

  ActionClient(const std::string& name) : n_(name), id_generator_(name)
  {
    initClient();
  }

  ActionClient(const ros::NodeHandle& n, const std::string& name) : n_(n, name), id_generator_(name)
  {
    initClient();
  }

  void initClient()
  {
    goal_pub_ = n_.advertise<ActionGoal>("goal", 1);
    status_sub_   = n_.subscribe("status",   1, &ActionClientT::statusCb, this);
    feedback_sub_ = n_.subscribe("feedback", 1, &ActionClientT::feedbackCb, this);
    result_sub_   = n_.subscribe("result",   1, &ActionClientT::resultCb, this);
  }

  GoalHandle sendGoal(const Goal& goal,
                      CompletionCallback cb = CompletionCallback(),
                      ros::Duration timeout = ros::Duration(0,0) )
  {
    GoalID goal_id = id_generator_.generateID();

    // Add a goal manager to our list of managers

    GoalManagerPtr cur_manager = GoalManagerPtr(new GoalManager(goal_id, goal, cb, this, timeout));

    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.insert(manager_list_.end(), cur_manager );

    // Tell the goal manager where it is in the list
    (*it)->it_ = it;

    // Create a custom deallocater to eventually destroy the GoalManager, once
    //   we no longer have any GoalHandles in scope
    HandleTrackerDeleter d(this, it);
    boost::shared_ptr<void> handle_tracker((void*) NULL, d);
    (*it)->handle_tracker_ = handle_tracker;

    GoalHandle gh = GoalHandle(it);//, this);

    return gh;
  }

private:
  ros::NodeHandle n_;

  ros::Subscriber feedback_sub_;
  ros::Publisher  goal_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  boost::recursive_mutex manager_list_mutex_;
  std::list<GoalManagerPtr > manager_list_;

  GoalIDGenerator id_generator_;

  // Timer durations
  ros::Duration waiting_for_ack_timeout_;       // Maximum time we're willing to stay in WAITING_FOR_ACK
  ros::Duration runtime_timeout_;               // Maximum time we're willing to stay in {PURSUING_GOAL} before Preempting
  ros::Duration waiting_for_preempted_timeout_; // Maximum time we're willing to stay in WAITING_FOR_PREEMPTED until we release the goal as TIMED_OUT
  ros::Duration waiting_for_result_timeout_;    // Maximum time we're willing to stay in WAITING_FOR_RESULT until we release the goal as TIMED_OUT

  // *************** Implementation ***************

  void statusCb(const GoalStatusArrayConstPtr& status_array)
  {
    boost::mutex::scoped_lock(manager_list_mutex_);
    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.begin();
    while(it != manager_list_.end())
    {
      (*it)->updateStatus(status_array);
      ++it;
    }
  }

  void feedbackCb(const ActionFeedbackConstPtr& feedback)
  {
    boost::mutex::scoped_lock(manager_list_mutex_);
    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.begin();
    while(it != manager_list_.end())
    {
      (*it)->updateFeedback(feedback);
      ++it;
    }
  }

  void resultCb(const ActionResultConstPtr& result)
  {
    boost::mutex::scoped_lock(manager_list_mutex_);
    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.begin();
    while(it != manager_list_.end())
    {
      (*it)->updateResult(result);
      ++it;
    }
  }

};

#define ActionClientTemplate \
  template<class ActionGoal, class Goal, class ActionResult, class Result, class ActionFeedback, class Feedback>
#define ActionClientPrefix \
  ActionClient<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>

ActionClientTemplate
ActionClientPrefix::GoalManager::GoalManager(const GoalID& goal_id, const Goal& goal, CompletionCallback cb,
                                             ActionClientT* ac, const ros::Duration& runtime_timeout)
 : client_goal_status_(ClientGoalStatus::PENDING),
   comm_state_(WAITING_FOR_ACK),
   goal_id_(goal_id),
   ac_(ac),
   runtime_timeout_(runtime_timeout),
   cb_(cb)
{
  setCommState(WAITING_FOR_ACK);

  waiting_for_ack_timer_.registerOneShotCb(      boost::bind(&ActionClientPrefix::GoalManager::waitingForAckTimeoutCallback, this, _1));
  runtime_timer_.registerOneShotCb(              boost::bind(&ActionClientPrefix::GoalManager::runtimeTimeoutCallback, this, _1));
  waiting_for_preempted_timer_.registerOneShotCb(boost::bind(&ActionClientPrefix::GoalManager::waitingForPreemptedTimeoutCallback, this, _1));
  waiting_for_result_timer_.registerOneShotCb(   boost::bind(&ActionClientPrefix::GoalManager::waitingForResultTimeoutCallback, this, _1));

  boost::shared_ptr<ActionGoal> action_goal(new ActionGoal);
  action_goal->goal_id = goal_id;
  action_goal->request_type = ActionGoal::GOAL_REQUEST;
  action_goal->goal = goal;
  ac_->goal_pub_.publish(action_goal);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::preemptGoal()
{
  ActionGoal preempt_msg;
  preempt_msg.request_type = ActionGoal::PREEMPT_REQUEST;
  preempt_msg.goal_id = goal_id_;

  switch (comm_state_)
  {
    case WAITING_FOR_ACK:       finishWaitingForAck(); break;
    case PENDING:               finishPending(); break;
    case PURSUING_GOAL:         ROS_DEBUG("in preempt goal"); finishPursuingGoal(); break;
    case WAITING_FOR_PREEMPTED: break;
    case WAITING_FOR_RESULT:    finishWaitingForResult(); break;
    case DONE:                  break;
    default: ROS_ERROR("BUG: In a funny state"); break;
  }

  startWaitingForPreempted();
  ac_->goal_pub_.publish(preempt_msg);
}

ActionClientTemplate
const GoalStatus* ActionClientPrefix::GoalManager::findGoalStatus(const std::vector<GoalStatus>& status_vec)
{
  for (unsigned int i=0; i<status_vec.size(); i++)
    if (status_vec[i].goal_id.id == goal_id_.id)
      return &status_vec[i];
  return NULL;
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::updateStatus(const GoalStatusArrayConstPtr& status_array)
{
  ScopedLock(ac_->manager_list_mutex_);

  switch( comm_state_ )
  {
    case WAITING_FOR_ACK :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        finishWaitingForAck();
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            startPending();break;
          case GoalStatus::ACTIVE :
            startPursuingGoal();break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            startWaitingForResult(ClientGoalStatus(*goal_status)); break;
          default:
            ROS_ERROR("BUG: Got an unknown status from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      break;
    }
    case PENDING :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            break;
          case GoalStatus::ACTIVE :
            finishPending();
            startPursuingGoal(); break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            finishPending();
            startWaitingForResult(ClientGoalStatus(*goal_status)); break;
          default:
            ROS_ERROR("BUG: Got an unknown goal status from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      else
        processLost();
      break;
    }
    case PURSUING_GOAL :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            ROS_ERROR("Invalid transition from PURSUING_GOAL to PENDING"); break;
          case GoalStatus::ACTIVE :
            break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
            ROS_DEBUG("In status update");
            finishPursuingGoal();
            startWaitingForResult(ClientGoalStatus(*goal_status)); break;
          case GoalStatus::REJECTED :
            ROS_ERROR("Invalid transition from PURSUING_GOAL to REJECTED"); break;
          default:
            ROS_ERROR("BUG: Got an unknown goal status from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      else
        processLost();
      break;
    }
    case WAITING_FOR_PREEMPTED :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            ROS_ERROR("Invalid transition from WAITING_FOR_PREEMPTED to PENDING"); break;
          case GoalStatus::ACTIVE :
            break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            finishWaitingForPreempted();
            startWaitingForResult(ClientGoalStatus(*goal_status)); break;
          default:
            ROS_ERROR("BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      else
        processLost();
      break;
    }
    case WAITING_FOR_RESULT :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            ROS_ERROR("Invalid transition from WAITING_FOR_RESULT to PENDING"); break;
          case GoalStatus::ACTIVE :
            ROS_ERROR("Invalid transition from WAITING_FOR_RESULT to ACTIVE"); break;
          case GoalStatus::PREEMPTED :
            if (client_goal_status_ != ClientGoalStatus::PREEMPTED)
              ROS_ERROR("Got GoalStatus [PREEMPTED], but we're in ClientGoalState [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::SUCCEEDED :
            if (client_goal_status_ != ClientGoalStatus::SUCCEEDED)
              ROS_ERROR("Got GoalStatus==[SUCCEEDED], but we're in ClientGoalState==[%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::ABORTED :
            if (client_goal_status_ != ClientGoalStatus::ABORTED)
              ROS_ERROR("Got GoalStatus==[ABORTED], but we're in ClientGoalState==[%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::REJECTED :
            ROS_ERROR("Invalid Transition from WAITING_FOR_RESUT to REJECTED"); break;
          default:
            ROS_ERROR("BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      // No processLost() call here, since it's possible that the ActionServer stopped sending status,
      //   but we're still waiting for the result to come over the wire.
      break;
    }
    case DONE :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            ROS_ERROR("Invalid transition from WAITING_FOR_RESULT to PENDING"); break;
          case GoalStatus::ACTIVE :
            ROS_ERROR("Invalid transition from WAITING_FOR_RESULT to ACTIVE"); break;
          case GoalStatus::PREEMPTED :
            if (client_goal_status_ != ClientGoalStatus::PREEMPTED)
              ROS_ERROR("Got GoalStatus [PREEMPTED], but we're in ClientGoalStatus [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::SUCCEEDED :
            if (client_goal_status_ != ClientGoalStatus::SUCCEEDED)
              ROS_ERROR("Got GoalStatus [SUCCEEDED], but we're in ClientGoalStatus [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::ABORTED :
            if (client_goal_status_ != ClientGoalStatus::ABORTED)
              ROS_ERROR("Got GoalStatus [ABORTED], but we're in ClientGoalStatus [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::REJECTED :
            ROS_ERROR("Invalid transition from DONE to REJECTED"); break;
          default:
            ROS_ERROR("BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      break;
    }
    default :
      ROS_ERROR("BUG: Unknown CommState. comm_state_=%u", comm_state_);
  }
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::updateFeedback(const ActionFeedbackConstPtr& feedback)
{
  ROS_INFO("Need to still implement Feeback...");
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::updateResult(const ActionResultConstPtr& result)
{
  // {WAITING_FOR_ACK, PENDING, PURSUING_GOAL, WAITING_FOR_PREEMPTED, WAITING_FOR_RESULT, DONE};

  ROS_DEBUG("Got a result");
  ScopedLock(ac_->manager_list_mutex_);

  // Check if this result is for our goal
  if (goal_id_.id != result->status.goal_id.id)
    return;

  switch( comm_state_ )
  {
    case WAITING_FOR_ACK:
    case PENDING:
    case PURSUING_GOAL:
    case WAITING_FOR_PREEMPTED:
    case WAITING_FOR_RESULT:
      // Do cleanup for exiting state
      switch (comm_state_)
      {
        case WAITING_FOR_ACK:       finishWaitingForAck(); break;
        case PENDING:               finishPending(); break;
        case PURSUING_GOAL:         finishPursuingGoal(); break;
        case WAITING_FOR_PREEMPTED: finishWaitingForPreempted(); break;
        case WAITING_FOR_RESULT:    finishWaitingForResult(); break;
        default: ROS_ERROR("BUG: In a funny state"); break;
      }

      switch (result->status.status)
      {
        case GoalStatus::PENDING:
          ROS_ERROR("Got a Result with a PENDING GoalStatus"); break;
        case GoalStatus::ACTIVE:
          ROS_ERROR("Got a Result with an ACTIVE GoalStatus"); break;
        case GoalStatus::PREEMPTED:
        case GoalStatus::SUCCEEDED:
        case GoalStatus::ABORTED:
        case GoalStatus::REJECTED:
          setClientGoalStatus(ClientGoalStatus(result->status));
          startDone(result); break;
        default:
          ROS_ERROR("BUG: Got an unknown status from the ActionServer. status = %u", result->status.status); break;
      }
      break;
    case DONE:
      ROS_ERROR("Got a Result when this goal is already DONE"); break;
    default:
      ROS_ERROR("BUG: Unknown CommState. comm_state_=%u", comm_state_); break;
  }
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::setClientGoalStatus(const ClientGoalStatus::StateEnum& next_status_enum)
{
  setClientGoalStatus(ClientGoalStatus(next_status_enum));
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::setClientGoalStatus(const ClientGoalStatus& next_status)
{
  ROS_DEBUG("Transitioning ClientGoalStatus from [%s] to [%s]",
            client_goal_status_.toString().c_str(), next_status.toString().c_str());
  client_goal_status_ = next_status;
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::setCommState(const CommState& next_state)
{
  ROS_DEBUG("Transitioning CommState from %s to %s", commStateToString(comm_state_).c_str(), commStateToString(next_state).c_str());
  comm_state_ = next_state;
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startWaitingForAck()
{
  if (ac_->waiting_for_ack_timeout_ != ros::Duration(0,0))
  {
    ROS_DEBUG("Starting [%.2fs] timer for the WaitForAck timeout", ac_->waiting_for_ack_timeout_.toSec());
    waiting_for_ack_timer_ = ac_->n_.createTimer(ac_->wait_for_ack_timeout_, waiting_for_ack_timer_.getCb());
  }
  else
    ROS_DEBUG("Infinite WaitForAck timeout");
  setCommState(WAITING_FOR_ACK);
  setClientGoalStatus(ClientGoalStatus::PENDING);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::waitingForAckTimeoutCallback(const ros::TimerEvent& e)
{
  ROS_DEBUG("WaitingForAckTimer Timed out");
  processLost();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishWaitingForAck()
{
  ROS_DEBUG("Stopping WaitForAck timer");
  waiting_for_ack_timer_.stop();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startPending()
{
  setCommState(PENDING);
  setClientGoalStatus(ClientGoalStatus::PENDING);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishPending()
{

}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startPursuingGoal()
{
  if (runtime_timeout_ != ros::Duration(0,0))
  {
    ROS_DEBUG("Starting [%.2fs] timer for the Runtime timeout", runtime_timeout_.toSec());
    runtime_timer_ = ac_->n_.createTimer(runtime_timeout_, runtime_timer_.getCb());
  }
  else
    ROS_DEBUG("Infinite Runtime timeout");

  setCommState(PURSUING_GOAL);
  setClientGoalStatus(ClientGoalStatus::ACTIVE);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::runtimeTimeoutCallback(const ros::TimerEvent& e)
{
  preemptGoal();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishPursuingGoal()
{
  ROS_DEBUG("Stopping Runtime timer");
  runtime_timer_.stop();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startWaitingForPreempted()
{
  if (ac_->waiting_for_preempted_timeout_ != ros::Duration(0,0))
  {
    ROS_DEBUG("Starting [%.2fs] timer for the WaitingForPreempted timeout", ac_->waiting_for_preempted_timeout_.toSec());
    waiting_for_preempted_timer_ = ac_->n_.createTimer(ac_->waiting_for_preempted_timeout_, waiting_for_preempted_timer_.getCb());
  }
  else
    ROS_DEBUG("Infinite WaitingForPreempted timeout");

  setCommState(WAITING_FOR_PREEMPTED);
  setClientGoalStatus(ClientGoalStatus::ACTIVE);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishWaitingForPreempted()
{
  ROS_DEBUG("Stopping WaitForPreempted timer");
  waiting_for_preempted_timer_.stop();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::waitingForPreemptedTimeoutCallback(const ros::TimerEvent& e)
{
  ROS_DEBUG("WaitingForPreempted Timed out");
  processLost();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startWaitingForResult(const ClientGoalStatus& next_client_goal_status)
{
  if (ac_->waiting_for_result_timeout_ != ros::Duration(0,0))
  {
    ROS_DEBUG("Starting [%.2fs] timer for the WaitForResult timeout", ac_->waiting_for_result_timeout_.toSec());
    waiting_for_result_timer_ = ac_->n_.createTimer(ac_->waiting_for_result_timeout_, waiting_for_result_timer_.getCb());
  }
  else
    ROS_DEBUG("Infinite WaitForResult timeout");

  setCommState(WAITING_FOR_RESULT);
  setClientGoalStatus( ClientGoalStatus(next_client_goal_status) );
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::waitingForResultTimeoutCallback(const ros::TimerEvent& e)
{
  ROS_DEBUG("WaitingForResult Timed out");
  processLost();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::processLost()
{
  setClientGoalStatus(ClientGoalStatus::LOST);
  startDone(ActionResultConstPtr());
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startDone(const ActionResultConstPtr& action_result)
{
  if (action_result)
  {
    EnclosureDeleter<const ActionResult> d(action_result);
    result_ = ResultConstPtr(&(action_result->result), d);
  }
  else
    result_.reset(); // If they weren't expecting a result or we never got one, then don't give them one

  setCommState(DONE);

  if (cb_)
    cb_(GoalHandle(it_));
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishWaitingForResult()
{
  ROS_DEBUG("Stopping WaitForResult timer");
  waiting_for_result_timer_.stop();
}

}

#undef ActionClientTemplate
#undef ActionClientPrefix

#endif // ACTION_TOOLS_ACTION_CLIENT_H_
