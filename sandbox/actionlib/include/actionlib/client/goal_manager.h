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

#ifndef ACTIONLIB_GOAL_MANAGER_H_
#define ACTIONLIB_GOAL_MANAGER_H_

#include <boost/thread/recursive_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>


#include "actionlib/action_definition.h"

#include "actionlib/managed_list.h"
#include "actionlib/enclosure_deleter.h"
#include "actionlib/goal_id_generator.h"

#include "actionlib/client/comm_state.h"
#include "actionlib/client/terminal_state.h"

// msgs
#include "actionlib/GoalID.h"
#include "actionlib/GoalStatusArray.h"
#include "actionlib/RequestType.h"

namespace actionlib
{

//! \todo figure out why I get compile errors trying to use boost::mutex::scoped_lock()
class ScopedLock
{
public:
  ScopedLock(boost::recursive_mutex& mutex) : mutex_(mutex)  {  mutex_.lock(); }
  ~ScopedLock()  { mutex_.unlock(); }
private:
  boost::recursive_mutex& mutex_;
};

template <class ActionSpec>
class GoalHandle;

template <class ActionSpec>
class CommStateMachine;

template <class ActionSpec>
class GoalManager
{
public:
  ACTION_DEFINITION(ActionSpec);
  typedef GoalManager<ActionSpec> GoalManagerT;
  typedef boost::function<void (GoalHandle<ActionSpec>) > TransitionCallback;
  typedef boost::function<void (GoalHandle<ActionSpec>, const FeedbackConstPtr&) > FeedbackCallback;
  typedef boost::function<void (const ActionGoalConstPtr)> SendGoalFunc;

  GoalManager() { }

  void registerSendGoalFunc(SendGoalFunc send_goal_func);
  GoalHandle<ActionSpec> initGoal( const Goal& goal,
                                   TransitionCallback transition_cb = TransitionCallback(),
                                   FeedbackCallback feedback_cb = FeedbackCallback() );

  void updateStatuses(const GoalStatusArrayConstPtr& status_array);
  void updateFeedbacks(const ActionFeedbackConstPtr& action_feedback);
  void updateResults(const ActionResultConstPtr& action_result);

  friend class GoalHandle<ActionSpec>;

  // should be private
  typedef ManagedList< boost::shared_ptr<CommStateMachine<ActionSpec> > > ManagedListT;
  ManagedListT list_;
private:
  SendGoalFunc send_goal_func_ ;
  boost::recursive_mutex list_mutex_;

  GoalIDGenerator id_generator_;

  void listElemDeleter(typename ManagedListT::iterator it);
};


template <class ActionSpec>
class GoalHandle
{
  public:
    ACTION_DEFINITION(ActionSpec);

    GoalHandle();
    void reset();
    CommState getCommState();
    TerminalState getTerminalState();
    ResultConstPtr getResult();
    void resend();
    void cancel();

    friend class GoalManager<ActionSpec>;
  private:
    typedef GoalManager<ActionSpec> GoalManagerT;
    typedef ManagedList< boost::shared_ptr<CommStateMachine<ActionSpec> > > ManagedListT;

    GoalHandle(GoalManagerT* gm, typename ManagedListT::Handle handle);

    GoalManagerT* gm_;
    //typename ManagedListT::iterator it_;
    typename ManagedListT::Handle list_handle_;
};

template <class ActionSpec>
class CommStateMachine
{
  private:
    //generates typedefs that we'll use to make our lives easier
    ACTION_DEFINITION(ActionSpec);

  public:
    typedef boost::function<void (const GoalHandle<ActionSpec>&) > TransitionCallback;
    typedef boost::function<void (const GoalHandle<ActionSpec>&, const FeedbackConstPtr&) > FeedbackCallback;
    typedef GoalHandle<ActionSpec> GoalHandleT;

    CommStateMachine(const ActionGoalConstPtr& action_goal,
                     TransitionCallback transition_callback,
                     FeedbackCallback feedback_callback);

    ActionGoalConstPtr getActionGoal() const;
    CommState getCommState() const;
    GoalStatus getGoalStatus() const;
    ResultConstPtr getResult() const;

    // Transitions caused by messages
    void updateStatus(GoalHandleT& gh, const GoalStatusArrayConstPtr& status_array);
    void updateFeedback(GoalHandleT& gh, const ActionFeedbackConstPtr& feedback);
    void updateResult(GoalHandleT& gh, const ActionResultConstPtr& result);

    // Forced transitions
    void transitionToState(GoalHandleT& gh, const CommState::StateEnum& next_state);
    void transitionToState(GoalHandleT& gh, const CommState& next_state);
    void processLost(GoalHandleT& gh);
  private:
    CommStateMachine();

    // State
    CommState state_;
    ActionGoalConstPtr action_goal_;
    GoalStatus latest_goal_status_;
    ActionResultConstPtr latest_result_;

    // Callbacks
    TransitionCallback transition_cb_;
    FeedbackCallback   feedback_cb_;

    // **** Implementation ****
    //! Change the state, as well as print out ROS_DEBUG info
    void setCommState(const CommState& state);
    void setCommState(const CommState::StateEnum& state);
    const GoalStatus* findGoalStatus(const std::vector<GoalStatus>& status_vec) const;
};

}

#include "actionlib/client/goal_manager.cpp"
#include "actionlib/client/goal_handle.cpp"
#include "actionlib/client/comm_state_machine.cpp"

#endif // ACTIONLIB_GOAL_MANAGER_H_
