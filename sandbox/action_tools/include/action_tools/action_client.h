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

#ifndef ACTION_TOOLS_ROBUST_MULTI_GOAL_ACTION_CLIENT_H_
#define ACTION_TOOLS_ROBUST_MULTI_GOAL_ACTION_CLIENT_H_

#include <list>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "action_tools/one_shot_timer.h"

// Messages
#include "action_tools/ActionHeader.h"
#include "action_tools/GoalStatus.h"

namespace action_tools
{

template<class ActionGoal, class Goal, class ActionResult, class Result, class ActionFeedback, class Feedback>
class ActionClient
{
public:
  // Need forward declaration for typedefs
  class GoalHandle;

  typedef ActionClient<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback> ActionClientT;
  typedef boost::shared_ptr<const Goal> GoalConstPtr;
  typedef boost::shared_ptr<const Feedback> FeedbackConstPtr;
  typedef boost::shared_ptr<const Result> ResultConstPtr;
  typedef boost::function<void (const GoalHandle&) > CompletionCallback;

  class GoalStates
  {
  public:
    enum GoalState  { PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, LOST } ;
  };

  // Keeps track of the communication status of a single requested goal
  class GoalManager
  {
    public:
      GoalManager(const ros::Time& stamp, const GoalID& id)
      {
        comm_state_ = IDLE;
      }
      void update(const GoalStatus& status_msg);

    private:
      enum CommState {IDLE, WAITING_FOR_ACK, PURSUING_GOAL, WAITING_FOR_PREEMPTED, WAITING_FOR_TERMINAL_STATE, WAITING_FOR_RESULT, LOST};

      boost::mutex mutex_;
      typename GoalStates::GoalState goal_state_;
      CommState comm_state_;
      ros::Time stamp;
      boost::weak_ptr<void> handle_tracker_;
  };

  // Used to clean up the GoalManager list in the ActionClient, once there are no more goal handles pointing to the status
  class HandleTrackerDeleter
  {
    public:
      HandleTrackerDeleter(ActionClientT* ac, typename std::list<GoalManager>::iterator it) : it_(it), ac_(ac)
      {  }

      void operator() (void* ptr)
      {
        boost::mutex::scoped_lock(ac_->manager_list_mutex_);
        ac_->status_trackers_.erase(it_);
      }

    private:
      typename std::list<GoalManager>::iterator it_;
      ActionClientT* ac_;
  };

  // Object provided to user to make queries against a specific goal
  class GoalHandle
  {
    public:
      GoalHandle(typename std::list<GoalManager>::iterator it)//, ActionClientT* ac)
       : status_it_(it), handle_tracker_((*status_it_).handle_tracker_.lock()) //, ac_(ac)
      {  }

      typename GoalStates::GoalState getState()
      {
        boost::mutex::scoped_lock(status_it_->mutex_);
        return status_it_->goal_state_;
      }

    private:
      typename std::list<GoalManager>::iterator status_it_;
      boost::shared_ptr<void> handle_tracker_;
      // ActionClientT* ac_;
  };

  ActionClient(const std::string& name) : n_(name)
  {
    initClient();
  }

  ActionClient(const ros::NodeHandle& n, const std::string& name) : n_(n, name)
  {
    initClient();
  }

  void initClient()
  {
    goal_pub_ = n_.advertise<Goal>("goal", 1);
    status_sub_ = n_.subscribe("status", 1, &ActionClientT::statusCb, this);
    feedback_sub_ = n_.subscribe("feedback", 1, &ActionClientT::feedbackCb, this);
  }

  GoalHandle sendGoal(const Goal& goal, CompletionCallback cb)
  {
    // Add a goal manager to our list of managers
    typename std::list<GoalManager>::iterator it = manager_list_.insert(GoalManager(goal->action_header.goal_id), manager_list_.end());

    // Create a custom deallocater to eventually destroy the GoalManager, once
    //   we no longer have any GoalHandles in scope
    HandleTrackerDeleter d(this, it);
    boost::shared_ptr<void> handle_tracker((void*) NULL, d);
    (*it).handle_tracker_ = handle_tracker;

    GoalHandle gh = GoalHandle(it);//, this);
  }

private:
  ros::NodeHandle n_;

  ros::Subscriber feedback_sub_;
  ros::Publisher  goal_pub_;
  ros::Subscriber status_sub_;

  boost::mutex manager_list_mutex_;
  std::list<GoalManager> manager_list_;

  // *************** Implementation ***************

  void statusCb(const GoalStatusConstPtr& status)
  {




  }

  void feedbackCb(const FeedbackConstPtr& status)
  {

  }


};




}

#endif
