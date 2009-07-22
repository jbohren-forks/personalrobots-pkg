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

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "action_tools/one_shot_timer.h"

namespace action_tools
{

template<class Goal, class Feedback, class Result>
class ActionClient
{
public:
  typedef ActionClient<Goal, Feedback, Result> ActionClientT;
  typedef boost::shared_ptr<const Goal> GoalConstPtr;
  typedef boost::shared_ptr<const Feedback> FeedbackConstPtr;
  typedef boost::shared_ptr<const Result> ResultConstPtr;
  typedef boost::function<void (const GoalHandle&)> CompletionCallback;

  enum GoalState  { PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, LOST } ;

  class GoalStatus
  {
    public:
      GoalStatus(const &

      void update(const ActionStatus& status_msg);


    private:
      enum CommState {WAITING_FOR_ACK, PURSUING_GOAL, WAITING_FOR_PREEMPTED, WAITING_FOR_TERMINAL_STATE, WAITING_FOR_RESULT, LOST};
      CommState comm_state_;
      ActionHeader header_;
      boost::weak_ptr<void> handle_tracker_;
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

  }


private:
  ros::NodeHandle n_;

  ros::Subscriber feedback_sub_;
  ros::Publisher  goal_pub_;
  ros::Subscriber status_sub_;

  // *************** Implementation ***************

  void statusCb(const ActionStatusConstPtr& status)
  {




  }
};




}

#endif
