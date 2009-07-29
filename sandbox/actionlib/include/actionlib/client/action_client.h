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

#ifndef ACTIONLIB_ACTION_CLIENT_H_
#define ACTIONLIB_ACTION_CLIENT_H_

#include "ros/ros.h"
#include "actionlib/client/goal_manager.h"

namespace actionlib
{

template <class ActionSpec>
class ActionClient
{
public:
  ACTION_DEFINITION(ActionSpec);
  typedef ActionClient<ActionSpec> ActionClientT;
  typedef boost::function<void (GoalHandle<ActionSpec>) > TransitionCallback;
  typedef boost::function<void (GoalHandle<ActionSpec>, const FeedbackConstPtr&) > FeedbackCallback;
  typedef boost::function<void (const ActionGoalConstPtr)> SendGoalFunc;

  ActionClient(const std::string& name) : n_(name)
  {
    initClient();
  }

  ActionClient(const ros::NodeHandle& n, const std::string& name) : n_(n, name)
  {
    initClient();
  }

  GoalHandle<ActionSpec> sendGoal(const Goal& goal,
                                  TransitionCallback transition_cb = TransitionCallback(),
                                  FeedbackCallback   feedback_cb   = FeedbackCallback())
  {
    ROS_DEBUG("about to start initGoal()");
    GoalHandle<ActionSpec> gh = manager_.initGoal(goal, transition_cb, feedback_cb);
    ROS_DEBUG("Done with initGoal()");

    ROS_DEBUG("Preemptively going to create a handle");
    typename ManagedList< boost::shared_ptr<CommStateMachine<ActionSpec> > >::Handle h = manager_.list_.begin().createHandle();
    ROS_DEBUG("Done");

    return gh;
  }

private:
  ros::NodeHandle n_;

  ros::Subscriber feedback_sub_;
  ros::Publisher  goal_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  GoalManager<ActionSpec> manager_;

  void sendGoalFunc(const ActionGoalConstPtr& action_goal)
  {
    goal_pub_.publish(action_goal);
  }

  void initClient()
  {
    // Start publishers and subscribers
    goal_pub_ = n_.advertise<ActionGoal>("goal", 1);
    manager_.registerSendGoalFunc(boost::bind(&ActionClientT::sendGoalFunc, this, _1));

    status_sub_   = n_.subscribe("status",   1, &ActionClientT::statusCb, this);
    feedback_sub_ = n_.subscribe("feedback", 1, &ActionClientT::feedbackCb, this);
    result_sub_   = n_.subscribe("result",   1, &ActionClientT::resultCb, this);
  }

  void statusCb(const GoalStatusArrayConstPtr& status_array)
  {
    manager_.updateStatuses(status_array);
  }

  void feedbackCb(const ActionFeedbackConstPtr& action_feedback)
  {
    manager_.updateFeedbacks(action_feedback);
  }

  void resultCb(const ActionResultConstPtr& action_result)
  {
    manager_.updateResults(action_result);
  }
};

}









#endif
