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

/**
 * \brief Full interface to an ActionServer
 *
 * ActionClient provides a complete client side implementation of the ActionInterface protocol.
 * It provides callbacks for every client side transition, giving the user full observation into
 * the client side state machine.
 */
template <class ActionSpec>
class ActionClient
{
private:
  ACTION_DEFINITION(ActionSpec);
  typedef ActionClient<ActionSpec> ActionClientT;
  typedef boost::function<void (GoalHandle<ActionSpec>) > TransitionCallback;
  typedef boost::function<void (GoalHandle<ActionSpec>, const FeedbackConstPtr&) > FeedbackCallback;
  typedef boost::function<void (const ActionGoalConstPtr)> SendGoalFunc;

public:
  /**
   * \brief Simple constructor
   *
   * Constructs an ActionClient and sets up the necessary ros topics for the ActionInterface
   * \param name The action name. Defines the namespace in which the action communicates
   */
  ActionClient(const std::string& name) : n_(name)
  {
    initClient();
  }

  /**
   * \brief Constructor with namespacing options
   *
   * Constructs an ActionClient and sets up the necessary ros topics for the ActionInterface,
   * and namespaces them according the a specified NodeHandle
   * \param n The node handle on top of which we want to namespace our action
   * \param name The action name. Defines the namespace in which the action communicates
   */
  ActionClient(const ros::NodeHandle& n, const std::string& name) : n_(n, name)
  {
    initClient();
  }

  /**
   * \brief Sends a goal to the ActionServer, and also registers callbacks
   * \param transition_cb Callback that gets called on every client state transition
   * \param feedback_cb Callback that gets called whenever feedback for this goal is received
   */
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

  /**
   * \brief Cancel all goals currently running on the action server
   *
   * This preempts all goals running on the action server at the point that
   * this message is serviced by the ActionServer.
   */
  void cancelAllGoals()
  {
    ActionGoal cancel_msg;
    // CancelAll policy encoded by stamp=0, id=0
    cancel_msg.goal_id.stamp = ros::Time(0,0);
    cancel_msg.goal_id.id = ros::Time(0,0);
    cancel_msg.request_type.type = RequestType::PREEMPT_REQUEST;
    goal_pub_.publish(cancel_msg);
  }

  /**
   * \brief Cancel all goals that were stamped at and before the specified time
   * \param time All goals stamped at or before `time` will be canceled
   */
  void cancelGoalsAtAndBeforeTime(const ros::Time& time)
  {
    ActionGoal cancel_msg;
    cancel_msg.goal_id.stamp = time;
    cancel_msg.goal_id.id = ros::Time(0,0);
    cancel_msg.request_type.type = RequestType::PREEMPT_REQUEST;
    goal_pub_.publish(cancel_msg);
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
