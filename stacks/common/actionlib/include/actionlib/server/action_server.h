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
#ifndef ACTION_LIB_ACTION_SERVER
#define ACTION_LIB_ACTION_SERVER

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <actionlib/GoalID.h>
#include <actionlib/GoalStatusArray.h>
#include <actionlib/GoalStatus.h>
#include <actionlib/RequestType.h>
#include <actionlib/enclosure_deleter.h>
#include <actionlib/action_definition.h>

#include <list>

namespace actionlib {
  /**
   * @class ActionServer
   * @brief The ActionServer is a helpful tool for managing goal requests to a
   * node. It allows the user to specify callbacks that are invoked when goal
   * or cancel requests come over the wire, and passes back GoalHandles that
   * can be used to track the state of a given goal request. The ActionServer
   * makes no assumptions about the policy used to service these goals, and
   * sends status for each goal over the wire until the last GoalHandle
   * associated with a goal request is destroyed.
   */
  template <class ActionSpec>
  class ActionServer {
    private:
      //generates typedefs that we'll use to make our lives easier
      ACTION_DEFINITION(ActionSpec);

      /**
       * @class StatusTracker
       * @brief A class for storing the status of each goal the action server
       * is currently working on
       */
      class StatusTracker {
        public:
          StatusTracker(const GoalID& goal_id, unsigned int status);

          StatusTracker(const boost::shared_ptr<const ActionGoal>& goal);

          boost::shared_ptr<const ActionGoal> goal_;
          boost::weak_ptr<void> handle_tracker_;
          GoalStatus status_;
          ros::Time handle_destruction_time_;
      };

      /**
       * @class HandleTrackerDeleter
       * @brief A class to help with tracking GoalHandles and removing goals
       * from the status list when the last GoalHandle associated with a given
       * goal is deleted.
       */
      //class to help with tracking status objects
      class HandleTrackerDeleter {
        public:
          HandleTrackerDeleter(ActionServer<ActionSpec>* as,
              typename std::list<StatusTracker>::iterator status_it);

          void operator()(void* ptr);

        private:
          ActionServer<ActionSpec>* as_;
          typename std::list<StatusTracker>::iterator status_it_;
      };

    public:
      /**
       * @class GoalHandle
       * @brief Encapsulates a state machine for a given goal that the user can
       * trigger transisions on. All ROS interfaces for the goal are managed by
       * the ActionServer to lessen the burden on the user.
       */
      class GoalHandle {
        public:
          /**
           * @brief  Default constructor for a GoalHandle
           */
          GoalHandle();

          /** @brief  Accept the goal referenced by the goal handle. This will
           * transition to the ACTIVE state or the PREEMPTING state depending
           * on whether a cancel request has been received for the goal
           */
          void setAccepted();

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to RECALLED or PREEMPTED
           * depending on what the current status of the goal is
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setCanceled(const Result& result = Result());

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to rejected
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setRejected(const Result& result = Result());

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to aborted
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setAborted(const Result& result = Result());

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to succeeded
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setSucceeded(const Result& result = Result());

          /**
           * @brief  Send feedback to any clients of the goal associated with this GoalHandle
           * @param feedback The feedback to send to the client
           */
          void publishFeedback(const Feedback& feedback);

          /**
           * @brief  Accessor for the goal associated with the GoalHandle
           * @return A shared_ptr to the goal object
           */
          boost::shared_ptr<const Goal> getGoal() const;

          /**
           * @brief  Accessor for the goal id associated with the GoalHandle
           * @return The goal id
           */
          GoalID getGoalID() const;

          /**
           * @brief  Accessor for the status associated with the GoalHandle
           * @return The goal status
           */
          GoalStatus getGoalStatus() const;

          /**
           * @brief  Equals operator for GoalHandles
           * @param other The GoalHandle to compare to
           * @return True if the GoalHandles refer to the same goal, false otherwise
           */
          bool operator==(const GoalHandle& other);

          /**
           * @brief  != operator for GoalHandles
           * @param other The GoalHandle to compare to
           * @return True if the GoalHandles refer to different goals, false otherwise
           */
          bool operator!=(const GoalHandle& other);

        private:
          /**
           * @brief  A private constructor used by the ActionServer to initialize a GoalHandle
           */
          GoalHandle(typename std::list<StatusTracker>::iterator status_it,
              ActionServer<ActionSpec>* as, boost::shared_ptr<void> handle_tracker);

          /**
           * @brief  A private method to set status to PENDING or RECALLING
           * @return True if the cancel request should be passed on to the user, false otherwise
           */
          bool setCancelRequested();

          typename std::list<StatusTracker>::iterator status_it_;
          boost::shared_ptr<const ActionGoal> goal_;
          ActionServer<ActionSpec>* as_;
          boost::shared_ptr<void> handle_tracker_;
          friend class ActionServer<ActionSpec>;
      };

      /**
       * @brief  Constructor for an ActionServer
       * @param  n A NodeHandle to create a namespace under
       * @param  name The name of the action
       * @param  goal_cb A goal callback to be called when the ActionServer receives a new goal over the wire
       * @param  cancel_cb A cancel callback to be called when the ActionServer receives a new cancel request over the wire
       */
      ActionServer(ros::NodeHandle n, std::string name,
          boost::function<void (GoalHandle)> goal_cb = boost::function<void (GoalHandle)>(),
          boost::function<void (GoalHandle)> cancel_cb = boost::function<void (GoalHandle)>());

      /**
       * @brief  Register a callback to be invoked when a new goal is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerGoalCallback(boost::function<void (GoalHandle)> cb);

      /**
       * @brief  Register a callback to be invoked when a new cancel is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerCancelCallback(boost::function<void (GoalHandle)> cb);

    private:
      /**
       * @brief  Publishes a result for a given goal
       * @param status The status of the goal with which the result is associated
       * @param result The result to publish
       */
      void publishResult(const GoalStatus& status, const Result& result);

      /**
       * @brief  Publishes feedback for a given goal
       * @param status The status of the goal with which the feedback is associated
       * @param feedback The feedback to publish
       */
      void publishFeedback(const GoalStatus& status, const Feedback& feedback);

      /**
       * @brief  The ROS callback for cancel requests coming into the ActionServer
       */
      void cancelCallback(const boost::shared_ptr<const GoalID>& goal_id);

      /**
       * @brief  The ROS callback for goals coming into the ActionServer
       */
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal);

      /**
       * @brief  Publish status for all goals on a timer event
       */
      void publishStatus(const ros::TimerEvent& e);

      /**
       * @brief  Explicitly publish status
       */
      void publishStatus();

      ros::NodeHandle node_;

      ros::Subscriber goal_sub_, cancel_sub_;
      ros::Publisher status_pub_, result_pub_, feedback_pub_;

      boost::recursive_mutex lock_;

      ros::Timer status_timer_;

      std::list<StatusTracker> status_list_;

      boost::function<void (GoalHandle)> goal_callback_;
      boost::function<void (GoalHandle)> cancel_callback_;

      ros::Time last_cancel_;
      ros::Duration status_list_timeout_;


  };
};

//Sinc all of this is templated things need to be header-based, but its still
//nice to separate out the implementation which we'll include here
#include <actionlib/server/status_tracker_imp.h>
#include <actionlib/server/handle_tracker_deleter_imp.h>
#include <actionlib/server/goal_handle_imp.h>
#include <actionlib/server/action_server_imp.h>
#endif
