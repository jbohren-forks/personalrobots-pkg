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
#ifndef ACTION_TOOLS_ACTION_SERVER
#define ACTION_TOOLS_ACTION_SERVER

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <action_tools/GoalID.h>
#include <action_tools/GoalStatusArray.h>
#include <action_tools/GoalStatus.h>
#include <action_tools/EnclosureDeleter.h>

#include <list> 

namespace action_tools {
  /**
   * @class ActionServer
   * @brief The ActionServer is a helpful tool for managing goal requests to a
   * node. It allows the user to specify callbacks that are invoked when goal
   * or preempt requests come over the wire, and passes back GoalHandles that
   * can be used to track the state of a given goal request. The ActionServer
   * makes no assumptions about the policy used to service these goals, and
   * sends status for each goal over the wire until the last GoalHandle
   * associated with a goal request is destroyed.
   */
  template <class ActionGoal, class Goal, class ActionResult, class Result, class ActionFeedback, class Feedback>
  class ActionServer {
    private:
      /**
       * @class StatusTracker
       * @brief A class for storing the status of each goal the action server
       * is currently working on
       */
      class StatusTracker {
        public:
          StatusTracker(const boost::shared_ptr<const ActionGoal>& goal)
            : goal_(goal) {
              //set the goal_id from the message
              status_.goal_id = goal_->goal_id;

              //initialize the status of the goal to pending
              status_.status = GoalStatus::PENDING;

              //if the goal id is zero, then we need to make up an id/timestamp for the goal
              if(status_.goal_id.id == ros::Time()){
                status_.goal_id.id = ros::Time::now();
                status_.goal_id.stamp = ros::Time::now();
              }
            }

          boost::shared_ptr<const ActionGoal> goal_;
          boost::weak_ptr<void> handle_tracker_;
          GoalStatus status_;
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
          HandleTrackerDeleter(ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>* as, 
              typename std::list<StatusTracker>::iterator status_it)
            : as_(as), status_it_(status_it) {}

          void operator()(void* ptr){
            if(as_){
              //make sure to lock while we erase status for this goal from the list
              as_->lock_.lock();
              as_->status_list_.erase(status_it_);
              as_->lock_.unlock();
            }
          }

        private:
          ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>* as_;
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
          GoalHandle(){}

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to active
           */
          void setActive(){
            if(goal_){
              if((*status_it_).status_.status == GoalStatus::PENDING){
                (*status_it_).status_.status = GoalStatus::ACTIVE;
                as_->publishStatus();
              }
              else
                ROS_ERROR("To transition to an active state, the goal must be in a pending state, it is currently in state: %d", 
                    (*status_it_).status_.status);
            }
            else
              ROS_ERROR("Attempt to set status on an uninitialized GoalHandle");
          }

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to rejected
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setRejected(const Result& result = Result()){
            if(goal_){
              if((*status_it_).status_.status == GoalStatus::PENDING){
                (*status_it_).status_.status = GoalStatus::REJECTED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to a rejected state, the goal must be in a pending state, it is currently in state: %d", 
                    (*status_it_).status_.status);
            }
            else
              ROS_ERROR("Attempt to set status on an uninitialized GoalHandle");
          }

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to aborted
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setAborted(const Result& result = Result()){
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PENDING || status == GoalStatus::ACTIVE){
                (*status_it_).status_.status = GoalStatus::ABORTED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to an aborted state, the goal must be in a pending or active state, it is currently in state: %d", 
                    status);
            }
            else
              ROS_ERROR("Attempt to set status on an uninitialized GoalHandle");
          }

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to preempted
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setPreempted(const Result& result = Result()){
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PENDING || status == GoalStatus::ACTIVE){
                (*status_it_).status_.status = GoalStatus::PREEMPTED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to a preempted state, the goal must be in a pending or active state, it is currently in state: %d", 
                    status);
            }
            else
              ROS_ERROR("Attempt to set status on an uninitialized GoalHandle");
          }

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to succeeded
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setSucceeded(const Result& result = Result()){
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PENDING || status == GoalStatus::ACTIVE){
                (*status_it_).status_.status = GoalStatus::SUCCEEDED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to a succeeded state, the goal must be in a pending or active state, it is currently in state: %d", 
                    status);
            }
            else
              ROS_ERROR("Attempt to set status on an uninitialized GoalHandle");
          }

          /**
           * @brief  Send feedback to any clients of the goal associated with this GoalHandle
           * @param feedback The feedback to send to the client 
           */
          void publishFeedback(const Feedback& feedback){
            if(goal_) {
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::ACTIVE || status == GoalStatus::PENDING)
                as_->publishFeedback((*status_it_).status_, feedback);
              else
                ROS_ERROR("To send feedback, the goal must be in a pending or active state, it is currently in state: %d", 
                    status);
            }
            else
              ROS_ERROR("Attempt to publish feedback on an uninitialized GoalHandle");
          }

          /**
           * @brief  Accessor for the goal associated with the GoalHandle
           * @return A shared_ptr to the goal object
           */
          boost::shared_ptr<const Goal> getGoal() const{
            //if we have a goal that is non-null
            if(goal_){
              //create the deleter for our goal subtype
              EnclosureDeleter<const ActionGoal> d(goal_);
              return boost::shared_ptr<const Goal>(&(goal_->goal), d);
            }
            return boost::shared_ptr<const Goal>();
          }

          /**
           * @brief  Accessor for the goal id associated with the GoalHandle
           * @return The goal id
           */
          GoalID getGoalID() const{
            if(goal_)
              return (*status_it_).status_.goal_id;
            else{
              ROS_ERROR("Attempt to get a goal id on an uninitialized GoalHandle");
              return GoalID();
            }
          }

          /**
           * @brief  Accessor for the status associated with the GoalHandle
           * @return The goal status
           */
          GoalStatus getGoalStatus() const{
            if(goal_)
              return (*status_it_).status_;
            else{
              ROS_ERROR("Attempt to get goal status on an uninitialized GoalHandle");
              return GoalStatus();
            }
          } 

          /**
           * @brief  Equals operator for GoalHandles
           * @param other The GoalHandle to compare to 
           * @return True if the GoalHandles refer to the same goal, false otherwise
           */
          bool operator==(const GoalHandle& other){
            if(!goal_ || !other.goal_)
              return false;
            GoalID my_id = getGoalID();
            GoalID their_id = other.getGoalID();
            return my_id.id == their_id.id;
          }

          /**
           * @brief  != operator for GoalHandles
           * @param other The GoalHandle to compare to 
           * @return True if the GoalHandles refer to different goals, false otherwise
           */
          bool operator!=(const GoalHandle& other){
            if(!goal_ || !other.goal_)
              return true;
            GoalID my_id = getGoalID();
            GoalID their_id = other.getGoalID();
            return my_id.id != their_id.id;
          }

        private:
          /**
           * @brief  A private constructor used by the ActionServer to initialize a GoalHandle
           */
          GoalHandle(typename std::list<StatusTracker>::iterator status_it,
              ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>* as)
            : status_it_(status_it), goal_((*status_it).goal_),
              as_(as), handle_tracker_((*status_it).handle_tracker_.lock()){}

          typename std::list<StatusTracker>::iterator status_it_;
          boost::shared_ptr<const ActionGoal> goal_;
          ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>* as_;
          boost::shared_ptr<void> handle_tracker_;
          friend class ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>;
      };

      /**
       * @brief  Constructor for an ActionServer
       * @param  n A NodeHandle to create a namespace under
       * @param  name The name of the action
       * @param  goal_cb A goal callback to be called when the ActionServer receives a new goal over the wire
       * @param  preempt_cb A preempt callback to be called when the ActionServer receives a new preempt over the wire
       * @return 
       */
      ActionServer(ros::NodeHandle n, std::string name, 
          boost::function<void (GoalHandle)> goal_cb = boost::function<void (GoalHandle)>(), 
          boost::function<void (GoalHandle)> preempt_cb = boost::function<void (GoalHandle)>())
        : node_(n, name), goal_callback_(goal_cb), preempt_callback_(preempt_cb) {
          status_pub_ = node_.advertise<action_tools::GoalStatusArray>("status", 1);
          result_pub_ = node_.advertise<ActionResult>("result", 1);
          feedback_pub_ = node_.advertise<ActionFeedback>("feedback", 1);

          goal_sub_ = node_.subscribe<ActionGoal>("goal", 1,
              boost::bind(&ActionServer::goalCallback, this, _1));

          //read the frequency with which to publish status from the parameter server
          double status_frequency;
          node_.param("status_frequency", status_frequency, 5.0);

          status_timer_ = node_.createTimer(ros::Duration(1.0 / status_frequency),
              boost::bind(&ActionServer::publishStatus, this, _1));

      }

      /**
       * @brief  Register a callback to be invoked when a new goal is received, this will replace any  previously registerd callback
       * @param  cb The callback to invoke
       */
      void registerGoalCallback(boost::function<void (GoalHandle)> cb){
        goal_callback_ = cb;
      }

      /**
       * @brief  Register a callback to be invoked when a new preempt is received, this will replace any  previously registerd callback
       * @param  cb The callback to invoke
       */
      void registerPreemptCallback(boost::function<void (GoalHandle)> cb){
        preempt_callback_ = cb;
      }

    private:
      /**
       * @brief  Publishes a result for a given goal
       * @param status The status of the goal with which the result is associated 
       * @param result The result to publish
       */
      void publishResult(const GoalStatus& status, const Result& result){
        boost::mutex::scoped_lock(lock_);
        //we'll create a shared_ptr to pass to ROS to limit copying
        boost::shared_ptr<ActionResult> ar(new ActionResult);
        ar->status = status;
        ar->result = result;
        result_pub_.publish(ar);
      }

      /**
       * @brief  Publishes feedback for a given goal
       * @param status The status of the goal with which the feedback is associated 
       * @param feedback The feedback to publish
       */
      void publishFeedback(const GoalStatus& status, const Feedback& feedback){
        boost::mutex::scoped_lock(lock_);
        //we'll create a shared_ptr to pass to ROS to limit copying
        boost::shared_ptr<ActionFeedback> af(new ActionFeedback);
        af->status = status;
        af->feedback = feedback;
        feedback_pub_.publish(af);
      }

      /**
       * @brief  The ROS callback for goals coming into the ActionServer
       */
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal){
        boost::mutex::scoped_lock(lock_);
        ROS_DEBUG("The action server is in the ROS goal callback");

        //first, we'll check if its a goal callback
        if(goal->request_type == ActionGoal::GOAL_REQUEST){
          //first, we need to create a StatusTracker associated with this goal and push it onto our list
          typename std::list<StatusTracker>::iterator it = status_list_.insert(status_list_.end(), StatusTracker(goal));

          //we need to create a handle tracker for the incoming goal and update the StatusTracker
          HandleTrackerDeleter d(this, it);
          boost::shared_ptr<void> handle_tracker((void *)NULL, d);
          (*it).handle_tracker_ = handle_tracker;

          //check if this goal has already been preempted
          if(goal->goal_id.stamp != ros::Time() && goal->goal_id.stamp <= last_preempt_){
            //if it has... just create a GoalHandle for it and setPreempted
            GoalHandle gh(it, this);
            gh.setPreempted();
          }
          else{
            //now, we need to create a goal handle and call the user's callback
            goal_callback_(GoalHandle(it, this));
          } 
        }
        //we need to handle a preempt for the user
        else if(goal->request_type == ActionGoal::PREEMPT_REQUEST){
          for(typename std::list<StatusTracker>::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
            //check if the goal id is zero or if it is equal to the goal id of the iterator
            if(
                (goal->goal_id.id == ros::Time() && goal->goal_id.stamp == ros::Time()) //id and stamp 0 --> preempt everything
                || goal->goal_id.id == (*it).status_.goal_id.id //ids match... preempt that goal 
                || (goal->goal_id.stamp != ros::Time() && (*it).status_.goal_id.stamp <= goal->goal_id.stamp) //stamp != 0 --> preempt everything before stamp
                ){
              //call the user's preempt callback on the relevant goal
              preempt_callback_(GoalHandle(it, this));
            }
          }

          //make sure to set last_preempt_ based on the stamp associated with this preempt
          if(goal->goal_id.stamp > last_preempt_)
            last_preempt_ = goal->goal_id.stamp;
        }
        else{
          //someone sent a goal with an unsupported status... we'll throw an error
          ROS_ERROR("A goal was sent to this action server with an undefined request_type! This goal will not be processed");
        }

      }

      /**
       * @brief  Publish status for all goals on a timer event
       */
      void publishStatus(const ros::TimerEvent& e){
        boost::mutex::scoped_lock(lock_);
        publishStatus();
      }

      /**
       * @brief  Explicitly publish status
       */
      void publishStatus(){
        boost::mutex::scoped_lock(lock_);
        //build a status array
        GoalStatusArray status_array;

        status_array.set_status_list_size(status_list_.size());

        unsigned int i = 0;
        for(typename std::list<StatusTracker>::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
          status_array.status_list[i] = (*it).status_;
          ++i;
        }
        status_pub_.publish(status_array);
      }

      ros::NodeHandle node_;

      ros::Subscriber goal_sub_;
      ros::Publisher status_pub_, result_pub_, feedback_pub_;

      boost::recursive_mutex lock_;

      ros::Timer status_timer_;

      std::list<StatusTracker> status_list_;

      boost::function<void (GoalHandle)> goal_callback_;
      boost::function<void (GoalHandle)> preempt_callback_;

      ros::Time last_preempt_;


  };
};
#endif
