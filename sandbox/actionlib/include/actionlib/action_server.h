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
          StatusTracker(const GoalID& goal_id, unsigned int status){
            //set the goal id and status appropriately
            status_.goal_id = goal_id;
            status_.status = status;
          }

          StatusTracker(const boost::shared_ptr<const ActionGoal>& goal)
            : goal_(goal) {
              //set the goal_id from the message
              status_.goal_id = goal_->goal_id;

              //initialize the status of the goal to pending
              status_.status = GoalStatus::PENDING;

              //if the goal id is zero, then we need to make up an id for the goal
              if(status_.goal_id.id == ros::Time()){
                status_.goal_id.id = ros::Time::now();
              }

              //if the timestamp of the goal is zero, then we'll set it to now()
              if(status_.goal_id.stamp == ros::Time()){
                status_.goal_id.stamp = ros::Time::now();
              }
            }

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
              typename std::list<StatusTracker>::iterator status_it)
            : as_(as), status_it_(status_it) {}

          void operator()(void* ptr){
            if(as_){
              //make sure to lock while we erase status for this goal from the list
              as_->lock_.lock();
              (*status_it_).handle_destruction_time_ = ros::Time::now();
              //as_->status_list_.erase(status_it_);
              as_->lock_.unlock();
            }
          }

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
          GoalHandle(){}

          /** @brief  Accept the goal referenced by the goal handle. This will
           * transition to the ACTIVE state or the PREEMPTING state depending
           * on whether a cancel request has been received for the goal
           */
          void setAccepted(){
            ROS_DEBUG("Accepting goal, id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
            if(goal_){
              unsigned int status = (*status_it_).status_.status;

              //if we were pending before, then we'll go active
              if(status == GoalStatus::PENDING){
                (*status_it_).status_.status = GoalStatus::ACTIVE;
                as_->publishStatus();
              }
              //if we were recalling before, now we'll go to preempting
              else if(status == GoalStatus::RECALLING){
                (*status_it_).status_.status = GoalStatus::PREEMPTING;
                as_->publishStatus();
              }
              else
                ROS_ERROR("To transition to an active state, the goal must be in a pending or recalling state, it is currently in state: %d",
                    (*status_it_).status_.status);
            }
            else
              ROS_ERROR("Attempt to set status on an uninitialized GoalHandle");
          }

          /**
           * @brief  Set the status of the goal associated with the GoalHandle to RECALLED or PREEMPTED
           * depending on what the current status of the goal is
           * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
           */
          void setCanceled(const Result& result = Result()){
            ROS_DEBUG("Setting status to canceled on goal, id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PENDING || status == GoalStatus::RECALLING){
                (*status_it_).status_.status = GoalStatus::RECALLED;
                as_->publishResult((*status_it_).status_, result);
              }
              else if(status == GoalStatus::ACTIVE || status == GoalStatus::PREEMPTING){
                (*status_it_).status_.status = GoalStatus::PREEMPTED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to a cancelled state, the goal must be in a pending, recalling, active, or preempting state, it is currently in state: %d",
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
            ROS_DEBUG("Setting status to rejected on goal, id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PENDING || status == GoalStatus::RECALLING){
                (*status_it_).status_.status = GoalStatus::REJECTED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to a rejected state, the goal must be in a pending or recalling state, it is currently in state: %d",
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
            ROS_DEBUG("Setting status to aborted on goal, id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PREEMPTING || status == GoalStatus::ACTIVE){
                (*status_it_).status_.status = GoalStatus::ABORTED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to an aborted state, the goal must be in a preempting or active state, it is currently in state: %d",
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
            ROS_DEBUG("Setting status to succeeded on goal, id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PREEMPTING || status == GoalStatus::ACTIVE){
                (*status_it_).status_.status = GoalStatus::SUCCEEDED;
                as_->publishResult((*status_it_).status_, result);
              }
              else
                ROS_ERROR("To transition to a succeeded state, the goal must be in a preempting or active state, it is currently in state: %d",
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
            ROS_DEBUG("Publishing feedback for goal, id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
            if(goal_) {
              as_->publishFeedback((*status_it_).status_, feedback);
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
              ActionServer<ActionSpec>* as, boost::shared_ptr<void> handle_tracker)
            : status_it_(status_it), goal_((*status_it).goal_),
              as_(as), handle_tracker_(handle_tracker){}

          /**
           * @brief  A private method to set status to PENDING or RECALLING
           * @return True if the cancel request should be passed on to the user, false otherwise
           */
          bool setCancelRequested(){
            ROS_DEBUG("Transisitoning to a cancel requested state on goal id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
            if(goal_){
              unsigned int status = (*status_it_).status_.status;
              if(status == GoalStatus::PENDING){
                (*status_it_).status_.status = GoalStatus::RECALLING;
                as_->publishStatus();
                return true;
              }

              if(status == GoalStatus::ACTIVE){
                (*status_it_).status_.status = GoalStatus::PREEMPTING;
                as_->publishStatus();
                return true;
              }

            }
            return false;
          }

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
          boost::function<void (GoalHandle)> cancel_cb = boost::function<void (GoalHandle)>())
        : node_(n, name), goal_callback_(goal_cb), cancel_callback_(cancel_cb) {
          status_pub_ = node_.advertise<actionlib::GoalStatusArray>("status", 1);
          result_pub_ = node_.advertise<ActionResult>("result", 1);
          feedback_pub_ = node_.advertise<ActionFeedback>("feedback", 1);

          goal_sub_ = node_.subscribe<ActionGoal>("goal", 1,
              boost::bind(&ActionServer::goalCallback, this, _1));

          cancel_sub_ = node_.subscribe<GoalID>("cancel", 1,
              boost::bind(&ActionServer::cancelCallback, this, _1));

          //read the frequency with which to publish status from the parameter server
          double status_frequency, status_list_timeout;
          node_.param("status_frequency", status_frequency, 5.0);
          node_.param("status_list_timeout", status_list_timeout, 5.0);

          status_list_timeout_ = ros::Duration(status_list_timeout);

          status_timer_ = node_.createTimer(ros::Duration(1.0 / status_frequency),
              boost::bind(&ActionServer::publishStatus, this, _1));

      }

      /**
       * @brief  Register a callback to be invoked when a new goal is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerGoalCallback(boost::function<void (GoalHandle)> cb){
        goal_callback_ = cb;
      }

      /**
       * @brief  Register a callback to be invoked when a new cancel is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerCancelCallback(boost::function<void (GoalHandle)> cb){
        cancel_callback_ = cb;
      }

    private:
      /**
       * @brief  Publishes a result for a given goal
       * @param status The status of the goal with which the result is associated
       * @param result The result to publish
       */
      void publishResult(const GoalStatus& status, const Result& result){
        boost::recursive_mutex::scoped_lock lock(lock_);
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
        boost::recursive_mutex::scoped_lock lock(lock_);
        //we'll create a shared_ptr to pass to ROS to limit copying
        boost::shared_ptr<ActionFeedback> af(new ActionFeedback);
        af->status = status;
        af->feedback = feedback;
        feedback_pub_.publish(af);
      }

      /**
       * @brief  The ROS callback for cancel requests coming into the ActionServer
       */
      void cancelCallback(const boost::shared_ptr<const GoalID>& goal_id){
        boost::recursive_mutex::scoped_lock lock(lock_);
        //we need to handle a cancel for the user
        ROS_DEBUG("The action server has received a new cancel request");
        bool goal_id_found = false;
        for(typename std::list<StatusTracker>::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
          //check if the goal id is zero or if it is equal to the goal id of
          //the iterator or if the time of the iterator warrants a cancel
          if(
              (goal_id->id == ros::Time() && goal_id->stamp == ros::Time()) //id and stamp 0 --> cancel everything
              || goal_id->id == (*it).status_.goal_id.id //ids match... cancel that goal
              || (goal_id->stamp != ros::Time() && (*it).status_.goal_id.stamp <= goal_id->stamp) //stamp != 0 --> cancel everything before stamp
            ){
            //we need to check if we need to store this cancel request for later
            if(goal_id->id == (*it).status_.goal_id.id)
              goal_id_found = true;

            //attempt to get the handle_tracker for the list item if it exists
            boost::shared_ptr<void> handle_tracker = (*it).handle_tracker_.lock();

            if((*it).handle_tracker_.expired()){
              //if the handle tracker is expired, then we need to create a new one
              HandleTrackerDeleter d(this, it);
              handle_tracker = boost::shared_ptr<void>((void *)NULL, d);
              (*it).handle_tracker_ = handle_tracker;

              //we also need to reset the time that the status is supposed to be removed from the list
              (*it).handle_destruction_time_ = ros::Time();
            }

            //set the status of the goal to PREEMPTING or RECALLING as approriate
            //and check if the request should be passed on to the user
            GoalHandle gh(it, this, handle_tracker);
            if(gh.setCancelRequested()){
              //call the user's cancel callback on the relevant goal
              cancel_callback_(gh);
            }
          }

        }

        //if the requested goal_id was not found, and it is non-zero, then we need to store the cancel request
        if(goal_id->id != ros::Time() && !goal_id_found){
          typename std::list<StatusTracker>::iterator it = status_list_.insert(status_list_.end(),
              StatusTracker(*goal_id, GoalStatus::RECALLING));
          //start the timer for how long the status will live in the list without a goal handle to it
          (*it).handle_destruction_time_ = ros::Time::now();
        }

        //make sure to set last_cancel_ based on the stamp associated with this cancel request
        if(goal_id->stamp > last_cancel_)
          last_cancel_ = goal_id->stamp;
      }

      /**
       * @brief  The ROS callback for goals coming into the ActionServer
       */
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal){
        boost::recursive_mutex::scoped_lock lock(lock_);

        ROS_DEBUG("The action server has received a new goal request");

        //we need to check if this goal already lives in the status list
        for(typename std::list<StatusTracker>::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
          if(goal->goal_id.id == (*it).status_.goal_id.id){

            //if this is a request for a goal that has no active handles left,
            //we'll bump how long it stays in the list
            if((*it).handle_tracker_.expired()){
              (*it).handle_destruction_time_ = ros::Time::now();
            }

            //make sure not to call any user callbacks or add duplicate status onto the list
            return;
          }
        }

        //if the goal is not in our list, we need to create a StatusTracker associated with this goal and push it on
        typename std::list<StatusTracker>::iterator it = status_list_.insert(status_list_.end(), StatusTracker(goal));

        //we need to create a handle tracker for the incoming goal and update the StatusTracker
        HandleTrackerDeleter d(this, it);
        boost::shared_ptr<void> handle_tracker((void *)NULL, d);
        (*it).handle_tracker_ = handle_tracker;

        //check if this goal has already been canceled based on its timestamp
        if(goal->goal_id.stamp != ros::Time() && goal->goal_id.stamp <= last_cancel_){
          //if it has... just create a GoalHandle for it and setCanceled
          GoalHandle gh(it, this, handle_tracker);
          gh.setCanceled();
        }
        else{
          //now, we need to create a goal handle and call the user's callback
          goal_callback_(GoalHandle(it, this, handle_tracker));
        }
      }

      /**
       * @brief  Publish status for all goals on a timer event
       */
      void publishStatus(const ros::TimerEvent& e){
        boost::recursive_mutex::scoped_lock lock(lock_);
        publishStatus();
      }

      /**
       * @brief  Explicitly publish status
       */
      void publishStatus(){
        boost::recursive_mutex::scoped_lock lock(lock_);
        //build a status array
        GoalStatusArray status_array;

        status_array.set_status_list_size(status_list_.size());

        unsigned int i = 0;
        for(typename std::list<StatusTracker>::iterator it = status_list_.begin(); it != status_list_.end();){
          status_array.status_list[i] = (*it).status_;

          //check if the item is due for deletion from the status list
          if((*it).handle_destruction_time_ != ros::Time()
              && (*it).handle_destruction_time_ + status_list_timeout_ < ros::Time::now()){
            it = status_list_.erase(it);
          }
          else
            ++it;

          ++i;
        }

        status_pub_.publish(status_array);
      }

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
#endif
