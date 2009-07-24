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
  template <class ActionGoal, class Goal, class ActionResult, class Result, class ActionFeedback, class Feedback>
  class ActionServer {
    private:
      //class for storing the status of each goal the server is working on
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
      //we need to create a GoalHandle class for this ActionServer
      class GoalHandle {
        public:
          GoalHandle(){}

          GoalHandle(typename std::list<StatusTracker>::iterator status_it,
              ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>* as)
            : status_it_(status_it), goal_((*status_it).goal_),
              as_(as), handle_tracker_((*status_it).handle_tracker_.lock()){}

          void setActive(){
            if(goal_){
              if((*status_it_).status_.status == GoalStatus::PENDING){
                (*status_it_).status_.status = GoalStatus::ACTIVE;
                as_->publishStatus();
              }
              else
                ROS_ERROR("To transition to a rejected state, the goal must be in a pending state, it is currently in state: %d", 
                    (*status_it_).status_.status);
            }
            else
              ROS_ERROR("Attempt to set status on an uninitialized GoalHandle");
          }

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

          void publishFeedback(const Feedback& feedback){
            if(goal_) {
              as_->publishFeedback((*status_it_).status_, feedback);
            }
            else
              ROS_ERROR("Attempt to publish feedback on an uninitialized GoalHandle");
          }

          boost::shared_ptr<const Goal> getGoal() const{
            //if we have a goal that is non-null
            if(goal_){
              //create the deleter for our goal subtype
              EnclosureDeleter<const ActionGoal> d(goal_);
              return boost::shared_ptr<const Goal>(&(goal_->goal), d);
            }
            return boost::shared_ptr<const Goal>();
          }

          GoalID getGoalID() const{
            if(goal_)
              return (*status_it_).status_.goal_id;
            else{
              ROS_ERROR("Attempt to get a goal id on an uninitialized GoalHandle");
              return GoalID();
            }
          }

          GoalStatus getGoalStatus() const{
            if(goal_)
              return (*status_it_).status_;
            else{
              ROS_ERROR("Attempt to get goal status on an uninitialized GoalHandle");
              return GoalStatus();
            }
          } 

        private:
          typename std::list<StatusTracker>::iterator status_it_;
          boost::shared_ptr<const ActionGoal> goal_;
          ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>* as_;
          boost::shared_ptr<void> handle_tracker_;
          friend class ActionServer<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>;
      };

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

      void publishResult(const GoalStatus& status, const Result& result){
        boost::mutex::scoped_lock(lock_);
        //we'll create a shared_ptr to pass to ROS to limit copying
        boost::shared_ptr<ActionResult> ar(new ActionResult);
        ar->status = status;
        ar->result = result;
        result_pub_.publish(ar);
      }

      void publishFeedback(const GoalStatus& status, const Feedback& feedback){
        boost::mutex::scoped_lock(lock_);
        //we'll create a shared_ptr to pass to ROS to limit copying
        boost::shared_ptr<ActionFeedback> af(new ActionFeedback);
        af->status = status;
        af->feedback = feedback;
        feedback_pub_.publish(af);
      }

      void registerGoalCallback(boost::function<void (GoalHandle)> cb){
        goal_callback_ = cb;
      }

      void registerPreemptCallback(boost::function<void (GoalHandle)> cb){
        preempt_callback_ = cb;
      }

    private:
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

          //now, we need to create a goal handle and call the user's callback
          goal_callback_(GoalHandle(it, this));

        }
        //we need to handle a preempt for the user
        else if(goal->request_type == ActionGoal::PREEMPT_REQUEST){
          for(typename std::list<StatusTracker>::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
            //check if the goal id is zero or if it is equal to the goal id of the iterator
            if(goal->goal_id.id == ros::Time::now() 
                || goal->goal_id.id == (*it).status_.goal_id.id){
              //call the user's preempt callback on the relevant goal
              preempt_callback_(GoalHandle(it, this));
            }
          }
        }
        else{
          //someone sent a goal with an unsupported status... we'll throw an error
          ROS_ERROR("A goal was sent to this action server with an undefined request_type! This goal will not be processed");
        }

      }

      void publishStatus(const ros::TimerEvent& e){
        boost::mutex::scoped_lock(lock_);
        publishStatus();
      }

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


  };
};
#endif
