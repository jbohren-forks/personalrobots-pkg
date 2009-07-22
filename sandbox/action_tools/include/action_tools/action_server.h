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
#include <action_tools/Preempt.h>
#include <action_tools/ActionHeader.h>
#include <action_tools/StatusList.h>
#include <list>

namespace action_tools {
  template <class Goal, class Result, class Feedback>
  class ActionServer {
    private:
      //class for storing the status of each goal the server is working on
      class GoalStatus {
        public:
          GoalStatus(const boost::shared_ptr<const Goal>& goal)
            : goal_(goal) {
              //do something with the ActionHeader
          }

          boost::shared_ptr<const Goal> goal_;
          boost::weak_ptr<void> handle_tracker_;
          ActionHeader status_;
      };

      //class to help with tracking status objects
      class HandleTrackerDeleter {
        public:
          HandleTrackerDeleter(ActionServer<Goal, Result, Feedback>* as, 
              std::list<GoalStatus>::iterator status_it)
            : as_(as), status_it_(status_it) {}

          void operator()(void* ptr){
            if(as_){
              boost::mutex::scoped_lock(as_->status_lock_);
              as_->status_list_.erase(status_it_);
            }
          }

        private:
          ActionServer<Goal, Result, Feedback>* as_;
          std::list<GoalStatus>::iterator status_it_;
      };

    public:
      //we need to create a GoalHandle class for this ActionServer
      class GoalHandle {
        public:
          GoalHandle(){}

          GoalHandle(std::list<GoalStatus>::iterator status_it,
              ActionServer<Goal, Result, Feedback>* as)
            : status_it_(status_it), goal_(*status_it.goal_),
              as_(as), handle_tracker_(*status_it.handle_tracker_.lock()){}

          void setActive(){
            *status_it_.status_.status = *status_it_.status_.ACTIVE;
            as_->publishStatus();
          }

          void setRejected(){
            *status_it_.status_.status = *status_it_.status_.REJECTED;
            as_->publishStatus();
          }

          void setAborted(){
            *status_it_.status_.status = *status_it_.status_.ABORTED;
            as_->publishStatus();
          }

          void setPreempted(){
            *status_it_.status_.status = *status_it_.status_.PREEMPTED;
            as_->publishStatus();
          }

          void setSucceeded(){
            *status_it_.status_.status = *status_it_.status_.SUCCEEDED;
            as_->publishStatus();
          }

          boost::shared_ptr<const Goal> getGoal(){
            return action_goal_;
          }

        private:
          std::list<GoalStatus>::iterator status_it_;
          boost::shared_ptr<const Goal> goal_;
          const ActionServer<Goal, Result, Feedback>* as_;
          boost::shared_ptr<void> handle_tracker_;
          friend class ActionServer<Goal, Result, Feedback>;
      };

      ActionServer(ros::NodeHandle n, std::string name, 
          boost::function<void (const GoalHandle&)> goal_cb, 
          boost::function<void (const GoalHandle&)> preempt_cb, double status_frequency)
        : node_(n, name), goal_callback_(goal_cb), preempt_callback_(preempt_cb) {
          status_pub_ = node_.advertise<action_tools::StatusList>("status", 1);
          result_pub_ = node_.advertise<Result>("result", 1);
          feedback_pub_ = node.advertise<Feedback>("feedback", 1);

          goal_sub_ = node_.subscribe<Goal>("goal", 1,
              boost::bind(&ActionServer::goalCallback, this, _1));

          status_timer_ = node_.createTimer(ros::Duration(1.0 / status_frequency),
              boost::bind(&ActionServer::publishStatus, this, _1));

      }

      void sendResult(const Result& result){
        boost::mutex::scoped_lock(lock_);
        result_pub_.publish(result);
      }

      void registerGoalCallback(boost::function<void (GoalHandle)> cb){
        goal_callback_ = cb;
      }

      void registerPreemptCallback(boost::function<void (GoalHandle)> cb){
        preempt_callback_ = cb;
      }

    private:
      void goalCallback(const boost::shared_ptr<const Goal>& goal){
        boost::mutex::scoped_lock(lock_);
        ROS_DEBUG("The action server is in the ROS goal callback");

        //first, we'll check if its a goal callback
        if(goal->header.status.status == goal->header.status.GOAL_REQUEST){
          //first, we need to create a GoalStatus associated with this goal and push it onto our list
          std::list<GoalStatus>::iterator it = status_list_.insert(GoalStatus(goal), status_list_.end());

          //we need to create a handle tracker for the incoming goal and update the GoalStatus
          HandleTrackerDeleter d(this, it);
          boost::shared_ptr<void> handle_tracker(NULL, d);
          *it.handle_tracker_ = handle_tracker;

          //now, we need to create a goal handle and call the user's callback
          goal_callback_(GoalHandle(it, this));

        }
        else if(goal->header.status.status == goal->header.status.PREEMPT_REQUEST){
          //we need to handle a preempt for the user
        }
        else{
          //someone sent a goal with an unsupported status... we'll probably reject it by default.. and throw an error
        }

      }

      void publishStatus(const ros::TimerEvent& e){
        boost::mutex::scoped_lock(lock_);
        publishStatus();
      }

      void publishStatus(){
        boost::mutex::scoped_lock(lock_);
        status_pub_.publish(status_);
      }

      ros::NodeHandle node_;

      ros::Subscriber goal_sub_;
      ros::Publisher status_pub_, result_pub_, feedback_pub_;

      boost::recursive_mutex lock_;

      ros::Timer status_timer_;

      std::list<ActionHeader> status_list_;

      boost::function<void (GoalHandle)> goal_callback_;
      boost::function<void (GoalHandle)> preempt_callback_;


  };
};
#endif
