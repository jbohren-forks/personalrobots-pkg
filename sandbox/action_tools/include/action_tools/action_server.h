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
#include <action_tools/GoalStatus.h>
#include <action_tools/EnclosureDeleter.h>

namespace action_tools {
  enum ActionServerState {
    IDLE,
    RUNNING
  };

  template <class ActionGoal, class Goal, class ActionResult, class Result>
  class ActionServer {
    public:
      //we need to create a GoalHandle class for this ActionServer
      class GoalHandle{
        public:
          GoalHandle(){}

          GoalHandle(const boost::shared_ptr<const ActionGoal>& action_goal, ActionServer<ActionGoal, Goal, ActionResult, Result>* action_server)
            : action_goal_(action_goal), action_server_(action_server){}

          boost::shared_ptr<const Goal> getGoal(){
            //if we have a goal that is non-null
            if(action_goal_){
              //create the deleter for our goal subtype
              EnclosureDeleter<const ActionGoal> d(action_goal_);
              return boost::shared_ptr<const Goal>(&(action_goal_->goal), d);
            }
            return boost::shared_ptr<const Goal>();
          }

        private:
          boost::shared_ptr<const ActionGoal> action_goal_;
          const ActionServer<ActionGoal, Goal, ActionResult, Result>* action_server_;
          friend class ActionServer<ActionGoal, Goal, ActionResult, Result>;
      };

      ActionServer(ros::NodeHandle n, std::string name, double status_frequency)
        : node_(n, name), new_goal_(false), preempt_request_(false) {
          status_pub_ = node_.advertise<action_tools::GoalStatus>("status", 1);
          result_pub_ = node_.advertise<ActionResult>("result", 1);

          goal_sub_ = node_.subscribe<ActionGoal>("goal", 1,
              boost::bind(&ActionServer::goalCallback, this, _1));

          preempt_sub_ = node_.subscribe<action_tools::Preempt>("preempt", 1,
              boost::bind(&ActionServer::preemptCallback, this, _1));

          status_timer_ = node_.createTimer(ros::Duration(1.0 / status_frequency),
              boost::bind(&ActionServer::publishStatus, this, _1));

          //initialize our goals to take advantage of the fact that the default constructor
          //for ros::Time gives a value of 0
          current_goal_ = boost::shared_ptr<const ActionGoal>(new ActionGoal());
          next_goal_ = boost::shared_ptr<const ActionGoal>(new ActionGoal());

          state_ = IDLE;
      }

      bool isNewGoalAvailable(){
        return new_goal_;
      }

      void sendResult(const Result& result){
        ActionResult r;
        r.header.stamp = ros::Time::now();

        lock_.lock();
        r.goal_id = current_goal_->goal_id;
        lock_.unlock();

        r.result = result;
        result_pub_.publish(r);
      }

      GoalHandle acceptNextGoal(){
        boost::mutex::scoped_lock(lock_);

        //check if we need to send a preempted message for the goal that we're currently pursuing
        if(isActive() && current_goal_->goal_id.id != next_goal_->goal_id.id){
          status_.status = status_.PREEMPTED;
          publishStatus();
        }

        //accept the next goal
        current_goal_ = next_goal_;
        new_goal_ = false;

        //generate the goal handle to return
        GoalHandle ret(current_goal_, this);

        status_.goal_id = current_goal_->goal_id;
        status_.status = status_.ACTIVE;
        state_ = RUNNING;
        publishStatus();

        return ret;
      }

      bool isPreempted(){
        return preempt_request_;
      }

      bool isActive(){
        return state_ == RUNNING;
      }

      void succeeded(const Result& result){
        boost::mutex::scoped_lock(lock_);
        sendResult(result);
        succeeded();
      }

      void succeeded(){
        boost::mutex::scoped_lock(lock_);
        if(state_ == IDLE){
          ROS_WARN("Trying to succeed on a goal that is already in a terminal state... doing nothing");
          return;
        }
        status_.status = status_.SUCCEEDED;
        state_ = IDLE;
        publishStatus();
      }

      void aborted(const Result& result){
        boost::mutex::scoped_lock(lock_);
        sendResult(result);
        aborted();
      }

      void aborted(){
        boost::mutex::scoped_lock(lock_);
        if(state_ == IDLE){
          ROS_WARN("Trying to abort on a goal that is already in a terminal state... doing nothing");
          return;
        }
        status_.status = status_.ABORTED;
        state_ = IDLE;
        publishStatus();
      }

      void preempted(const Result& result){
        boost::mutex::scoped_lock(lock_);
        sendResult(result);
        preempted();
      }

      void preempted(){
        boost::mutex::scoped_lock(lock_);
        if(state_ == IDLE){
          ROS_WARN("Trying to preempt on a goal that is already in a terminal state... doing nothing");
          return;
        }
        status_.status = status_.PREEMPTED;
        state_ = IDLE;
        publishStatus();
      }

      void registerGoalCallback(boost::function<void ()> cb){
        boost::mutex::scoped_lock(goal_cb_lock_);
        goal_callback_ = cb;
      }

      void registerPreemptCallback(boost::function<void ()> cb){
        boost::mutex::scoped_lock(preempt_cb_lock_);
        preempt_callback_ = cb;
      }

    private:
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal){
        ROS_DEBUG("The action server is in the ROS goal callback");
        boost::mutex::scoped_lock(lock_);
        //check that the timestamp is past that of the current goal, the next goal, and past that of the last preempt
        if(goal->header.stamp > current_goal_->header.stamp
            && goal->header.stamp > next_goal_->header.stamp
            && goal->header.stamp > last_preempt_.header.stamp){
          next_goal_ = goal;
          new_goal_ = true;

          //if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
          if(next_goal_->goal_id.id != current_goal_->goal_id.id){
            GoalStatus status;
            status.status = status.PREEMPTED;
            publishGoalStatus(*next_goal_, status);
          }

          //if the user has defined a goal callback, we'll call it now
          if(goal_callback_)
            goal_callback_();
        }
        else{
          //the goal requested has already been preempted, so we're not going to execute it
          GoalStatus status;
          status.status = status.PREEMPTED;
          publishGoalStatus(*goal, status);
        }
      }

      void preemptCallback(const boost::shared_ptr<const action_tools::Preempt>& preempt){
        boost::mutex::scoped_lock(lock_);
        ROS_DEBUG("In Preempt Callback");

        //check that the timestamp is past that of the current goal and the last preempt
        if(preempt->header.stamp >= current_goal_->header.stamp
            && preempt->header.stamp > last_preempt_.header.stamp){
          ROS_DEBUG("Setting preempt_request bit to TRUE");
          preempt_request_ = true;
          last_preempt_ = *preempt;

          //if the preempt also applies to the next goal, then we need to preempt it too
          //makes sure that the current and next goals are different
          //also makes sure to set new_goal_ to false if it had been set to true
          if(preempt->header.stamp >= next_goal_->header.stamp
              && next_goal_->goal_id.id != current_goal_->goal_id.id){
            GoalStatus status;
            status.status = status.PREEMPTED;
            publishGoalStatus(*next_goal_, status);

            if(new_goal_)
              new_goal_ = false;
          }

          //if the user has registered a preempt callback, we'll call it now
          if(preempt_callback_)
            preempt_callback_();
        }
      }

      void publishStatus(const ros::TimerEvent& e){
        boost::mutex::scoped_lock(lock_);
        publishStatus();
      }

      void publishStatus(){
        status_pub_.publish(status_);
      }

      void publishGoalStatus(const ActionGoal& goal, GoalStatus& status){
        //make sure that the status is published with the correct id and stamp 
        status.header.stamp = ros::Time::now();  
        status.goal_id = goal.goal_id;
        status_pub_.publish(status);
      }

      ros::NodeHandle node_;

      ros::Subscriber goal_sub_, preempt_sub_;
      ros::Publisher status_pub_, result_pub_;

      ActionServerState state_;
      boost::shared_ptr<const ActionGoal> current_goal_;
      boost::shared_ptr<const ActionGoal> next_goal_;
      action_tools::GoalStatus status_;
      action_tools::Preempt last_preempt_;

      bool new_goal_, preempt_request_;

      boost::recursive_mutex lock_;

      ros::Timer status_timer_;

      boost::function<void ()> goal_callback_;
      boost::function<void ()> preempt_callback_;


  };
};
#endif
