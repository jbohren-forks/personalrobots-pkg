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

namespace action_tools {
  enum ActionServerState {
    IDLE,
    RUNNING
  };

  template <class ActionGoal, class ActionResult>
  class ActionServer {
    public:
      ActionServer(ros::NodeHandle n, std::string name, double status_frequency)
        : node_(n, name), new_goal_(false), preempt_request_(false) {
          status_pub_ = node_.advertise<action_tools::GoalStatus>("~status", 1);
          result_pub_ = node_.advertise<ActionResult>("~result", 1);

          goal_sub_ = node_.subscribe<ActionGoal>("~goal", 1,
              boost::bind(&ActionServer::goalCallback, this, _1));

          preempt_sub_ = node_.subscribe<action_tools::Preempt>("~preempt", 1,
              boost::bind(&ActionServer::preemptCallback, this, _1));

          status_timer_ = node_.createTimer(ros::Duration(1.0 / status_frequency),
              boost::bind(&ActionServer::publishStatus, this, _1));

          state_ = IDLE;
      }

      bool isNewGoalAvailable(){
        return new_goal_;
      }

      template <class Result> void sendResult(Result result){
        ActionResult r;
        r.header.stamp = ros::Time::now();

        lock_.lock();
        r.goal_id = current_goal_.goal_id;
        lock_.unlock();

        r.result = result;
        result_pub_.publish(r);
      }

      template <class Goal> Goal getNextGoal(){
        lock_.lock();
        Goal ret = next_goal_.goal;
        lock_.unlock();
        return ret;
      }

      void acceptNextGoal(){
        lock_.lock();
        //check that there isn't a preempt that would apply to the next goal as well
        if(next_goal_.header.stamp > last_preempt_.header.stamp){
          current_goal_ = next_goal_;
          new_goal_ = false;
          state_ = RUNNING;
          status_.goal_id.id = current_goal_.goal_id.id;
          status_.status = status_.ACTIVE;
        }
        else{
          //report the goal preempted even though it never started?
        }
        lock_.unlock();
      }

      bool isPreempted(){
        return preempt_request_;
      }

      bool isActive(){
        return state_ == RUNNING;
      }

      void succeeded(){
        lock_.lock();
        status_.status = status_.SUCCEEDED;
        state_ = IDLE;
        lock_.unlock();

        publishStatus();
      }

      void aborted(){
        lock_.lock();
        status_.status = status_.ABORTED;
        state_ = IDLE;
        lock_.unlock();

        publishStatus();
      }

      void preempted(){
        lock_.lock();
        status_.status = status_.PREEMPTED;
        state_ = IDLE;
        lock_.unlock();

        publishStatus();
      }

      void registerGoalCallback(boost::function<void ()> cb){
        lock_.lock();
        goal_callback_ = cb;
        lock_.unlock();
      }

      void registerPreemptCallback(boost::function<void ()> cb){
        lock_.lock();
        preempt_callback_ = cb;
        lock_.unlock();
      }

    private:
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal){
        lock_.lock();
        //check that the timestamp is past that of the current goal, the next goal, and past that of the last preempt
        if(goal->header.stamp > current_goal_.header.stamp
            && goal->header.stamp > next_goal_.header.stamp
            && goal->header.stamp > last_preempt_.header.stamp){
          next_goal_ = *goal;
          new_goal_ = true;

          //if the user has registered a goal callback, we'll call it now
          if(goal_callback_)
            goal_callback_();
        }
        else{
          //reject goal?
        }
        lock_.unlock();

      }

      void preemptCallback(const boost::shared_ptr<const action_tools::Preempt>& preempt){
        lock_.lock();
        //check that the timestamp is past that of the current goal and the last preempt
        if(preempt->header.stamp > current_goal_.header.stamp
            && preempt->header.stamp > last_preempt_.header.stamp){
          preempt_request_ = true;
          last_preempt_ = *preempt;

          //if the user has registered a preempt callback, we'll call it now
          if(preempt_callback_)
            preempt_callback_();
        }
        else {
          //send something about a preempt failing?
        }
        lock_.unlock();

      }

      void publishStatus(const ros::TimerEvent& e){
        publishStatus();
      }

      void publishStatus(){
        lock_.lock();
        status_pub_.publish(status_);
        lock_.unlock();
      }

      ros::NodeHandle node_;

      ros::Subscriber goal_sub_, preempt_sub_;
      ros::Publisher status_pub_, result_pub_;

      ActionServerState state_;
      ActionGoal current_goal_;
      ActionGoal next_goal_;
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
