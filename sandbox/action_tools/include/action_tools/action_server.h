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

  /*
   * This allows the creation of a shared pointer to a section
   * of an already reference counted structure. For example,
   * if in the following picture Enclosure is reference counted with
   * a boost::shared_ptr and you want to return a boost::shared_ptr
   * to the Member that is referenced counted along with Enclosure objects
   *
   * Enclosure ---------------  <--- Already reference counted
   * -----Member <------- A member of enclosure objects, eg. Enclosure.Member
   */
  template <class Enclosure> class EnclosureDeleter {
    public:
      EnclosureDeleter(const boost::shared_ptr<Enclosure>& enc_ptr) : enc_ptr_(enc_ptr){}

      template<class Member> void operator()(Member* member_ptr){
        enc_ptr_.reset();
      }

    private:
      boost::shared_ptr<Enclosure> enc_ptr_;
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
          status_pub_ = node_.advertise<action_tools::GoalStatus>("~status", 1);
          result_pub_ = node_.advertise<ActionResult>("~result", 1);

          goal_sub_ = node_.subscribe<ActionGoal>("~goal", 1,
              boost::bind(&ActionServer::goalCallback, this, _1));

          preempt_sub_ = node_.subscribe<action_tools::Preempt>("~preempt", 1,  
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
      
      void sendResult(Result result){
        ActionResult r;
        r.header.stamp = ros::Time::now();

        lock_.lock();
        r.goal_id = current_goal_->goal_id;
        lock_.unlock();

        r.result = result;
        result_pub_.publish(r);
      }

      GoalHandle getNextGoal(){
        lock_.lock();
        //create a GoalHandle for the current goal to pass to the user
        GoalHandle ret(current_goal_, this);
        lock_.unlock();
        return ret;
      }

      GoalHandle acceptNextGoal(){
        lock_.lock();

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

        lock_.unlock();


        return ret;
      }

      bool acceptGoal(const GoalHandle& goal_handle){
        if(goal_handle.action_server != this){
          ROS_ERROR("This Action Server will not accept a goal generated by another Action Server");
          return false;
        }

        lock_.lock();
        //check if we need to send a preempted message for the goal that we're currently pursuing
        if(isActive() && current_goal_->goal_id.id != goal_handle.action_goal_->goal_id.id){
          status_.status = status_.PREEMPTED;
          publishStatus();
        }

        current_goal_ = goal_handle.action_goal_;

        //if we are accepting the same goal that is stored in next_goal then we'll set new_goal to false
        if(current_goal_->goal_id.id == next_goal_->goal_id.id){
          new_goal_ = false;
        }

        status_.goal_id = current_goal_->goal_id;
        status_.status = status_.ACTIVE;
        state_ = RUNNING;
        publishStatus();

        lock_.unlock();

        return true;
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
        publishStatus();
        lock_.unlock();
      }

      void aborted(){
        lock_.lock();
        status_.status = status_.ABORTED;
        state_ = IDLE;
        publishStatus();
        lock_.unlock();
      }

      void preempted(){
        lock_.lock();
        status_.status = status_.PREEMPTED;
        state_ = IDLE;
        publishStatus();
        lock_.unlock();
      }

      void registerGoalCallback(boost::function<void (GoalHandle)> cb){
        goal_cb_lock_.lock();
        goal_callback_ = cb;
        goal_cb_lock_.unlock();
      }

      void registerPreemptCallback(boost::function<void ()> cb){
        preempt_cb_lock_.lock();
        preempt_callback_ = cb;
        preempt_cb_lock_.unlock();
      }

    private:
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal){
        bool call_cb = false;

        boost::shared_ptr<const ActionGoal> cb_action_goal;

        lock_.lock();
        //check that the timestamp is past that of the current goal, the next goal, and past that of the last preempt
        if(goal->header.stamp > current_goal_->header.stamp
            && goal->header.stamp > next_goal_->header.stamp
            && goal->header.stamp > last_preempt_.header.stamp){
          next_goal_ = goal;
          new_goal_ = true;

          //we'll call the callback outside of our global lock to prevent deadlock
          call_cb = true;
          cb_action_goal = next_goal_;
        }
        else{
          //reject goal?
        }
        lock_.unlock();

        //if the user has registered a goal callback, we'll call it now
        goal_cb_lock_.lock();
        if(goal_callback_ && call_cb){
          goal_callback_(GoalHandle(cb_action_goal, this));
        }
        goal_cb_lock_.unlock();

      }

      void preemptCallback(const boost::shared_ptr<const action_tools::Preempt>& preempt){
        bool call_cb = false;

        lock_.lock();
        //check that the timestamp is past that of the current goal and the last preempt
        if(preempt->header.stamp > current_goal_->header.stamp
            && preempt->header.stamp > last_preempt_.header.stamp){
          preempt_request_ = true;
          last_preempt_ = *preempt;

          //we'll call the callback outside of our global lock to prevent deadlock
          call_cb = true;
        }
        else {
          //send something about a preempt failing?
        }
        lock_.unlock();

        //if the user has registered a preempt callback, we'll call it now
        preempt_cb_lock_.lock();
        if(preempt_callback_ && call_cb){
          preempt_callback_();
        }
        preempt_cb_lock_.unlock();

      }

      void publishStatus(const ros::TimerEvent& e){
        lock_.lock();
        publishStatus();
        lock_.unlock();
      }

      void publishStatus(){
        status_pub_.publish(status_);
      }

      void publishGoalStatus(const ActionGoal& goal, const GoalStatus& status){
        status_pub_.publish(status_);
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

      boost::recursive_mutex lock_, goal_cb_lock_, preempt_cb_lock_;

      ros::Timer status_timer_;

      boost::function<void (GoalHandle)> goal_callback_;
      boost::function<void ()> preempt_callback_;


  };
};
#endif
