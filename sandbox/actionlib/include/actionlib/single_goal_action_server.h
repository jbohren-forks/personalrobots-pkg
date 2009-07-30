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
#ifndef ACTIONLIB_SINGLE_GOAL_ACTION_SERVER_H_
#define ACTIONLIB_SINGLE_GOAL_ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/action_server.h>
#include <actionlib/action_definition.h>

namespace actionlib {
  /** @class SingleGoalActionServer @brief The SingleGoalActionServer
   * implements a singe goal policy on top of the ActionServer class. The
   * specification of the policy is as follows: only one goal can have an
   * active status at a time, new goals preempt previous goals based on the
   * stamp in their GoalID field (later goals preempt earlier ones), an
   * explicit preempt goal preempts all goals with timestamps that are less
   * than or equal to the stamp associated with the preempt, accepting a new
   * goal implies successful preemption of any old goal and the status of the
   * old goal will be change automatically to reflect this.
   */
  template <class ActionSpec>
  class SingleGoalActionServer {
    public:
      //generates typedefs that we'll use to make our lives easier
      ACTION_DEFINITION(ActionSpec);

      typedef typename ActionServer<ActionSpec>::GoalHandle GoalHandle;

      /**
       * @brief  Constructor for a SingleGoalActionServer
       * @param n A NodeHandle to create a namespace under
       * @param name A name for the action server
       */
      SingleGoalActionServer(ros::NodeHandle n, std::string name)
        : new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false) {

          //create the action server
          as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n, name,
                boost::bind(&SingleGoalActionServer::goalCallback, this, _1),
                boost::bind(&SingleGoalActionServer::preemptCallback, this, _1)));

      }

      /**
       * @brief  Accepts a new goal when one is available The status of this
       * goal is set to active upon acceptance, and the status of any
       * previously active goal is set to preempted. Preempts received for the
       * new goal between checking if isNewGoalAvailable or invokation of a
       * goal callback and the acceptNewGoal call will not trigger a preempt
       * callback.  This means, isPreemptReqauested should be called after
       * accepting the goal even for callback-based implementations to make
       * sure the new goal does not have a pending preempt request.
       * @return A shared_ptr to the new goal.
       */
      boost::shared_ptr<const Goal> acceptNewGoal(){
        boost::mutex::scoped_lock(lock_);

        if(!new_goal_ || !next_goal_.getGoal()){
          ROS_ERROR("Attempting to accept the next goal when a new goal is not available");
          return boost::shared_ptr<const Goal>();
        }

        //check if we need to send a preempted message for the goal that we're currently pursuing
        if(isActive()
            && current_goal_.getGoal()
            && current_goal_ != next_goal_){
          current_goal_.setCanceled();
        }

        ROS_DEBUG("Accepting a new goal");

        //accept the next goal
        current_goal_ = next_goal_;
        new_goal_ = false;

        //set preempt to request to equal the preempt state of the new goal
        preempt_request_ = new_goal_preempt_request_;
        new_goal_preempt_request_ = false;

        //set the status of the current goal to be active
        current_goal_.setAccepted();

        return current_goal_.getGoal();
      }

      /**
       * @brief  Allows  polling implementations to query about the availability of a new goal
       * @return True if a new goal is available, false otherwise
       */
      bool isNewGoalAvailable(){
        return new_goal_;
      }


      /**
       * @brief  Allows  polling implementations to query about preempt requests
       * @return True if a preempt is requested, false otherwise
       */
      bool isPreemptRequested(){
        return preempt_request_;
      }

      /**
       * @brief  Allows  polling implementations to query about the status of the current goal
       * @return True if a goal is active, false otherwise
       */
      bool isActive(){
        if(!current_goal_.getGoal())
          return false;
        return current_goal_.getGoalStatus().status == GoalStatus::ACTIVE;
      }

      /**
       * @brief  Sets the status of the active goal to succeeded
       * @param  result An optional result to send back to any clients of the goal
       */
      void setSucceeded(const Result& result = Result()){
        boost::mutex::scoped_lock(lock_);
        current_goal_.setSucceeded(result);
      }

      /**
       * @brief  Sets the status of the active goal to aborted
       * @param  result An optional result to send back to any clients of the goal
       */
      void setAborted(const Result& result = Result()){
        boost::mutex::scoped_lock(lock_);
        current_goal_.setAborted(result);
      }

      /**
       * @brief  Sets the status of the active goal to preempted
       * @param  result An optional result to send back to any clients of the goal
       */
      void setPreempted(const Result& result = Result()){
        boost::mutex::scoped_lock(lock_);
        current_goal_.setCanceled(result);
      }

      /**
       * @brief  Allows users to register a callback to be invoked when a new goal is available
       * @param cb The callback to be invoked
       */
      void registerGoalCallback(boost::function<void ()> cb){
        goal_callback_ = cb;
      }

      /**
       * @brief  Allows users to register a callback to be invoked when a new preempt request is available
       * @param cb The callback to be invoked
       */
      void registerPreemptCallback(boost::function<void ()> cb){
        preempt_callback_ = cb;
      }

    private:
      /**
       * @brief  Callback for when the ActionServer receives a new goal and passes it on
       */
      void goalCallback(GoalHandle goal){
        boost::mutex::scoped_lock(lock_);
        ROS_DEBUG("A new goal has been recieved by the single goal action server");

        //check that the timestamp is past that of the current goal and the next goal
        if((!current_goal_.getGoal() || goal.getGoalID().stamp > current_goal_.getGoalID().stamp)
            && (!next_goal_.getGoal() || goal.getGoalID().stamp > next_goal_.getGoalID().stamp)){

          //if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
          if(next_goal_.getGoal() && (!current_goal_.getGoal() || next_goal_ != current_goal_)){
            next_goal_.setCanceled();
          }

          next_goal_ = goal;
          new_goal_ = true;
          new_goal_preempt_request_ = false;

          //if the user has defined a goal callback, we'll call it now
          if(goal_callback_)
            goal_callback_();
        }
        else{
          //the goal requested has already been preempted by a different goal, so we're not going to execute it
          goal.setCanceled();
        }
      }

      /**
       * @brief  Callback for when the ActionServer receives a new preempt and passes it on
       */
      void preemptCallback(GoalHandle preempt){
        boost::mutex::scoped_lock(lock_);
        ROS_DEBUG("A preempt has been received by the SingleGoalActionServer");

        //if the preempt is for the current goal, then we'll set the preemptRequest flag and call the user's preempt callback
        if(preempt == current_goal_){
          ROS_DEBUG("Setting preempt_request bit for the current goal to TRUE and invoking callback");
          preempt_request_ = true;

          //if the user has registered a preempt callback, we'll call it now
          if(preempt_callback_)
            preempt_callback_();
        }
        //if the preempt applies to the next goal, we'll set the preempt bit for that
        else if(preempt == next_goal_){
          ROS_DEBUG("Setting preempt request bit for the next goal to TRUE");
          new_goal_preempt_request_ = true;
        }
      }

      boost::shared_ptr<ActionServer<ActionSpec> > as_;

      GoalHandle current_goal_, next_goal_;

      bool new_goal_, preempt_request_, new_goal_preempt_request_;

      boost::recursive_mutex lock_;

      boost::function<void ()> goal_callback_;
      boost::function<void ()> preempt_callback_;

  };
};
#endif
