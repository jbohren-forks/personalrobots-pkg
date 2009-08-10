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
#ifndef ACTION_LIB_SINGLE_GOAL_ACTION_SERVER_H_
#define ACTION_LIB_SINGLE_GOAL_ACTION_SERVER_H_
namespace actionlib {
  template <class ActionSpec>
  SingleGoalActionServer<ActionSpec>::SingleGoalActionServer(ros::NodeHandle n, std::string name)
    : new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false) {

      //create the action server
      as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n, name,
            boost::bind(&SingleGoalActionServer::goalCallback, this, _1),
            boost::bind(&SingleGoalActionServer::preemptCallback, this, _1)));

    }

  template <class ActionSpec>
  boost::shared_ptr<const typename SingleGoalActionServer<ActionSpec>::Goal> SingleGoalActionServer<ActionSpec>::acceptNewGoal(){
    boost::recursive_mutex::scoped_lock lock(lock_);

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

  template <class ActionSpec>
  bool SingleGoalActionServer<ActionSpec>::isNewGoalAvailable(){
    return new_goal_;
  }


  template <class ActionSpec>
  bool SingleGoalActionServer<ActionSpec>::isPreemptRequested(){
    return preempt_request_;
  }

  template <class ActionSpec>
  bool SingleGoalActionServer<ActionSpec>::isActive(){
    if(!current_goal_.getGoal())
      return false;
    unsigned int status = current_goal_.getGoalStatus().status;
    return status == GoalStatus::ACTIVE || status == GoalStatus::PREEMPTING;
  }

  template <class ActionSpec>
  void SingleGoalActionServer<ActionSpec>::setSucceeded(const Result& result){
    boost::recursive_mutex::scoped_lock lock(lock_);
    current_goal_.setSucceeded(result);
  }

  template <class ActionSpec>
  void SingleGoalActionServer<ActionSpec>::setAborted(const Result& result){
    boost::recursive_mutex::scoped_lock lock(lock_);
    current_goal_.setAborted(result);
  }

  template <class ActionSpec>
  void SingleGoalActionServer<ActionSpec>::setPreempted(const Result& result){
    boost::recursive_mutex::scoped_lock lock(lock_);
    ROS_DEBUG("Setting the current goal as canceled");
    current_goal_.setCanceled(result);
  }

  template <class ActionSpec>
  void SingleGoalActionServer<ActionSpec>::registerGoalCallback(boost::function<void ()> cb){
    goal_callback_ = cb;
  }

  template <class ActionSpec>
  void SingleGoalActionServer<ActionSpec>::registerPreemptCallback(boost::function<void ()> cb){
    preempt_callback_ = cb;
  }

  template <class ActionSpec>
  void SingleGoalActionServer<ActionSpec>::goalCallback(GoalHandle goal){
    boost::recursive_mutex::scoped_lock lock(lock_);
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

  template <class ActionSpec>
  void SingleGoalActionServer<ActionSpec>::preemptCallback(GoalHandle preempt){
    boost::recursive_mutex::scoped_lock lock(lock_);
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
};
#endif
