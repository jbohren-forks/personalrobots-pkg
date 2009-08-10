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
#ifndef ACTIONLIB_GOAL_HANDLE_IMP_H_
#define ACTIONLIB_GOAL_HANDLE_IMP_H_
namespace actionlib {
  template <class ActionSpec>
  ActionServer<ActionSpec>::GoalHandle::GoalHandle(){}

  template <class ActionSpec>
  void ActionServer<ActionSpec>::GoalHandle::setAccepted(){
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

  template <class ActionSpec>
  void ActionServer<ActionSpec>::GoalHandle::setCanceled(const Result& result){
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

  template <class ActionSpec>
  void ActionServer<ActionSpec>::GoalHandle::setRejected(const Result& result){
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

  template <class ActionSpec>
  void ActionServer<ActionSpec>::GoalHandle::setAborted(const Result& result){
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

  template <class ActionSpec>
  void ActionServer<ActionSpec>::GoalHandle::setSucceeded(const Result& result){
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

  template <class ActionSpec>
  void ActionServer<ActionSpec>::GoalHandle::publishFeedback(const Feedback& feedback){
    ROS_DEBUG("Publishing feedback for goal, id: %.2f, stamp: %.2f", getGoalID().id.toSec(), getGoalID().stamp.toSec());
    if(goal_) {
      as_->publishFeedback((*status_it_).status_, feedback);
    }
    else
      ROS_ERROR("Attempt to publish feedback on an uninitialized GoalHandle");
  }

  template <class ActionSpec>
  boost::shared_ptr<const typename ActionServer<ActionSpec>::Goal> ActionServer<ActionSpec>::GoalHandle::getGoal() const{
    //if we have a goal that is non-null
    if(goal_){
      //create the deleter for our goal subtype
      EnclosureDeleter<const ActionGoal> d(goal_);
      return boost::shared_ptr<const Goal>(&(goal_->goal), d);
    }
    return boost::shared_ptr<const Goal>();
  }

  template <class ActionSpec>
  GoalID ActionServer<ActionSpec>::GoalHandle::getGoalID() const{
    if(goal_)
      return (*status_it_).status_.goal_id;
    else{
      ROS_ERROR("Attempt to get a goal id on an uninitialized GoalHandle");
      return GoalID();
    }
  }

  template <class ActionSpec>
  GoalStatus ActionServer<ActionSpec>::GoalHandle::getGoalStatus() const{
    if(goal_)
      return (*status_it_).status_;
    else{
      ROS_ERROR("Attempt to get goal status on an uninitialized GoalHandle");
      return GoalStatus();
    }
  }

  template <class ActionSpec>
  bool ActionServer<ActionSpec>::GoalHandle::operator==(const GoalHandle& other){
    if(!goal_ || !other.goal_)
      return false;
    GoalID my_id = getGoalID();
    GoalID their_id = other.getGoalID();
    return my_id.id == their_id.id;
  }

  template <class ActionSpec>
  bool ActionServer<ActionSpec>::GoalHandle::operator!=(const GoalHandle& other){
    if(!goal_ || !other.goal_)
      return true;
    GoalID my_id = getGoalID();
    GoalID their_id = other.getGoalID();
    return my_id.id != their_id.id;
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::GoalHandle::GoalHandle(typename std::list<StatusTracker>::iterator status_it,
      ActionServer<ActionSpec>* as, boost::shared_ptr<void> handle_tracker)
    : status_it_(status_it), goal_((*status_it).goal_),
    as_(as), handle_tracker_(handle_tracker){}

  template <class ActionSpec>
  bool ActionServer<ActionSpec>::GoalHandle::setCancelRequested(){
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
};
#endif
