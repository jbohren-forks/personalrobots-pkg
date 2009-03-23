/*********************************************************************
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

// Author Conor McGann (mcgann@willowgarage.com)

#ifndef H_robot_actions_Action
#define H_robot_actions_Action

#include <robot_actions/ActionStatus.h>
#include <boost/bind.hpp>

namespace robot_actions {

  /**
   * @brief Abstract base class for a durative, preemptable action.
   */
  template<class Goal, class Feedback> class Action {

  protected:

    /**
     * @brief This method is called on receipt of a new goal. The derived class will implement this method to 
     * pursue the goal.
     * @param Goal The goal to accomplish.
     * @see notifyActivated
     */
    virtual void handleActivate(const Goal& goalMsg) = 0;

    /**
     * @brief This method is called to preempt execution of a goal. The derived class must terminate all activiity
     * pursuing an active goal. As soon as we return from this call, the actor will be deactivated.
     * @see notifyPreempted
     */
    virtual void handlePreempt() = 0;

    /**
     * @brief A method call periodically - allows the action to provide intermediate feedback.
     */
    virtual void handleUpdate(Feedback& feedback){}

  public:

    /**
     * @brief This method is called on receipt of a new goal. The derived class will implement this method to 
     * pursue the goal. The actor we be activated as soon as notifyActivated is called on the actioncontainer.
     * @param Goal The goal to accomplish.
     * @see notifyActivated
     */
    void activate(const Goal& goal){
      ROS_DEBUG("%s received request to activate.", getName().c_str());

      if(!isActive()){
	_goal = goal;
	handleActivate(_goal);
      }
    }

    /**
     * @brief This method is called to preempt execution of a goal. The derived class must terminate all activiity
     * pursuing an active goal. The actor will be deactivated as soon as notifyPreempted is called on the actioncontainer.
     * @see notifyPreempted
     */
    void preempt(){
      ROS_DEBUG("%s received request to preempt.", getName().c_str());
      if(isActive())
	handlePreempt();
    }

    /**
     * @brief A call made periodically to provide an opportunity to execute. 
     */
    void update(){

      // Allow for action to update
      if(isActive())
	handleUpdate(_feedback);

      // Always post a state update
      _callback(_status, _goal, _feedback);
    }

    /**
     * @brief Accessor for the action name
     */
    const std::string& getName() const { return _name; }

    /**
     * @brief Connect the action to a container for handling outbound messages
     * @todo Implement with a functor object and bost bind perhaps
     */
    void connect(const boost::function< void(const ActionStatus&, const Goal&, const Feedback&) >& callback){ _callback = callback; }

  protected:

    /** The notification methods below are used by derived action classes to generate state transition events **/

    /**
     * @brief An action will call this method when it has been activated successfully
     * @param Feedback to provide in the state update
     */
    void notifyActivated(){
      ROS_DEBUG("%s activated.", getName().c_str());

      _status.lock();
      if(!isActive()){
	_status.value = ActionStatus::ACTIVE;
	_callback(_status, _goal, _feedback);
      }
      _status.unlock();
    }

    /**
     * @brief An action will call this method when it has completed successfully.
     * @param Feedback to provide in the state update
     */
    void notifySucceeded(const Feedback& feedback){
      ROS_DEBUG("%s completed successfully.", getName().c_str());
      handleDeactivation(feedback, ActionStatus::SUCCESS);
    }

    /**
     * @brief An action will call this method when it has aborted of its own volition
     * @param Feedback to provide in the state update
     */
    void notifyAborted(const Feedback& feedback){
      ROS_DEBUG("%s aborted.", getName().c_str());
      handleDeactivation(feedback, ActionStatus::ABORTED);
    }

    /**
     * @brief An action will call this method when it has successfully been preempted
     * @param Feedback to provide in the state update
     */
    void notifyPreempted(const Feedback& feedback){
      ROS_DEBUG("%s preempted.", getName().c_str());
      handleDeactivation(feedback, ActionStatus::PREEMPTED);
    }

    /**
     * @brief Constructor
     * @param name The action name
     */
    Action(const std::string& name)
    : _name(name) {
      _status.value = ActionStatus::UNDEFINED; 
    }

    virtual ~Action(){}

    /**
     * @brief True if the action is active, false otherwise
     */
    bool isActive() const {
      return _status.value == ActionStatus::ACTIVE;
    }

  private:

    void handleDeactivation(const Feedback& feedback, int8_t status_flag){
      // Only do work if we are active. Need to lock protect this
      _status.lock();
      if(isActive()){
	_feedback = feedback;
	_status.value = status_flag;
	_callback(_status, _goal, _feedback);
      }
      _status.unlock();
    }

    const std::string _name; /*!< Name for the action */
    boost::function< void(const ActionStatus&, const Goal&, const Feedback&) > _callback; /*!< Callback function for sending updates */
    ActionStatus _status;
    Goal _goal;
    Feedback _feedback;
  };

}
#endif
