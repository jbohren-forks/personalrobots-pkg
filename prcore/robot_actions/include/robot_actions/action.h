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

/**
 * @mainpage
 * 
 * The robot_actions package provides a construct for goal achieving, modular behavior primitives that are non-blocking
 * for the client, and preemptable. This is in contrast to a service which is not preemptable and blocks the client. 
 *
 * @htmlinclude manifest.html
 *
 * @author Conor McGann
 */

#ifndef H_robot_actions_Action
#define H_robot_actions_Action

#include <robot_actions/ActionStatus.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace robot_actions {

  /**
   * @brief Result codes for execution of an action. All values are sub states of the inactive state for an action.
   * @see ActionStatus
   */
  enum ResultStatus {
    SUCCESS = 1,
    ABORTED = 2,
    PREEMPTED = 3
  };

  /**
   * @brief Abstract base class for a durative, preemptable action.
   */
  template<class Goal, class Feedback> class Action {

  protected:

    /**
     * @brief Blocking, user specified code for the action. This function should check periodically if isPreemptRequested is true, and if so, exit.
     * @param goal The goal message currently associated with the controller.
     * @param feedback The feedback message. At the end of the function, this will be published.
     */
    virtual ResultStatus execute(const Goal& goal, Feedback& feedback) = 0;

    /**
     * @brief Allows the user to check if the action is active. This is primarily used to condition behavior
     * for low duty cycles when the action is inactive
     * @return True if the action is active, false otherwise.
     */
    bool isActive() const { return _status.value == _status.ACTIVE; }
    
    /**
     * @brief Called by the user to check if the action has a new goal.
     * @return True if a request to preempt is made, false otherwise.
     */
    bool isPreemptRequested() const { return _preempt_request; }

    /**
     * @brief Called by the user when a new feedback message is ready to be published.
     * @param feedback The feedback message.
     */
    void update(const Feedback& feedback) {
      // Update local feedback state
      _goal.lock();
      _feedback = feedback;
      _goal.unlock();

      makeCallback(_status, _goal, feedback);
    }

  public:

    /**
     * @brief Accessor for the action name
     */
    const std::string& getName() const { return _name; }

    /**
     * @brief Preempts the controller. Blocks until it preempts.
     */
    void preempt() {
      _preempt_request = true; 
      ROS_DEBUG("[%s]Preempt requested\n", getName().c_str());
      while(isActive()) {
	ros::Duration d; d.fromSec(0.001);
	d.sleep();
      }
      ROS_DEBUG("[%s]Preempt achieved\n", getName().c_str());
      _preempt_request = false; 
    }

    /**
     * @brief Activates the controller.
     */
    void activate(const Goal& goal) {
      if (isActive()) {
	ROS_DEBUG("[%s]New goal forcing preemption of current active goal\n", getName().c_str());
	preempt();
      }

      // Set the goal and activate
      ROS_DEBUG("[%s]Setting new goal.", getName().c_str());
      _goal.lock();
      _goal = goal;
      _goal.unlock();
      _status.value = _status.ACTIVE;
      update(Feedback());
    }

    /**
     * @brief Connect the action to a container for handling outbound messages
     * @todo Implement with a functor object and bost bind perhaps
     */
    void connect(const boost::function< void(const ActionStatus&, const Goal&, const Feedback&) >& callback){ _callback = callback; }

    /**
     * @brief Call to terminate an action - prevent it from runLoopning again.
     */
    void terminate() { 
      _terminated = true;

      if(isActive()){
	preempt();
      }
    }

    /**
     * @brief Called by an external client to publish an update
     */
    void publish(){
      makeCallback(_status, _goal, _feedback);
    }

  protected:

    /**
     * @brief Constructor
     * @param name The action name
     */
    Action(const std::string& name)
      : _name(name), _preempt_request(false), _result_status(SUCCESS), _terminated(false), _action_thread(NULL), _callback(NULL){
      _status.value = ActionStatus::UNDEFINED; 
      _action_thread = new boost::thread(boost::bind(&Action<Goal, Feedback>::runLoop, this));
    }

    virtual ~Action(){
      terminate();
      _action_thread->join();
      delete _action_thread;
    }


    /**
     * @brief Called by the derived class to deactivate the node. Used when actions leverage call
     * backs for processing. In  that case the derived class will have to implement a busy loop in the execute
     * method.
     */
    void deactivate(const ResultStatus& result_status, const Feedback& feedback){
      if(!isActive()){
	ROS_DEBUG("[%s]Tried to deeactivate when already deactivated.\n", getName().c_str());
      }

      _result_status = result_status;
      _status.value = result_status;
      _goal.lock();
      _feedback = feedback;
      _goal.unlock();
      ROS_DEBUG("[%s]Deactivated\n", getName().c_str());
    }

    /**
     * @brief This method is defined for derived class implementations that deactivate via callbacks or other threads. It
     * provides a simple blocking pattern for implementing execute method.
     */
    ResultStatus waitForDeactivation(Feedback& feedback){
      ROS_DEBUG("[%s]Waiting for completion\n", getName().c_str());
      ros::Duration d; d.fromSec(0.001);
      while(isActive()){
	d.sleep();
      }

      // Write feedback
      _goal.lock();
      feedback = _feedback;
      _goal.unlock();

      ROS_DEBUG("[%s]Completed\n", getName().c_str());
      return _result_status;
    }

  private:

    void runLoop() {
      ROS_DEBUG("[%s]Run loop enabled", _name.c_str());
      ros::Duration d; 
      d.fromSec(0.010);
      while (!_terminated){
	if(isActive()){
	  Feedback end_result;
	  ROS_DEBUG("[%s]Starting execution", _name.c_str());
	  _status.value = execute(_goal, end_result);
	  ROS_DEBUG("[%s]Completed execution", _name.c_str());
	  update(end_result);
	}

	d.sleep();
      }
      ROS_DEBUG("[%s]Run loop disabled", _name.c_str());
    }

    void makeCallback(const ActionStatus& status, const Goal& goal, const Feedback& feedback){
      if(_callback != NULL)
	_callback(status, goal, feedback);
      else
	ROS_WARN("No callback registered for action %s", _name.c_str());
    }


    const std::string _name; /*!< Name for the action */
    bool _preempt_request; /*!< True when preemption has been requested. */
    ActionStatus _status; /*!< The current action status */
    ResultStatus _result_status; /*!< The current action status */
    Goal _goal; /*!< The current goal, if the action is active */
    Feedback _feedback; /*!< Current feedback. May or may not get updated on interim updates. */
    bool _terminated;
    boost::thread* _action_thread; /*!< Thread running the action */
    boost::function< void(const ActionStatus&, const Goal&, const Feedback&) > _callback; /*!< Callback function for sending updates */
  };
}
#endif
