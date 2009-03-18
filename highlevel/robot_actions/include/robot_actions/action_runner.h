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

#ifndef H_robot_actions_ActionRunner
#define H_robot_actions_ActionRunner

#include <robot_actions/action.h>
#include <robot_actions/ActionStatus.h>
#include <boost/thread.hpp>
#include <ros/node.h>
#include <ros/console.h>
#include <boost/thread.hpp>

namespace robot_actions {

  /**
   * @brief A class that provides a container for running an action. 
   * 
   * This class provides
   * - A call back for goal message handling
   * - A call back for preemption message handling
   * - A thread for polling the action for status at a desired frequency.
   *
   * The class is a template base class that assumes the state message published contains
   * - ActionStatus status
   * - Goal goal
   * - Feedback feedback
   *
   * An instance of this class will subscribe to a goal and preemption message, and will publish a state message.
   * @todo Check proper values for QUEUE_MAX
   * @todo Modify this so an action connector can be used for the call backs and stick them on a stack so we can iterate over them with a single thread.
   */
  template <class Goal, class State, class Feedback> class ActionRunner {

  public:

    /**
     * @brief Constructor
     * @param actionName The name for the action. This is used to scope the request, premption and notify topics. All other parameters are obtained
     * via ROS paramaters
     */
  ActionRunner(Action<Goal, Feedback>& action)
    : _action(action), _initialized(false), _terminated(false), _update_rate(10), _update_thread(NULL), _update_topic(_action.getName() + "/state_update") {

      // Connect the action to this container
      _action.connect(boost::bind(&ActionRunner<Goal, State, Feedback>::notify, this, _1, _2, _3));

      // Obtain paramaters from the param server
      ros::Node::instance()->param(_action.getName() + "/update_rate", _update_rate, _update_rate);
      ROS_ASSERT(_update_rate > 0);

      // Advertize state updates
      ros::Node::instance()->advertise<State>(_update_topic, QUEUE_MAX());

      // Subscribe to goal requests. 
      ros::Node::instance()->subscribe(_action.getName() + "/activate", _request_msg, &ActionRunner<Goal, State, Feedback>::requestHandler,  this, 1);

      // Subscribe to goal premptions.
      ros::Node::instance()->subscribe(_action.getName() + "/preempt", _premption_msg, &ActionRunner<Goal, State, Feedback>::premptionHandler,  this, 1);

      // Initially inactive
      this->_state_msg.status.value = this->_state_msg.status.UNDEFINED;

      // Start the action_runner_thread
      _update_thread = new boost::thread(boost::bind(&ActionRunner<Goal, State, Feedback>::run, this));
    }

    virtual ~ActionRunner(){
      terminate();
      delete _update_thread;
    }

    void notify(const ActionStatus&, const Goal&, const Feedback&){}

    /**
     * @brief Call when ready to run the node
     */
    void initialize(){
      _initialized = true;
    }

    /**
     * @brief Call this to permanently decommision the controller
     */
    void terminate() {
      // Only do anything the first time
      if(_terminated)
	return;

      ROS_INFO("Terminating %s", _action.getName().c_str());

      _terminated = true;

      // Preempt all actions
      _action.preempt();

      // We must wait till it becomes inactive, and terminates the thread loop
      _update_thread->join();
    }

    /**
     * @brief Test if the we have  received required inputs allowing it to commence business as usual.
     * @see initialize()
     */
    bool isInitialized() const {
      return _initialized;
    }

    /**
     * @brief Test if the HLC has been terminated (decommissioned permanently)
     */
    bool isTerminated() const {
      return _terminated;
    }

  protected:

    /**
     * @brief A subclass will call this method when it has been activated successfully
     * @param Feedback to provide in the state update
     */
    virtual void notifyActivated(const Feedback& feedback_msg){
      ROS_INFO("Activated %s", _action.getName().c_str());
      publishUpdate(feedback_msg, this->_state_msg.status.ACTIVE);
    }

    /**
     * @brief A sublcass will call this method when it has completed successfully.
     * @param Feedback to provide in the state update
     */
    virtual void notifySucceded(const Feedback& feedback_msg){
      ROS_INFO("Completed %s", _action.getName().c_str());
      publishUpdate(feedback_msg, this->_state_msg.status.SUCCESS);
    }

    /**
     * @brief A sublcass will call this method when it has aborted of its own volition
     * @param Feedback to provide in the state update
     */
    virtual void notifyAborted(const Feedback& feedback_msg){
      ROS_INFO("Aborted %s", _action.getName().c_str());
      publishUpdate(feedback_msg, this->_state_msg.status.ABORTED);
    }

    /**
     * @brief A sublcass will call this method when it has successfully been preempted
     * @param Feedback to provide in the state update
     */
    virtual void notifyPreempted(const Feedback& feedback_msg){
      ROS_INFO("Preempted %s", _action.getName().c_str());
      publishUpdate(feedback_msg, this->_state_msg.status.PREEMPTED);
    }

    /**
     * @brief Used to set queue max parameter for subscribe and advertise of ROS topics
     */
    static unsigned int QUEUE_MAX(){return 10;}

  private:

    /**
     * @brief Handle a new request.
     */
    void requestHandler(){
      if(isInitialized() && !isTerminated()){
	_action.activate(_request_msg);
      }
    }

    /**
     * @brief Handle a premption. Will ignore the request unless the action is currently active. Delegates to subclass for details.
     */
    void premptionHandler(){
      if(isInitialized())
	_action.preempt();
    }

    /**
     * @brief The main run loop of the controller
     */
    void run(){
    
      while(ros::Node::instance()->ok()) {

	ros::Time curr = ros::Time::now();
	
	// Guard with initialization check to prevent sending bogus state messages.
	if(isInitialized()){

	  // Update the action status
	  _action.updateStatus();

	  State state_msg;
	  _action.getState(state_msg.status, state_msg.goal, state_msg.feedback);
	  ros::Node::instance()->publish(_update_topic, state_msg);

	  if(isTerminated() && state_msg.status.value != ActionStatus::ACTIVE)
	    break;
	}
 

	sleep(curr, 1 / _update_rate);
      }

      ROS_INFO("Terminating update loop for %s\n", _action.getName().c_str());
    }

    /**
     * @brief Sleep for remaining time of the cycle
     */
    void sleep(ros::Time loopstart, double loopDuration)
    {
      ros::Time curr = ros::Time::now();
      ros::Duration cycleTime;
      cycleTime = cycleTime.fromSec(loopDuration);
      ros::Time desiredCycleEnd = loopstart + cycleTime;

      ros::Duration diff = desiredCycleEnd - curr;
    
      if(diff <= ros::Duration()){
	ROS_DEBUG("Missed deadline and not sleeping; check machine load. Started %f, ended %f, wanted end %f. Wanted delta %f, got %f. Tryed to correct %f sec.\n", 
		  loopstart.toSec(), curr.toSec(), desiredCycleEnd.toSec(), cycleTime.toSec(), (curr - loopstart).toSec(), diff.toSec());
      } else {
	diff.sleep();
      }
    }
    
    void publishUpdate(const Feedback& feedback_msg, const int8_t& status){
      this->_state_msg.feedback = feedback_msg;
      this->_state_msg.status.value = status;
      ros::Node::instance()->publish(_update_topic, this->_state_msg);
    }
    
    /** DATA MEMBERS **/
    Action<Goal, Feedback>& _action; /*! The action to do the real work */
    bool _initialized; /*!< Marks if the node has been initialized, and is ready for use. */
    bool _terminated; /*!< Marks if the node has been terminated. */
    double _update_rate; /*! Update rate for the update thread */
    boost::thread* _update_thread; /*!< Thread running the planner loop */
    const std::string _update_topic;
    Goal _request_msg; /*!< Message populated by handler for a request */
    Goal _premption_msg; /*!< Message pupulated by handler for a premption */
    State _state_msg; /*!< Message published. */
  };

}
#endif
