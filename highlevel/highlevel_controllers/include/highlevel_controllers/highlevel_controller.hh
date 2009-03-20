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

#ifndef H_highlevel_controller_HighlevelController
#define H_highlevel_controller_HighlevelController

#include <ros/node.h>
#include <boost/thread.hpp>
#include <ros/console.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <highlevel_controllers/Ping.h>

/**
 * @brief Abstract base class for a high level controller node which is parameterized by the state update
 * and goal messages exchanged with a client. The controller utilizes a pattern where the controller is tasked with
 * goals, and can transition between an active state (when pursuing a goal), and an inactive state when is its effectively
 * idle and should not be imposing any control. A high level control must also handle goal recalls, which is termed 'preemption'
 */
template <class S, class G> class HighlevelController {
public:


  /**
   * @brief Subclass will implement this method to update goal data based on new values in goalMsg. Derived class should
   * be sure to lock and unlock when accessing member variables
   */
  virtual void updateGoalMsg(){}

  /**
   * @brief Subclass will implement this method to update the published state msssage with data from telemetry input. Derived class should
   * be sure to lock and unlock when accessing member variables
   */
  virtual void updateStateMsg(){}

  /**
   * @brief Subclass will implement this message to generate a plan.
   * @return true if a plan is found, otherwise false
   */
  virtual bool makePlan() = 0;

  /**
   * @brief Subclass will implement this method to test of the goal has been reached
   * @return true if goal reached, otherwise false.
   */
  virtual bool goalReached() = 0;

  /**
   * @brief Subclass will implement this method to generate command messages in order to accomplish its goal
   * The base class encapsulates with a lock/unlock.
   * @return true if plan is still valid, otherwise return false.
   */
  virtual bool dispatchCommands() = 0;

  /**
   * @brief A Hook to catch a deactivation event
   */
  virtual void handleDeactivation() {}

  /**
   * @brief A Hook to catch an activation event
   */
  virtual void handleActivation(){}

  /**
   * @brief A hook to handle the case when global planning fails
   */
  virtual void handlePlanningFailure(){}

  virtual ~HighlevelController(){
    terminate();
    delete plannerThread_;
  }


  /**
   * @brief The main run loop of the controller
   */
  void run(){
    while(ros::Node::instance()->ok()  && !isTerminated()) {
      ros::Time curr = ros::Time::now();
	
      // Guard with initialization check to prevent sending bogus state messages.
      if(isInitialized()){
	doOneCycle();

	ros::Node::instance()->publish(stateTopic, this->stateMsg);
      }
 
      sleep(curr, controllerCycleTime_);
    }
  }

protected:

  /**
   * @brief Constructor
   * @param nodeName The name for the node, which will appear in botherder
   * @param stateTopic The ROS topic on which controller state update messages are published
   * @param goalTopic The ROS topic on which controller goals are received
   */
  HighlevelController(const std::string& nodeName, const std::string& _stateTopic,  const std::string& _goalTopic): 
    initialized(false), terminated(false), stateTopic(_stateTopic), 
    goalTopic(_goalTopic), controllerCycleTime_(0.1), plannerCycleTime_(0.0), plannerThread_(NULL), 
    lastPlan_(ros::Time::now()), timeout_(0, 0), valid_(false) {

    // Obtain the control frequency for this node
    double controller_frequency(10);
    ros::Node::instance()->param("~move_base/controller_frequency", controller_frequency, controller_frequency);
    ROS_ASSERT(controller_frequency > 0);
    ROS_DEBUG("FREQ: %.4f", controller_frequency);
    controllerCycleTime_ = 1/controller_frequency;

    // Obtain the planner frequency for this node. A negative value means run as fast as possible. A zero value means run
    // on demand. Otherwise, run at the specified positive frequency
    double planner_frequency(0.0);
    ros::Node::instance()->param("~move_base/planner_frequency", planner_frequency, planner_frequency);
    if(planner_frequency  > 0)
      plannerCycleTime_ = 1/planner_frequency;
    else if (planner_frequency < 0)
      plannerCycleTime_ = -1;

    // Advertize controller state updates - do not want to miss a state transition.
    ros::Node::instance()->advertise<S>(stateTopic, QUEUE_MAX());

    // Subscribe to controller goal requests. Last request winds. We drop others
    ros::Node::instance()->subscribe(goalTopic, goalMsg, &HighlevelController<S, G>::goalCallback,  this, 1);

    // Subscribe to executive shutdown signal
    ros::Node::instance()->subscribe("highlevel_controllers/shutdown", shutdownMsg_, &HighlevelController<S, G>::shutdownCallback, this, 1);

    // Initially inactive
    this->stateMsg.status.value = this->stateMsg.status.UNDEFINED;

    deactivate();

    // Start planner loop on a separate thread
    plannerThread_ = new boost::thread(boost::bind(&HighlevelController<S, G>::plannerLoop, this));
  }

  /**
   * @brief Used to set queue max parameter for subscribe and advertise of ROS topics
   */
  static unsigned int QUEUE_MAX(){return 10;}

  /**
   * @brief Test if the HLC has received required inputs allowing it to commence business as usual.
   * @see initialize()
   */
  bool isInitialized() const {
    return initialized;
  }

  /**
   * @brief Call this to permanently decommision the controller
   */
  void terminate() {
    terminated = true;
  }

  /**
   * @brief Test if the HLC has been terminated (decommissioned permanently)
   */
  bool isTerminated() const {
    return terminated;
  }

  /**
   * @brief Accessor for state of the controller
   */
  bool isActive() {
    return this->stateMsg.status.value == this->stateMsg.status.ACTIVE;
  }

  /**
   * @brief Access for valid status of the controller
   */
  bool isValid() {
    return valid_;
  }
  /**
   * @brief Access aborted state of the planner.
   */
  bool isAborted() {
    return this->stateMsg.status.value == this->stateMsg.status.ABORTED;
  }

  /**
   * @brief Access preempted state of the planner.
   */
  bool isPreempted() {
    return this->stateMsg.status.value == this->stateMsg.status.PREEMPTED;
  }

  /**
   * @brief Abort the controller. This is the only state transition that can be explicitly initiated
   * by a derived class. Otherwise the state transitions are governed by implementations of methods
   * for planning and dispatching commands, and checking of a goal has been reached.
   */
  void abort(){
    ROS_INFO("Aborting controller\n");
    this->stateMsg.status.value = this->stateMsg.status.ABORTED;
  }

  /**
   * @brief Marks the node as initialized. Shoud be called by subclass when expected inbound messages
   * are received to make sure it only publishes meaningful states
   */
  void initialize(){
    initialized = true;
  }

  /**
   * @brief Aquire node level lock
   */
  void lock(){lock_.lock();}
  
  /**
   * @brief Release node level lock
   */
  void unlock(){lock_.unlock();}
  
  /**
   * @brief Lock/unlock sentry for HighlevelController and its subclasses.
   * Useful for locking across multiple points of return like this:
   * @code
   * int some_func(HighlevelController * hlc) {
   *   HighlevelController::sentry guard(hlc); // calls hlc->lock()
   *   if (op1())
   *     return 42; // ~sentry calls hlc->unlock()
   *   if (op2()) {
   *     op3();
   *     return 17; // ~sentry calls hlc->unlock()
   *   }
   *   return -1; // ~sentry calls hlc->unlock()
   * }
   * @endcode
   * @note Should me move this sentry class to ros? Seems generally useful.
   */
  template<class T> class sentry {
  public:
    sentry(T * t): t_(t) { t_->lock(); }
    ~sentry() { t_->unlock(); }
  protected:
    T * t_;
  };
  

  G goalMsg; /*!< Message populated by callback */
  S stateMsg; /*!< Message published. Will be populated in the control loop */


private:


  /**
   * @brief Runs the planning loop. If the node is not initialized or if inactive, then it will do nothing. Otherwise
   * it will call the planner with the given timeout.
   */
  void plannerLoop(){

    while(ros::Node::instance()->ok() && !isTerminated()){
      ros::Time curr = ros::Time::now();

      if(isInitialized() && isActive()){

	// If the plannerCycleTime is 0 then we only call the planner when we need to
	if(plannerCycleTime_ != 0 || !isValid()){
	  setValid(makePlan());

	  // Could use a refined locking scheme but for now do not want to delegate that to a derived class
	  lock();

	  if(!isValid()){
  
	    ros::Duration elapsed_time = ros::Time::now() - lastPlan_;
	    if ((elapsed_time > timeout_) && timeout_.toSec() != 0.0){
	      ROS_DEBUG("Last current value at %f seconds:", curr.toSec());
	      ROS_DEBUG("Last plan at %f seconds:", lastPlan_.toSec());
	      ROS_DEBUG("Elapsed time is %f seconds:", elapsed_time.toSec());
	      abort();
	    }

	    handlePlanningFailure();
	  } else {
	    // Reset the planning timeout counter
	    lastPlan_ = ros::Time::now();
	  }

	  unlock();
	}
      }

      if(plannerCycleTime_ >= 0)
	sleep(curr, std::max(plannerCycleTime_, controllerCycleTime_));
      else
	sleep(curr, curr.toSec() + 0.001);
    }
  }

  /**
   * @brief Setter for state msg valid flag
   */
  void setValid(bool isValid){
    valid_ = isValid;
  }

  /**
   * @brief Activation of the controller will set the state, the stateMsg but indicate that the
   * goal has not yet been accomplished and that no plan has been constructed yet.
   */
  void activate(){
    ROS_INFO("Activating controller with timeout of %f seconds\n", this->goalMsg.timeout);

    valid_ = 0;
    this->stateMsg.status.value = this->stateMsg.status.ACTIVE;

    handleActivation();
  }

  void preempt(){
    ROS_INFO("Controller preempted.");
    this->stateMsg.status.value = this->stateMsg.status.PREEMPTED;
    deactivate();
  }

  /**
   * @brief Deactivation of the controller will set the state to inactive, and clear the valid flag.
   */
  void deactivate(){
    ROS_INFO("Deactivating controller");
    handleDeactivation();
  }

  void goalCallback(){
    if (goalMsg.timeout < 0) {
      ROS_ERROR("Controller was given negative timeout, setting to zero.");
      timeout_ = ros::Duration().fromSec(0);
    } else {
      timeout_ = ros::Duration().fromSec(goalMsg.timeout);
    }

    // Do nothing if not initialized
    if(!isInitialized() || isTerminated())
      return;

    lock();

    // reset the timer on planning delay bound
    lastPlan_ = ros::Time::now();

    if(!isActive() && goalMsg.enable){
      activate();
    }
    else if(isActive()){
      preempt();

      // If we are active, and this is a goal, publish the state message and activate. This allows us
      // to over-ride a new goal, but still forces the transition between active and inactive states
      if(goalMsg.enable){
	ROS_DEBUG("Publishing state %d", stateMsg.status.value);
	ros::Node::instance()->publish(stateTopic, stateMsg);
	activate();
      }
    }

    // Call to allow derived class to update goal member variables
    updateGoalMsg();

    // Update the state message with suitable data from inputs
    updateStateMsg();

    // Publish a response reflecting the state for this cycle. The state may change
    // after this execution, but publishing it here ensures we get a message where the state
    // is active, even if it transitions in the first cycle to an inactive state. This can occur for
    // example if the planner returns that there is no plan to be had, for example.
    ros::Node::instance()->publish(stateTopic, this->stateMsg);

    unlock();
  }

  void shutdownCallback(){
    lock();
    ROS_INFO("Shutdown received from executive.\n");
    deactivate();
    unlock();
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

  /**
   * @brief The main control loop. Locking happens before and after
   */
  void doOneCycle(){
    lock();

    // Update the state message with suitable data from inputs
    updateStateMsg();

    // Publish a response reflecting the state for this cycle. The state may change
    // after this execution, but publishing it here ensures we get a message where the state
    // is active, even if it transitions in the first cycle to an inactive state
    ros::Node::instance()->publish(stateTopic, this->stateMsg);

    // If we are in an active state, we want to evalaute what to do whether we have a plan or not. In
    // the latter case, commands may be given to maintain a fail-safe state. The structure here ensures
    // the controller has an opportunity to command appropriately whether in a valid state or not. If this were
    // not the case, for example, on a mobil robot, it could just maintain prior commanded values
    // when the planner invalidates the plan, which can happen since planning is interleaved.
    if(isActive()){
      if(isValid() && goalReached()){
	ROS_INFO("Success! Goal achieved :)\n");
	this->stateMsg.status.value = this->stateMsg.status.SUCCESS;
	deactivate();
      }
      else {
	// Dispatch plans to accomplish goal, according to the plan. If there is a failure in 
	// dispatching, it should return false, which will force re-planning
	setValid(dispatchCommands());
      }
    }

    unlock();
  }


  bool initialized; /*!< Marks if the node has been initialized, and is ready for use. */
  bool terminated; /*!< Marks if the node has been terminated. */
  const std::string stateTopic; /*!< The topic on which state updates are published */
  const std::string goalTopic; /*!< The topic on which it subscribes for goal requests and recalls */
  double controllerCycleTime_; /*!< The duration for each control cycle */
  double plannerCycleTime_; /*!< The duration for each planner cycle. */
  boost::recursive_mutex lock_; /*!< Lock for access to class members in callbacks */
  boost::thread* plannerThread_; /*!< Thread running the planner loop */
  highlevel_controllers::Ping shutdownMsg_; /*!< For receiving shutdown from executive */
  ros::Time lastPlan_; /* The last time a plan was computed. Will be cleared with every new goal */
  ros::Duration timeout_; /*< The time limit for planning failure. */
  bool valid_; /*< True if it is valid */
};

#endif
