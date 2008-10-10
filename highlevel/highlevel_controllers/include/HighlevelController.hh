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

#ifndef H_HighlevelController

#include <ros/node.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <iostream>

/**
 * @brief Abstract base class for a high level controller node which is parameterized by the state update
 * and goal messages exchanged with a client. The controller utilizes a pattern where the controller is tasked with
 * goals, and can transition between an active state (when pursuing a goal), and an inactive state when is its effectively
 * idle and should not be imposing any control. A high level control must also handle goal recalls.
 */
template <class S, class G> class HighlevelController: public ros::node {
public:
  enum State {
    INACTIVE = 0,
    ACTIVE
  };

  /**
   * @brief Used to set queue max parameter for subscribe and advertise of ROS topics
   */
  static unsigned int QUEUE_MAX(){return 10;}

  /**
   * @brief Constructor
   * @param nodeName The name for the node, which will appear in botherder
   * @param stateTopic The ROS topic on which controller state update messages are published
   * @param goalTopic The ROS topic on which controller goals are received
   * @param cycleTime Determines the control loop frequency in Hz (frequency = 1/cycleTime)
   */
  HighlevelController(const std::string& nodeName, const std::string& _stateTopic,  const std::string& _goalTopic, double _cycleTime = 0.1): 
    ros::node(nodeName), initialized(false), stateTopic(_stateTopic), goalTopic(_goalTopic), cycleTime(_cycleTime){

    // Advertize controller state updates
    advertise<S>(stateTopic, QUEUE_MAX());

    // Subscribe to controller goal requests
    subscribe(goalTopic, goalMsg, &HighlevelController<S, G>::goalCallback, QUEUE_MAX());

    // Initially inactive
    deactivate();
  }

  virtual ~HighlevelController(){}  

  /**
   * @brief Test if the node has received required inputs allowing it to commence business as usual.
   * @see initialize()
   */
  bool isInitialized(){
    lock();
    bool result =  initialized;
    unlock();
    return result;
  }

  /**
   * @brief The main run loop of the controller
   */
  void run(){
    while(ok())
      {
	struct timeval curr;
	gettimeofday(&curr,NULL);
	
	// Guard with initialization check to prevent sending bogus state messages.
	if(isInitialized()){
	  doOneCycle();
	  publish(stateTopic, this->stateMsg);
	}
	else {
	}
 
	sleep(curr.tv_sec+curr.tv_usec/1e6);
      }
  }

protected:

  /**
   * @brief Accessor for state of the controller
   */
  bool isActive() {
    lock();
    bool result = this->state == ACTIVE;
    unlock();
    return result;
  }

  /**
   * @brief Access for valid status of the controller
   */
  bool isValid() {
    lock();
    bool result =  this->stateMsg.valid;
    unlock();
    return result;
  }

  /**
   * @brief Marks the node as initialized. Shoud be called by subclass when expected inbound messages
   * are received to make sure it only publishes meaningful states
   */
  void initialize(){
    lock();
    initialized = true;
    unlock();
  }


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
   * @brief return true if plan is still valid, otherwise return false.
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
   * @brief Aquire node level lock
   */
  void lock(){lock_.lock();}

  /**
   * @brief Release node level lock
   */
  void unlock(){lock_.unlock();}

  G goalMsg; /*!< Message populated by callback */
  S stateMsg; /*!< Message published. Will be populated in the control loop */

private:

  void goalCallback(){
    lock();
    if(state == INACTIVE && goalMsg.enable){
      activate();
    }
    else if(state == ACTIVE){
      deactivate();

      // If we are active, and this is a goal, publish the state message and activate. This allows us
      // to over-ride a new goal, but still forces the transition between active and inactive states
      if(goalMsg.enable){
	publish(stateTopic, stateMsg);
	activate();
      }
    }

    unlock();

    // Call to allow derived class to update goal member variables
    updateGoalMsg();
  }

  /**
   * @brief Activation of the controller will set the state, the stateMsg but indicate that the
   * goal has not yet been accomplished and that no plan has been constructed yet.
   */
  void activate(){
    std::cout << "Activating controller\n";
    this->state = ACTIVE;
    this->stateMsg.active = 1;
    this->stateMsg.valid = 0;
    this->stateMsg.done = 0;

    handleActivation();
  }

  /**
   * @brief Deactivation of the controller will set the state to inactive, and clear the valid flag.
   */
  void deactivate(){
    std::cout << "Deactivating controller\n";
    this->state = INACTIVE;
    this->stateMsg.active = 0;
    this->stateMsg.valid = 0;

    handleDeactivation();
  }

  /**
   * @brief Sleep for remaining time of the cycle
   */
  void sleep(double loopstart)
  {
    struct timeval curr;
    double currt,tdiff;
    gettimeofday(&curr,NULL);
    currt = curr.tv_sec + curr.tv_usec/1e6;
    tdiff = this->cycleTime - (currt-loopstart);
    if(tdiff <= 0.0)
      std::cout << "Missed deadline and not sleeping; check machine load\n";
    else
      usleep((unsigned int)rint(tdiff*1e6));
  }

  /**
   * @brief The main control loop. Locking happens before and after
   */
  void doOneCycle(){

    // Update the state message with suitable data from inputs
    updateStateMsg();

    // Publish a response reflecting the state for this cycle. The state may change
    // after this execution, but publishing it here ensures we get a message where the state
    // is active, even if it transitions in the first cycle to an inactive state
    publish(stateTopic, this->stateMsg);

    // If not enabled, then nothing further to do
    if(!isActive())
      return;

    // If we have a new goal, we must make a plan
    if(!isValid()){
      setValid(makePlan());
    }

    // If we are pursuing a goal, and thus we have a plan, we should check for
    // the goal being reached, in which case we update the state
    if(isValid()){
      if(goalReached()){
	setDone(true);
	deactivate();
      }
      else {
	// Dispatch plans to accomplish goal, according to the plan. If there is a failure in 
	// dispatching, it should return false, which will force re-planning
	setValid(dispatchCommands());
      }
    }
  }

  /**
   * @brief Setter for state msg done flag
   */
  void setDone(bool isDone){
    lock();
    this->stateMsg.done = isDone;
    unlock();
  }

  /**
   * @brief Setter for state msg valid flag
   */
  void setValid(bool isValid){
    lock();
    this->stateMsg.valid = isValid;
    unlock();
  }

  /**
   * @brief Will evaluate if we have received an update on the goal state and will transition the controller state
   * accordingly. An update is where the logged state is not the same as the desired state.
   */
  void updateStateForGoal(){
  }

  bool initialized; /*!< Marks if the node has been initialized, and is ready for use. */
  const std::string stateTopic; /*!< The topic on which state updates are published */
  const std::string goalTopic; /*!< The topic on which it subscribes for goal requests and recalls */
  State state; /*!< Tracks the overall state of the controller */
  const double cycleTime; /*!< The duration for each control cycle */
  ros::thread::mutex lock_; /*!< Lock for access to class members in callbacks */
};

#endif
