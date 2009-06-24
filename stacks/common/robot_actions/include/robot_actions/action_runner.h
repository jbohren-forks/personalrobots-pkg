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

#include <robot_actions/message_adapter.h>
#include <boost/thread.hpp>
#include <ros/console.h>

namespace robot_actions {

  /**
   * @brief A container class for integrating actions with ROS messages and running them on a periodic update loop
   */
  class ActionRunner {

  public:

    /**
     * @brief Constructor
     * @param update_rate The frequency, in Hz, that we waant to run actions at
     */
    ActionRunner(double update_rate);


    /**
     * @brief Destructor will have to terminate all actions and wait for the update thread to wind down.
     */
    ~ActionRunner();


    /**
     * @brief Connects an action to be run according to the given update rate, and within a ROS message context
     * @see MessageAdapter
     * @note The ActionRunner should not be running when still connecting actions
     */   
    template<class Goal, class State, class Feedback> void connect(Action<Goal, Feedback>& action){

      if(_initialized){
	ROS_WARN("Tried to add action %s after action runner was already started.", action.getName().c_str());
	return;
      }

      MessageAdapter<Goal, State, Feedback>* adapter = new MessageAdapter<Goal, State, Feedback>(action);
      _adapters.push_back(adapter);
    }
  
    /**
     * @brief Call this to permanently decommision the controller
     */
    void terminate();
  
    /**
     * @brief Test if the HLC has been terminated (decommissioned permanently)
     */
    bool isTerminated() const;

    /**
     * @brief Invoke when you want to run the connected actions.
     * @see connect
     */
    void run();

  private:

    /**
     * @brief The main run loop will update all contained actions
     */
    void updateLoop();
  
    /**
     * @brief Sleep for remaining time of the cycle
     */
    void sleep(ros::Time loopstart, double loopDuration);

    bool _initialized; /*!< Marks that the ndoe is ready to run */
    bool _terminated; /*!< Marks if the node has been terminated. */
    double _update_rate; /*!< The duration for each control cycle */
    boost::thread* _update_thread; /*!< Thread running the planner loop */
    std::vector<AbstractAdapter*> _adapters; /*!< Collection of action adapters to run */
  };
}

#endif
