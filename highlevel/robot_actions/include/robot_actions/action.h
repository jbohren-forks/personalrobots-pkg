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

#include <robot_actions/action_container.h>

namespace robot_actions {

  /**
   * @brief Abstract base class for a durative, preemptable action.
   * @see ActionContainer Used to connnect the action to its execution context
   */
  template <class Goal, class Feedback> class Action {

  public:

    /**
     * @brief This method is called on receipt of a new goal. The derived class will implement this method to 
     * pursue the goal. The actor we be activated as soon as notifyActivated is called on the actioncontainer.
     * @param Goal The goal to accomplish.
     * @see notifyActivated
     */
    virtual void activate(const Goal& goalMsg) = 0;

    /**
     * @brief This method is called to preempt execution of a goal. The derived class must terminate all activiity
     * pursuing an active goal. The actor will be deactivated as soon as notifyPreempted is called on the actioncontainer.
     * @see notifyPreempted
     */
    virtual void preempt() = 0;

    /**
     * @brief A query to retrieve current state of execution, and offer an opportunity to publish specific 
     * feedback parameters
     */
    virtual void updateStatus(Feedback& feedback_msg) = 0;

    /**
     * @brief Accessor for the action name
     */
    const std::string& getName() const { return _name; }

    /**
     * @brief Connect the action to its container for posting status updates
     * @see getContainer
     */
    void connect(ActionContainer<Feedback>* container){
      _container = container;
    }

    ActionContainer<Feedback>& getContainer() {
      return *_container;
    }

  protected:

    /**
     * @brief Constructor
     * @param container The callback interface handle to notify changes in activation status
     */
  Action( const std::string& name)
    : _name(name), _container(NULL) {}

    virtual ~Action(){}

  private:

    const std::string _name; /*!< Name for the action */

    ActionContainer<Feedback>* _container; /*!< Holds the callback interface to notify changes in activation status */
  };

}
#endif
