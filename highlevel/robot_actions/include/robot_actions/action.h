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

namespace robot_actions {

  /**
   * @brief Abstract base class for a durative, preemptable action. Implementations may assume that instances
   * will be created witin a node
   * @param Goal The custom goal type provided in a goal request, and for feedback during execution
   * @param Feedback The custom message type used to provide
   */
  template <class Goal, class Feedback> class Action {
  public:

    /**
     * @brief This method is called on receipt of a new goal. The derived class will implement this method to 
     * pursue the goal.
     * @param Goal The goal to accomplish.
     */
    virtual void activate(const Goal& goal) = 0;

    /**
     * @brief This method is called to preempt execution of a goal. The derived class must terminate all activiity
     * pursuing an active goal. As soon as we return from this call, the actor will be deactivated.
     */
    virtual void preempt() = 0;

    /**
     * @brief A query to retrieve the feedback status on completion. The termination criteria is that this returns a value other than
     * ACTIVE.
     */
    virtual  ActionStatus checkStatus(Feedback& feedback) = 0;

    virtual ~Action(){}
  };

}
#endif
