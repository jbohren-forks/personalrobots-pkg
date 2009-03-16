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

#ifndef H_robot_actions_ActionContainer
#define H_robot_actions_ActionContainer

#include <robot_actions/ActionStatus.h>
#include <boost/thread.hpp>
#include <ros/node.h>
#include <ros/console.h>
#include <boost/thread.hpp>

namespace robot_actions {

  /**
   * @brief This class defines a call back interface for actions executing in some container context.
   */
  template <class Feedback> class ActionContainer {

  public:

    /**
     * @brief A client will call this method when it has been activated successfully
     * @param Feedback to provide in the state update
     */
    virtual void notifyActivated(const Feedback& feedback_msg) = 0;

    /**
     * @brief A client will call this method when it has completed successfully.
     * @param Feedback to provide in the state update
     */
    virtual void notifyCompleted(const Feedback& feedback_msg) = 0;

    /**
     * @brief A client will call this method when it has aborted of its own volition
     * @param Feedback to provide in the state update
     */
    virtual void notifyAborted(const Feedback& feedback_msg) = 0;

    /**
     * @brief A client will call this method when it has successfully been preempted
     * @param Feedback to provide in the state update
     */
    virtual void notifyPreempted(const Feedback& feedback_msg) = 0;


  protected:

    virtual ~ActionContainer(){}
  };

}
#endif
