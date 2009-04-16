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

#ifndef H_robot_actions_MessageAdapter
#define H_robot_actions_MessageAdapter

#include <robot_actions/action.h>
#include <robot_actions/ActionStatus.h>
#include <std_msgs/Empty.h>
#include <boost/thread.hpp>
#include <ros/node.h>
#include <ros/console.h>
#include <boost/thread.hpp>

namespace robot_actions {

  /**
   * @brief An abstract base class that exposes key methods used for running actions all together
   */
  class AbstractAdapter {
  public:

    /**
     * @Destructor
     */
    virtual ~AbstractAdapter(){}

    /**
     * @brief Check if OK
     */
    bool isOk() const { return _initialized && !_terminated; }

    /**
     * @brief Call when ready to run the node
     */
    void initialize(){
      _initialized = true;
    }

    /**
     * @brief Implement clean up call on termination
     */
    virtual void terminate(){
      _terminated = true;
    }

    /**
     * @brief Called to push state update
     */
    virtual void publish() = 0;


  protected:

  AbstractAdapter(): _initialized(false), _terminated(false) {}

  private:
    bool _initialized; /*!< Marks if the node has been initialized, and is ready for use. */
    bool _terminated; /*!< Marks if the node has been terminated. */
  };

  /**
   * @brief A class that provides ROS message binding for an action
   * 
   * This class provides
   * - A call back for goal message handling - inbound to the action
   * - A call back for preemption message handling - inbound to the action
   * - A call back for state update notification - outbound
   *
   * The class is a template base class that assumes the state message published contains
   * - ActionStatus status
   * - Goal goal
   * - Feedback feedback
   *
   * An instance of this class will subscribe to a goal and preemption message, and will publish a state message.
   * @todo Check proper values for QUEUE_MAX
   * @note If we wanted to, we could couple this with te action. Separation means we can alter action bindings so they are not run over ROS messages
   */
  template <class Goal, class State, class Feedback> class MessageAdapter: public AbstractAdapter {

  public:

    /**
     * @brief Constructor
     * @param actionName The name for the action. This is used to scope the request, preemption and notify topics. All other parameters are obtained
     * via ROS paramaters
     */
  MessageAdapter(Action<Goal, Feedback>& action)
    : _action(action), _update_topic(_action.getName() + "/feedback") {

      // Connect the action to this container
      _action.connect(boost::bind(&MessageAdapter<Goal, State, Feedback>::notify, this, _1, _2, _3));

      // Advertize state updates
      ros::Node::instance()->advertise<State>(_update_topic, QUEUE_MAX());

      // Subscribe to goal requests. 
      ros::Node::instance()->subscribe(_action.getName() + "/activate", _request_msg, &MessageAdapter<Goal, State, Feedback>::requestHandler,  this, 1);

      // Subscribe to goal preemptions.
      ros::Node::instance()->subscribe(_action.getName() + "/preempt", _preemption_msg, &MessageAdapter<Goal, State, Feedback>::preemptionHandler,  this, 1);
    }

    // Call back invoked from the action. Packages up as a state message and ships
    void notify(const ActionStatus& status, const Goal& goal, const Feedback& feedback){
      State state_msg;
      state_msg.status = status;
      state_msg.goal = goal;
      state_msg.feedback = feedback;
      ros::Node::instance()->publish(_update_topic, state_msg);
    }

    virtual void publish(){
      if(isOk())
	_action.publish();
    }

  protected:

    /**
     * @brief Used to set queue max parameter for subscribe and advertise of ROS topics
     */
    static unsigned int QUEUE_MAX(){return 10;}

  private:

    /**
     * @brief Handle a new request.
     */
    void requestHandler(){
      if(isOk()){
	_action.activate(_request_msg);
      }
    }

    /**
     * @brief Handle a preemption. Will ignore the request unless the action is currently active. Delegates to subclass for details.
     */
    void preemptionHandler(){
      if(isOk())
	_action.preempt();
    }

    void terminate(){
      AbstractAdapter::terminate();
      _action.terminate();
    }
    
    /** DATA MEMBERS **/
    Action<Goal, Feedback>& _action; /*! The action to do the real work */
    const std::string _update_topic;
    Goal _request_msg; /*!< Message populated by handler for a request */
    std_msgs::Empty _preemption_msg; /*!< Message populated by handler for a preemption. */
    State _state_msg; /*!< Message published. */
  };

}
#endif
