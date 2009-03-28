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
 * @file This file defines a set of actions implemented as stubs so that we can easily
 * test the ros adapters.
 *
 * @author Conor McGann
 */

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <robot_msgs/Pose.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/BatteryState.h>
#include <robot_msgs/PlugStow.h>
#include <robot_actions/action_runner.h>
#include <robot_actions/action.h>
#include <robot_actions/DoorActionState.h>
#include <robot_actions/ShellCommandState.h>
#include <robot_actions/MoveBaseState.h>
#include <robot_actions/Pose2D.h>
#include <robot_actions/RechargeState.h>
#include <robot_actions/DetectPlugOnBaseActionState.h>
#include <boost/thread.hpp>
#include <cstdlib>

/**
 * @note This file is temporary. Normally we would not declare and define all these actions together. Just
 * working with it until we converge on action design model.
 */
namespace executive_trex_pr2 {

  template <class T>
  class StubAction: public robot_actions::Action<T, T> {
  public:

    StubAction(const std::string& name): robot_actions::Action<T, T>(name) {}

  private:

    // Activation does all the real work
    virtual void handleActivate(const T& msg){
      // Immediate reply
      robot_actions::Action<T, T>::notifyActivated();
      _state = msg;
      notifySucceeded(msg);
    }

    // Activation does all the real work
    virtual void handlePreempt(){
      // Immediate reply
      notifyPreempted(_state);
    }

    T _state;
  };

  /**
   * @brief This stub handles publishing state messages at a given rate
   */
  template<class State> class StatePublisher{
  public:
    StatePublisher(const State& state, const std::string& update_topic, double update_rate)
      : _terminated(false), _state(state), _update_topic(update_topic), _update_rate(update_rate), _update_thread(NULL) {

      ROS_ASSERT(_update_rate > 0);

      // Register publisher
      ros::Node::instance()->advertise<State>(_update_topic, 1);

      // Start the update
      _update_thread = new boost::thread(boost::bind(&StatePublisher<State>::updateLoop, this));
    }

    ~StatePublisher(){
      _terminated = true;
      _update_thread->join();
      delete _update_thread;
    }

  private:

    void updateLoop(){
      ros::Duration sleep_time(1/_update_rate);
      while (!_terminated){
	ros::Node::instance()->publish(_update_topic, _state);
	sleep_time.sleep();
      }
    }

    bool _terminated;
    const State _state;
    const std::string _update_topic;
    const double _update_rate;
    boost::thread* _update_thread;
  };



  template <class Goal, class Feedback>
  class StubAction1: public robot_actions::Action<Goal, Feedback> {
  public:

    StubAction1(const std::string& name): robot_actions::Action<Goal, Feedback>(name) {}

  private:

    // Activation does all the real work
    virtual void handleActivate(const Goal& msg){
      // Immediate reply
      robot_actions::Action<Goal, Feedback>::notifyActivated();
      notifySucceeded(_feedback);
    }

    // Activation does all the real work
    virtual void handlePreempt(){
      // Immediate reply
      notifyPreempted(_feedback);
    }

    Feedback _feedback;
  };
}

int main(int argc, char** argv){ 
  ros::init(argc, argv);
  ros::Node node("executive_trex_pr2/action_container");

  // Create state publishers
  executive_trex_pr2::StatePublisher<robot_msgs::Pose> base_state_publisher(robot_msgs::Pose(), "base_state", 10.0);
  executive_trex_pr2::StatePublisher<robot_msgs::BatteryState> battery_state_publisher(robot_msgs::BatteryState(), "battery_state", 10.0);
  executive_trex_pr2::StatePublisher<robot_msgs::BatteryState> bogus_battery_state_publisher(robot_msgs::BatteryState(), "bogus_battery_state", 10.0);


  // Allocate an action runner with an update rate of 10 Hz
  robot_actions::ActionRunner runner(10.0);
  
  // Add action stubs for doors
  executive_trex_pr2::StubAction<robot_msgs::Door> detect_door("detect_door");
  executive_trex_pr2::StubAction<robot_msgs::Door> grasp_handle("grasp_handle");
  executive_trex_pr2::StubAction<robot_msgs::Door> open_door("open_door");
  runner.connect<robot_msgs::Door, robot_actions::DoorActionState, robot_msgs::Door>(detect_door);
  runner.connect<robot_msgs::Door, robot_actions::DoorActionState, robot_msgs::Door>(grasp_handle);
  runner.connect<robot_msgs::Door, robot_actions::DoorActionState, robot_msgs::Door>(open_door);

  // Action stubs for plugs
  executive_trex_pr2::StubAction1<std_msgs::Empty, robot_msgs::PlugStow> detect_plug_on_base("detect_plug_on_base");
  runner.connect<std_msgs::Empty, robot_actions::DetectPlugOnBaseActionState, robot_msgs::PlugStow>(detect_plug_on_base);

  // Allocate other action stubs
  executive_trex_pr2::StubAction<robot_actions::Pose2D> move_base("move_base");
  runner.connect<robot_actions::Pose2D, robot_actions::MoveBaseState, robot_actions::Pose2D>(move_base);
  executive_trex_pr2::StubAction<std_msgs::Float32> recharge("recharge_controller");
  runner.connect<std_msgs::Float32, robot_actions::RechargeState, std_msgs::Float32>(recharge);
  executive_trex_pr2::StubAction<std_msgs::String> shell_command("shell_command");
  runner.connect<std_msgs::String, robot_actions::ShellCommandState, std_msgs::String>(shell_command);

  // Miscellaneous
  runner.run();
  node.spin();

  return 0;
}

