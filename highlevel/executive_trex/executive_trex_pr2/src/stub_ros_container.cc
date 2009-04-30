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
#include <executive_trex_pr2/adapters.h>
#include <robot_actions/action.h>
#include <boost/thread.hpp>
#include <cstdlib>

/**
 * @note This file is temporary. Normally we would not declare and define all these actions together. Just
 * working with it until we converge on action design model.
 */
namespace executive_trex_pr2 {



  template <class Goal, class Feedback>
  class StubAction: public robot_actions::Action<Goal, Feedback> {
  public:

    StubAction(const std::string& name): robot_actions::Action<Goal, Feedback>(name) {
    }
    StubAction(const std::string& name, const Feedback& default_feedback) : 
      robot_actions::Action<Goal, Feedback>(name), default_feedback_(default_feedback) {
    }

  protected:

    virtual robot_actions::ResultStatus execute(const Goal& goal, Feedback& feedback){
      ROS_DEBUG("Executing %s\n", robot_actions::Action<Goal, Feedback>::getName().c_str());
      feedback = default_feedback_;
      return robot_actions::SUCCESS;
    }
  private:
    Feedback default_feedback_;
  };


  template <class T> class SimpleStubAction: public robot_actions::Action<T,T> {
  public:
    SimpleStubAction(const std::string& name): robot_actions::Action<T, T>(name) {
      _duration.fromSec(1.0);
    }
    SimpleStubAction(const std::string& name, double secs): robot_actions::Action<T, T>(name) {
      _duration.fromSec(secs);
    }

    virtual robot_actions::ResultStatus execute(const T& goal, T& feedback){
      feedback = goal;
      ROS_DEBUG("Executing %s\n", robot_actions::Action<T, T>::getName().c_str());
      _duration.sleep();

      if(robot_actions::Action<T, T>::isPreemptRequested())
	return robot_actions::PREEMPTED;

      return robot_actions::SUCCESS;
    }

  private:

    ros::Duration _duration;
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

}


/**
 * Test if a component should be created, according to ros params.
 */
bool getComponentParam(std::string name) {
  bool value = false;
  ros::Node::instance()->param(name, value, value);
  ROS_INFO("Parameter for %s is %d", name.c_str(), value);
  return value;
}



int main(int argc, char** argv){ 
  ros::init(argc, argv);
  ros::Node node("executive_trex_pr2/action_container");

  // Create state publishers, if parameters are set
  executive_trex_pr2::StatePublisher<deprecated_msgs::RobotBase2DOdom>* base_state_publisher = NULL;
  if (getComponentParam("/trex/enable_base_state_publisher"))
    base_state_publisher = new executive_trex_pr2::StatePublisher<deprecated_msgs::RobotBase2DOdom>(deprecated_msgs::RobotBase2DOdom(), "localizedpose", 10.0);

  executive_trex_pr2::StatePublisher<robot_msgs::BatteryState>* battery_state_publisher = NULL; 
  if (getComponentParam("/trex/enable_battery_state_publisher"))
    battery_state_publisher = new executive_trex_pr2::StatePublisher<robot_msgs::BatteryState>(robot_msgs::BatteryState(), "battery_state", 10.0);

  executive_trex_pr2::StatePublisher<robot_msgs::BatteryState>* bogus_battery_state_publisher = NULL; 
  if (getComponentParam("/trex/enable_bogus_battery_state_publisher"))
    bogus_battery_state_publisher = new executive_trex_pr2::StatePublisher<robot_msgs::BatteryState>(robot_msgs::BatteryState(), "bogus_battery_state", 10.0);


  // Allocate an action runner with an update rate of 10 Hz
  robot_actions::ActionRunner runner(10.0);
  
  /* Add action stubs for doors */

  executive_trex_pr2::StubAction<robot_msgs::PoseStamped, int8_t> check_path("check_path");
  if (getComponentParam("/trex/enable_check_path"))
    runner.connect<robot_msgs::PoseStamped, pr2_robot_actions::CheckPathState, int8_t>(check_path);

  // Detect Door
  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> detect_door("detect_door");
  if (getComponentParam("/trex/enable_detect_door"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(detect_door);

  // Detect Handle
  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> detect_handle("detect_handle");
  if (getComponentParam("/trex/enable_detect_handle"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(detect_handle);

  // Grasp Handle
  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> grasp_handle("grasp_handle");
  if (getComponentParam("/trex/enable_grasp_handle"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(grasp_handle);

  // Unlatch Handle
  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> unlatch_handle("unlatch_handle");
  if (getComponentParam("/trex/enable_unlatch_handle"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(unlatch_handle);

  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> open_door("open_door", 5.0);
  if (getComponentParam("/trex/enable_open_door"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(open_door);

  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> move_base_door("move_base_door");
  if (getComponentParam("/trex/enable_move_base_door"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(move_base_door);

  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> touch_door("touch_door");
  if (getComponentParam("/trex/enable_touch_door"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(touch_door);

  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> push_door("push_door");
  if (getComponentParam("/trex/enable_push_door"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(push_door);

  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> release_handle("release_handle");
  if (getComponentParam("/trex/enable_release_handle"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(release_handle);

  /* Action stubs for plugs */
  executive_trex_pr2::StubAction<std_msgs::Empty, robot_msgs::PlugStow> detect_plug_on_base("detect_plug_on_base");
  if (getComponentParam("/trex/enable_detect_plug_on_base"))
    runner.connect<std_msgs::Empty, pr2_robot_actions::DetectPlugOnBaseState, robot_msgs::PlugStow>(detect_plug_on_base);

  executive_trex_pr2::StubAction<robot_msgs::PlugStow, std_msgs::Empty> move_and_grasp_plug("move_and_grasp_plug");
  if (getComponentParam("/trex/enable_move_and_grasp_plug"))
    runner.connect<robot_msgs::PlugStow, pr2_robot_actions::MoveAndGraspPlugState, std_msgs::Empty>(move_and_grasp_plug);

  executive_trex_pr2::StubAction<robot_msgs::PlugStow, std_msgs::Empty> stow_plug("stow_plug");
  if (getComponentParam("/trex/enable_stow_plug"))
    runner.connect<robot_msgs::PlugStow, pr2_robot_actions::StowPlugState, std_msgs::Empty>(stow_plug);

  executive_trex_pr2::SimpleStubAction<std_msgs::Empty> plugs_untuck_arms("plugs_untuck_arms");
  if (getComponentParam("/trex/enable_plugs_untuck_arms"))
    runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(plugs_untuck_arms);

  executive_trex_pr2::SimpleStubAction<std_msgs::Empty> localize_plug_in_gripper("localize_plug_in_gripper");
  if (getComponentParam("/trex/enable_localize_plug_in_gripper"))
    runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(localize_plug_in_gripper);

  executive_trex_pr2::SimpleStubAction<std_msgs::Empty> unplug("unplug");
  if (getComponentParam("/trex/enable_unplug"))
    runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(unplug);
  
  executive_trex_pr2::SimpleStubAction<std_msgs::Empty> plug_in("plug_in");
  if (getComponentParam("/trex/enable_plug_in"))
    runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(plug_in);
  
  executive_trex_pr2::StubAction<robot_msgs::PointStamped, robot_msgs::PoseStamped> detect_outlet_fine("detect_outlet_fine");
  if (getComponentParam("/trex/enable_detect_outlet_fine"))
    runner.connect<robot_msgs::PointStamped, pr2_robot_actions::DetectOutletState, robot_msgs::PoseStamped>(detect_outlet_fine);

  robot_msgs::PoseStamped outlet_feedback;
  outlet_feedback.pose.orientation.x = 1.0;
  executive_trex_pr2::StubAction<robot_msgs::PointStamped, robot_msgs::PoseStamped> detect_outlet_coarse("detect_outlet_coarse", outlet_feedback);
  if (getComponentParam("/trex/enable_detect_outlet_coarse"))
    runner.connect<robot_msgs::PointStamped, pr2_robot_actions::DetectOutletState, robot_msgs::PoseStamped>(detect_outlet_coarse);

  /* Action stubs for resource management */
  executive_trex_pr2::StubAction<pr2_robot_actions::SwitchControllers, std_msgs::Empty> switch_controllers("switch_controllers");
  if (getComponentParam("/trex/enable_switch_controllers"))
    runner.connect<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState, std_msgs::Empty>(switch_controllers);

  // Navigation actions
  executive_trex_pr2::SimpleStubAction<robot_msgs::PoseStamped> move_base("move_base");
  if (getComponentParam("/trex/enable_move_base"))
    runner.connect<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped>(move_base);

  executive_trex_pr2::SimpleStubAction<robot_msgs::PoseStamped> move_base_local("move_base_local");
  if (getComponentParam("/trex/enable_move_base_local"))
    runner.connect<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped>(move_base_local);

  // Misc.
  executive_trex_pr2::SimpleStubAction<std_msgs::Float32> recharge("recharge_controller");
  if (getComponentParam("/trex/enable_recharge"))
    runner.connect<std_msgs::Float32, pr2_robot_actions::RechargeState, std_msgs::Float32>(recharge);

  executive_trex_pr2::SimpleStubAction<std_msgs::String> shell_command("shell_command");
  if (getComponentParam("/trex/enable_shell_command"))
    runner.connect<std_msgs::String, pr2_robot_actions::ShellCommandState, std_msgs::String>(shell_command);


  executive_trex_pr2::SimpleStubAction<robot_msgs::Door> notify_door_blocked("notify_door_blocked");
  if (getComponentParam("/trex/enable_notify_door_blocked"))
    runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(notify_door_blocked);

  executive_trex_pr2::SimpleStubAction<std_msgs::Empty> safety_tuck_arms("safety_tuck_arms");
  if (getComponentParam("/trex/enable_safety_tuck_arms"))
    runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(safety_tuck_arms);

  executive_trex_pr2::SimpleStubAction<std_msgs::Empty> doors_tuck_arms("doors_tuck_arms");
  if (getComponentParam("/trex/enable_doors_tuck_arms"))
    runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(doors_tuck_arms);

  // Miscellaneous
  runner.run();

  node.spin();

  if (base_state_publisher)
    delete base_state_publisher;
  if (battery_state_publisher)
    delete battery_state_publisher;
  if (bogus_battery_state_publisher)
    delete bogus_battery_state_publisher;

  return 0;
}

