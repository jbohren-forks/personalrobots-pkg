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

namespace executive_trex_pr2 {

  class StopAction: public robot_actions::Action<std_msgs::String, std_msgs::Empty> {
  public:

    StopAction(): robot_actions::Action<std_msgs::String, std_msgs::Empty>("stop_action") {
      ROS_DEBUG("Initializing %s\n", getName().c_str());
    }

  protected:

    virtual robot_actions::ResultStatus execute(const std_msgs::String& goal, std_msgs::Empty& feedback){

      ROS_DEBUG("Executing %s\n", getName().c_str());
      std::string preempt_topic(goal.data + "/preempt");
      ros::Node::instance()->advertise<std_msgs::Empty>(preempt_topic, 1);

      const ros::Time start_time = ros::Time::now();
      ros::Duration duration_bound(1.0);

      // Wait for activation. If we do not activate in time, we will preempt. Will not block for preemption
      // callback since the action may be bogus.
      ros::Duration SLEEP_DURATION(0.010);
      while(true){
	// Check if we need to preempt. If we do, we publish the preemption but will not terminate until
	// inactive
	ros::Duration elapsed_time = ros::Time::now() - start_time;

	if(elapsed_time > duration_bound){
	  break;
	}

	ROS_DEBUG("Dispatching premption command on topic %s\n", preempt_topic.c_str());
	ros::Node::instance()->publish(preempt_topic, std_msgs::Empty());
	SLEEP_DURATION.sleep();
      }
    

      ROS_DEBUG("Unadvertizing %s\n", preempt_topic.c_str());
      ros::Node::instance()->unadvertise(preempt_topic);

      return robot_actions::SUCCESS;
    }
  };
}

int main(int argc, char** argv){ 
  ros::init(argc, argv);
  ros::Node node("executive_trex_pr2/robot_actions");

  // Allocate an action runner with an update rate of 10 Hz
  executive_trex_pr2::StopAction stop_action;
  robot_actions::ActionRunner runner(10.0);
  runner.connect<std_msgs::String, pr2_robot_actions::StopActionState, std_msgs::Empty>(stop_action);

  // Miscellaneous
  runner.run();
  node.spin();
  return 0;
}

