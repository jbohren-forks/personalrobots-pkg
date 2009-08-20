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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 *
 *********************************************************************/

/* Author: Melonee Wise */
#include <ros/node.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  // helper variables
  ros::Duration timeout(30.0);

  ros::init(argc, argv, "test_fibonacci");  
  // start thread to spin node
  boost::thread spin_thread(&spinThread);

  // create the action client
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci");
  sleep(1);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForActionServerToStart(); //will wait for infinite time 
  ROS_INFO("Action server started, sending goal.");  

  // send a goal to the action 
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 10;
  ac.sendGoal(goal);

  // wait for the action to resturn 
  bool finished_before_timeout = ac.waitForGoalToFinish(timeout);

  if (finished_before_timeout)
    ROS_INFO("Finished");
  else  
    ROS_INFO("TimedOut");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  return 0;
}
