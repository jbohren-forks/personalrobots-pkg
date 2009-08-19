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
 **********************************************************************/

/* Author: Melonee Wise */

#include <ros/ros.h>

#include <actionlib/server/single_goal_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
public:
    
    FibonacciAction(std::string name) : 
      as_(nh_, name, boost::bind(&FibonacciAction::execute, this, _1)),
      action_name_(name)
    {
    }

    ~FibonacciAction(void)
    {
    }

    void execute(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
    {
      ros::Rate r(100);
      std::vector<int> sequence;
      int temp;  
      actionlib_tutorials::FibonacciFeedback feedback;
      actionlib_tutorials::FibonacciResult result;
      bool success = true;
      sequence.push_back(0);
      sequence.push_back(1);

	    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, sequence[0], sequence[1]);
          
    
      for(int i=1; i<=goal->order; i++)
      {        
        if (as_.isPreemptRequested())
	      {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          as_.setPreempted();
          success = false;
          break;
        }
        temp = sequence[i] + sequence[i-1];
        sequence.push_back(temp);
        feedback.set_sequence_vec(sequence);
        as_.publishFeedback(feedback);
        r.sleep(); 
      }

      if(success)
      {
        result.set_sequence_vec(sequence);
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result);
      }
    }

protected:
    
    ros::NodeHandle nh_;
    actionlib::SingleGoalActionServer<actionlib_tutorials::FibonacciAction> as_;
    std::string action_name_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fibonacci");

    FibonacciAction fibonacci(ros::this_node::getName());
	  ros::spin();

    return 0;
}
