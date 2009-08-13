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
 * $Id: test_executive.cpp 18230 2009-07-02 21:27:56Z tfoote $
 *
 *********************************************************************/

/* Author: Sachin Chitta */


#include <ros/node.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm/ActuateGripperAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_actuate_gripper");
    
    ros::NodeHandle nh;    
    ros::Duration timeout(10.0);

    actionlib::SimpleActionClient<move_arm::ActuateGripperAction> gripper(nh, "actuate_gripper_right_arm");
    
    move_arm::ActuateGripperGoal cmd;
    cmd.data = -100;
    
    gripper.sendGoal(cmd);

    bool finished_before_timeout = gripper.waitForGoalToFinish(timeout);
    if (finished_before_timeout)
	std::cout << "Final state is " << gripper.getTerminalState().toString() << std::endl;
    else
    {
	gripper.cancelGoal();
	std::cerr << "Failed achieving goal" << std::endl;
	return -1;
    }
    
    cmd.data = 100;

    gripper.sendGoal(cmd);

    finished_before_timeout = gripper.waitForGoalToFinish(timeout);
    if (finished_before_timeout)
	std::cout << "Final state is " << gripper.getTerminalState().toString() << std::endl;
    else
    {
	gripper.cancelGoal();
	std::cerr << "Failed achieving goal" << std::endl;
	return -1;
    }
    
    return 0;
}
