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

/* \author: Ioan Sucan */

#include <ros/ros.h>
#include <robot_actions/action_client.h>

#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <motion_planning_msgs/KinematicPath.h>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <fstream>

void spinThread(void)
{
    ros::spin();
}

void printJoints(const planning_environment::KinematicModelStateMonitor &km, const std::vector<std::string> &names)
{
    const planning_models::KinematicModel::ModelInfo &mi = km.getKinematicModel()->getModelInfo();
    
    for (unsigned int i = 0 ; i < names.size(); ++i)
    {
	int idx = km.getKinematicModel()->getJointIndex(names[i]);
	std::cout << "  " << i << " = " << names[i] << "  [" << mi.stateBounds[idx * 2] << ", " << mi.stateBounds[idx * 2 + 1] << "]" << std::endl;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_state", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    
    std::string arm = "r";
    if (argc >= 2)
	if (argv[1][0] == 'l')
	    arm = "l";
    
    ros::NodeHandle nh;
    
    std::vector<std::string> names(7);
    names[0] = arm + "_shoulder_pan_joint";
    names[1] = arm + "_shoulder_lift_joint";
    names[2] = arm + "_upper_arm_roll_joint";
    names[3] = arm + "_elbow_flex_joint";
    names[4] = arm + "_forearm_roll_joint";
    names[5] = arm + "_wrist_flex_joint";
    names[6] = arm + "_wrist_roll_joint";
    
    std::string group = (arm == "r") ? "right_arm" : "left_arm";
    
    planning_environment::RobotModels rm("robot_description");
    if (!rm.loadedModels())
	return 0;
    
    boost::thread th(&spinThread);    
    tf::TransformListener tf;
    planning_environment::KinematicModelStateMonitor km(&rm, &tf);
    km.waitForState();
    
    std::cout << std::endl << std::endl << "State recorder using joints:" << std::endl;
    printJoints(km, names);
    std::cout << std::endl;

    while (nh.ok() && std::cin.good() && !std::cin.eof())
    {
	std::cout << "Record state as: ";
	std::string cmd;
	std::cin >> cmd;
	
	if (cmd == "quit")
	    break;
	
	std::string fnm = getenv("HOME") + ("/states/" + cmd);
	std::vector<double> params;
	km.getRobotState()->copyParamsGroup(params, group);
	std::ofstream out(fnm.c_str());
	for (unsigned int i = 0 ; i < params.size() ; ++i)
	{
	    std::cout << params[i] << " ";
	    out << params[i] << " ";
	}
	std::cout << std::endl;
	out << std::endl;
	out.close();
	std::cout << "Saved in " << fnm << std::endl;	
	std::cout << std::endl;	
    }
    std::cout << "Press Ctrl + C" << std::endl;
    th.join();
    
    return 0;
}
