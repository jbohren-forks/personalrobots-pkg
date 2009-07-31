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
#include <manipulation_srvs/IKService.h>
#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>
#include <pr2_robot_actions/ActuateGripperState.h>
#include <std_msgs/Float64.h>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

void printHelp(void)
{
    std::cout << "Basic commands:" << std::endl;
    std::cout << "   - help       : this screen" << std::endl;
    std::cout << "   - list       : prints the list of arm joints" << std::endl;
    std::cout << "   - time       : shows the allowed execution time for arm movement" << std::endl;
    std::cout << "   - time <val> : sets the allowed execution time for arm movement" << std::endl;
    std::cout << "   - quit       : quits this program" << std::endl;
    std::cout << "Arm configuration commands:" << std::endl;
    std::cout << "   - show                    : shows the available configs" << std::endl;
    std::cout << "   - show <config>           : shows the values in <config>" << std::endl;
    std::cout << "   - view                    : shows the current config in the visualizer" << std::endl;
    std::cout << "   - view <config>           : shows <config> in the visualizer" << std::endl;
    std::cout << "   - clear                   : clears all stored configs" << std::endl;
    std::cout << "   - clear <config>          : remove stored <config>" << std::endl;
    std::cout << "   - current                 : show the values of the current configuration" << std::endl;
    std::cout << "   - current <config>        : set <config> to the current position of the arm" << std::endl;
    std::cout << "   - rand <config>           : set <config> to a random position of the arm" << std::endl;
    std::cout << "   - diff <config>           : show the difference from current position of the arm to <config>" << std::endl;
    std::cout << "   - ik <config> <x> <y> <z> : perform IK for the pose (<x>, <y>, <z>, 0, 0, 0, 1) and store it in <config>" << std::endl;
    std::cout << "   - go <config>             : sends the command <config> to the arm" << std::endl;
    std::cout << "   - go <x> <y> <z>          : move the end effector to pose (<x>, <y>, <z>, 0, 0, 0, 1)" << std::endl;
    std::cout << "   - grip <value>            : sends a command to the gripper of the arm" << std::endl;
    std::cout << "   - <config>[<idx>] = <val> : sets the joint specified by <idx> to <val> in <config>" << std::endl;
    std::cout << "   - <config2> = <config1>   : copy <config1> to <config2>" << std::endl;
    std::cout << "   - <config>                : same as show(<config>)" << std::endl;
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

void printPose(const btTransform &p)
{
    std::cout << "  -position [x, y, z]    = [" << p.getOrigin().x() << ", " << p.getOrigin().y() << ", " << p.getOrigin().z() << "]" << std::endl;
    btQuaternion q = p.getRotation();
    std::cout << "  -rotation [x, y, z, w] = [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]" << std::endl;
}

void goalToState(const pr2_robot_actions::MoveArmGoal &goal, planning_models::StateParams &sp)
{  
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size() ; ++i)
    {
	sp.setParamsJoint(&goal.goal_constraints.joint_constraint[i].value[0],
			  goal.goal_constraints.joint_constraint[i].joint_name);
    }
}

btTransform effPosition(const planning_environment::KinematicModelStateMonitor &km, const pr2_robot_actions::MoveArmGoal &goal)
{
    planning_models::StateParams sp(*km.getRobotState());
    goalToState(goal, sp);
    km.getKinematicModel()->computeTransforms(sp.getParams());
    return km.getKinematicModel()->getJoint(goal.goal_constraints.joint_constraint.back().joint_name)->after->globalTrans;
}

void printConfig(const pr2_robot_actions::MoveArmGoal &goal)
{
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size(); ++i)
	std::cout << "  " << goal.goal_constraints.joint_constraint[i].joint_name << " = " << goal.goal_constraints.joint_constraint[i].value[0] << std::endl;
}

void printConfigs(const std::map<std::string, pr2_robot_actions::MoveArmGoal> &goals)
{
    for (std::map<std::string, pr2_robot_actions::MoveArmGoal>::const_iterator it = goals.begin() ; it != goals.end() ; ++it)
	std::cout << "  " << it->first << std::endl;
}

void setupGoal(const std::vector<std::string> &names, pr2_robot_actions::MoveArmGoal &goal)
{
    goal.goal_constraints.joint_constraint.resize(names.size());
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size(); ++i)
    {
	goal.goal_constraints.joint_constraint[i].header.stamp = ros::Time::now();
	goal.goal_constraints.joint_constraint[i].header.frame_id = "/base_link";
	goal.goal_constraints.joint_constraint[i].joint_name = names[i];
	goal.goal_constraints.joint_constraint[i].value.resize(1);
	goal.goal_constraints.joint_constraint[i].tolerance_above.resize(1);
	goal.goal_constraints.joint_constraint[i].tolerance_below.resize(1);
	goal.goal_constraints.joint_constraint[i].value[0] = 0.0;
	goal.goal_constraints.joint_constraint[i].tolerance_below[0] = 0.0;
	goal.goal_constraints.joint_constraint[i].tolerance_above[0] = 0.0;
    }
}

void setupGoalEEf(const std::string &link, const std::vector<double> &pz, pr2_robot_actions::MoveArmGoal &goal)
{
    goal.goal_constraints.pose_constraint.resize(1);
    goal.goal_constraints.pose_constraint[0].type = motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z + 
	+ motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y;
    goal.goal_constraints.pose_constraint[0].link_name = link;
    goal.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
    goal.goal_constraints.pose_constraint[0].pose.header.frame_id = "/base_link";
    goal.goal_constraints.pose_constraint[0].pose.pose.position.x = pz[0];
    goal.goal_constraints.pose_constraint[0].pose.pose.position.y = pz[1];	
    goal.goal_constraints.pose_constraint[0].pose.pose.position.z = pz[2];	
    
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.x = pz[3];
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.y = pz[4];
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.z = pz[5];
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.w = pz[6];
    
    goal.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.01;
    goal.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.01;
    goal.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.015;
    goal.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.01;
    goal.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.01;
    goal.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.015;
    
    goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.15;
    goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.15;
    goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.15;
    goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.15;
    goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.15;
    goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.15;    

    goal.goal_constraints.pose_constraint[0].orientation_importance = 0.01;
}

void setConfig(const planning_models::StateParams *_sp, const std::vector<std::string> &names, pr2_robot_actions::MoveArmGoal &goal)
{
    setupGoal(names, goal);
    planning_models::StateParams sp(*_sp);
    sp.enforceBounds();
    for (unsigned int i = 0 ; i < names.size() ; ++i)
    {
	goal.goal_constraints.joint_constraint[i].value[0] = 
	    sp.getParamsJoint(goal.goal_constraints.joint_constraint[i].joint_name)[0];
    }
}

void diffConfig(const planning_environment::KinematicModelStateMonitor &km, pr2_robot_actions::MoveArmGoal &goal)
{
    std::vector<std::string> names;
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size(); ++i)
    {
	std::cout << "  " << goal.goal_constraints.joint_constraint[i].joint_name << " = " 
		  << goal.goal_constraints.joint_constraint[i].value[0] - km.getRobotState()->getParamsJoint(goal.goal_constraints.joint_constraint[i].joint_name)[0]
		  << std::endl;
	names.push_back(goal.goal_constraints.joint_constraint[i].joint_name);	
    }

    btTransform pose1 = effPosition(km, goal);
    pr2_robot_actions::MoveArmGoal temp;
    setConfig(km.getRobotState(), names, temp);
    btTransform pose2 = effPosition(km, temp);
    std::cout << std::endl;    
    double dist = pose1.getOrigin().distance(pose2.getOrigin());
    std::cout << "  -position distance: " << dist << std::endl;
    double angle = pose1.getRotation().angle(pose2.getRotation());
    std::cout << "  -rotation distance: " << angle << std::endl;
}
	
void viewState(ros::Publisher &view, const planning_environment::KinematicModelStateMonitor &km, const planning_models::StateParams &st)
{
    motion_planning_msgs::KinematicPath kp;	
    
    kp.header.frame_id = km.getFrameId();
    kp.header.stamp = km.lastMechanismStateUpdate();
    
    // fill in start state with current one
    std::vector<planning_models::KinematicModel::Joint*> joints;
    km.getKinematicModel()->getJoints(joints);
    
    kp.start_state.resize(joints.size());
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
    {
	kp.start_state[i].header.frame_id = km.getFrameId();
	kp.start_state[i].header.stamp = km.lastMechanismStateUpdate();
	kp.start_state[i].joint_name = joints[i]->name;
	st.copyParamsJoint(kp.start_state[i].value, joints[i]->name);
    }
    view.publish(kp);
}

void setConfigJoint(const unsigned int pos, const double value, pr2_robot_actions::MoveArmGoal &goal)
{
    goal.goal_constraints.joint_constraint[pos].value[0] = value;
}

void spinThread(void)
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_line_move_arm", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    
    std::string arm = "r";
    if (argc >= 2)
	if (argv[1][0] == 'l')
	    arm = "l";
    
    ros::NodeHandle nh;
    robot_actions::ActionClient<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t> move_arm(arm == "r" ? "move_right_arm" : "move_left_arm");
    robot_actions::ActionClient<std_msgs::Float64, pr2_robot_actions::ActuateGripperState, std_msgs::Float64> gripper(arm == "r" ? "actuate_gripper_right_arm" : "actuate_gripper_left_arm");
    ros::Publisher view = nh.advertise<motion_planning_msgs::KinematicPath>("display_kinematic_path", 1);
    
    int32_t                                               feedback;
    std::map<std::string, pr2_robot_actions::MoveArmGoal> goals;
    
    std::vector<std::string> names(7);
    names[0] = arm + "_shoulder_pan_joint";
    names[1] = arm + "_shoulder_lift_joint";
    names[2] = arm + "_upper_arm_roll_joint";
    names[3] = arm + "_elbow_flex_joint";
    names[4] = arm + "_forearm_roll_joint";
    names[5] = arm + "_wrist_flex_joint";
    names[6] = arm + "_wrist_roll_joint";
    

    planning_environment::RobotModels rm("robot_description");
    if (!rm.loadedModels())
	return 0;

    boost::thread th(&spinThread);    
    tf::TransformListener tf;
    planning_environment::KinematicModelStateMonitor km(&rm, &tf);
    km.waitForState();
    
    std::cout << std::endl << std::endl << "Using joints:" << std::endl;
    printJoints(km, names);
    std::cout << std::endl;
    double allowed_time = 10.0;

    while (nh.ok() && std::cin.good() && !std::cin.eof())
    {
	std::cout << "command> ";
	std::string cmd;
	std::getline(std::cin, cmd);
	boost::trim(cmd);
	ros::spinOnce();
	
	if (!nh.ok())
	    break;

	if (cmd == "")
	    continue;

	if (cmd == "help")
	    printHelp();
	else
	if (cmd == "list")
	    printJoints(km, names);
	else
	if (cmd == "quit")
	    break;
	else
	if (cmd == "show")
	    printConfigs(goals);
	else
	if (cmd == "current")
	{
	    pr2_robot_actions::MoveArmGoal temp;
	    setConfig(km.getRobotState(), names, temp);
	    printConfig(temp);
	    std::cout << std::endl;
	    btTransform p = effPosition(km, temp);
	    printPose(p);
	}
	else
	if (cmd == "time")
	{
	    std::cout << "  Allowed execution time is " << allowed_time << " seconds" << std::endl;
	}
	else
	if (cmd.substr(0, 5) == "time " && cmd.length() > 5)
	{
	    std::stringstream ss(cmd.substr(5));
	    if (ss.good() && !ss.eof())
	        ss >> allowed_time;
	    else
	        std::cerr << "Unable to parse time value " << ss.str() << std::endl;
	    if (allowed_time <= 0.0)
	        allowed_time = 10.0;
	    std::cout << "  Allowed execution time is " << allowed_time << " seconds" << std::endl;
	}
	else
	if (cmd == "clear")
	    goals.clear();
	else
	if (cmd.length() > 6 && cmd.substr(0, 6) == "clear ")
	{
	    std::string config = cmd.substr(6);
	    boost::trim(config);
	    if (goals.find(config) == goals.end())
		std::cout << "Configuration '" << config << "' not found" << std::endl;
	    else
		goals.erase(config);  
	}
	else
	if (cmd == "view")
	{
	    planning_models::StateParams st(*km.getRobotState());		
	    viewState(view, km, st);
	}
	else
	if (cmd.length() > 5 && cmd.substr(0, 5) == "view ")
	{
	    std::string config = cmd.substr(5);
	    boost::trim(config);
	    if (goals.find(config) == goals.end())
		std::cout << "Configuration '" << config << "' not found" << std::endl;
	    else
	    {
		planning_models::StateParams st(*km.getRobotState());		
		goalToState(goals[config], st);
		viewState(view, km, st);
	    }
	}
	else
	if (cmd.length() > 5 && cmd.substr(0, 5) == "diff ")
	{
	    std::string config = cmd.substr(5);
	    boost::trim(config);
	    if (goals.find(config) == goals.end())
		std::cout << "Configuration '" << config << "' not found" << std::endl;
	    else
		diffConfig(km, goals[config]);
	}
	else
	if (cmd.length() > 5 && cmd.substr(0, 5) == "show ")
	{
	    std::string config = cmd.substr(5);
	    boost::trim(config);
	    if (goals.find(config) == goals.end())
		std::cout << "Configuration '" << config << "' not found" << std::endl;
	    else
	    {
		printConfig(goals[config]);
		std::cout << std::endl;
		btTransform p = effPosition(km, goals[config]);
		printPose(p);
	    }
	}
	else
	if (cmd.length() > 8 && cmd.substr(0, 8) == "current ")
	{
	    std::string config = cmd.substr(8);
	    boost::trim(config);
	    setConfig(km.getRobotState(), names, goals[config]);
	}
	else
	if (cmd.length() > 5 && cmd.substr(0, 5) == "rand ")
	{
	    std::string config = cmd.substr(5);
	    boost::trim(config);
	    planning_models::StateParams *sp = km.getKinematicModel()->newStateParams();
	    sp->randomState();
	    setConfig(sp, names, goals[config]);
	    delete sp;
	}
	else
	if (cmd.length() > 3 && cmd.substr(0, 3) == "ik ")
	{
	    std::stringstream ss(cmd.substr(3));
	    std::string config;
	    
	    if (ss.good() && !ss.eof())
		ss >> config;
	    
	    if (config.empty())
		std::cerr << "Configuration name required. See 'help'" << std::endl;
	    else
	    {
		double x, y, z;
		bool err = true;
		
		if (ss.good() && !ss.eof())
		{
		    ss >> x;
		    if (ss.good() && !ss.eof())
		    {
			ss >> y;
			if (ss.good() && !ss.eof())
			{
			    ss >> z;
			    err = false;
			    std::cout << "Performing IK to " << x << ", " << y << ", " << z << ", 0, 0, 0, 1..." << std::endl;
			    
			    ros::ServiceClient client = nh.serviceClient<manipulation_srvs::IKService>("arm_ik");
			    manipulation_srvs::IKService::Request request;
			    manipulation_srvs::IKService::Response response;
			    request.data.pose_stamped.header.stamp = ros::Time::now();
			    request.data.pose_stamped.header.frame_id = km.getFrameId();
			    request.data.pose_stamped.pose.position.x = x;
			    request.data.pose_stamped.pose.position.y = y;
			    request.data.pose_stamped.pose.position.z = z;
			    request.data.pose_stamped.pose.orientation.x = 0;
			    request.data.pose_stamped.pose.orientation.y = 0;
			    request.data.pose_stamped.pose.orientation.z = 0;
			    request.data.pose_stamped.pose.orientation.w = 1;
			    request.data.joint_names = names;
			    
			    planning_models::StateParams rs(*km.getRobotState());
			    rs.randomState();
			    
			    for(unsigned int i = 0; i < names.size() ; ++i)
			    {
				const double *params = rs.getParamsJoint(names[i]);
				const unsigned int u = km.getKinematicModel()->getJoint(names[i])->usedParams;
				for (unsigned int j = 0 ; j < u ; ++j)
				    request.data.positions.push_back(params[j]);
			    }
			    if (client.call(request, response))
			    { 
				planning_models::StateParams sp(*km.getRobotState());
				unsigned int n = 0;		    
				for (unsigned int i = 0 ; i < names.size() ; ++i)
				{
				    unsigned int u = km.getKinematicModel()->getJoint(names[i])->usedParams;
				    for (unsigned int j = 0 ; j < u ; ++j)
				    {
					std::vector<double> params(response.solution.begin() + n, response.solution.begin() + n + u);
					sp.setParamsJoint(params, names[i]);
				    }
				    n += u;			
				}
				setConfig(&sp, names, goals[config]);
				std::cout << "Success!" << std::endl;
			    }
			    else
				std::cerr << "IK Failed" << std::endl;
			}
		    }
		}
		if (err)
		    std::cerr << "Unable to parse IK position" << std::endl;
	    }
	}
	else
	if (cmd.length() > 3 && cmd.substr(0, 3) == "go ")
	{
	    std::string config = cmd.substr(3);
	    boost::trim(config);
	    if (goals.find(config) == goals.end())
	      {
		std::stringstream ss(config);
		std::vector<double> nrs;
		while (ss.good() && !ss.eof())
		  {
		    double value;
		    ss >> value;
		    nrs.push_back(value);
		  }

		bool err = true;
		if (nrs.size() == 3)
		{
		  nrs.push_back(0);
		  nrs.push_back(0);
		  nrs.push_back(0);
		  nrs.push_back(1);
		}

		if (nrs.size() == 7)
		  {
		    err = false;

		std::string link = km.getKinematicModel()->getJoint(names.back())->after->name;
		std::cout << "Moving " << link << " to " << nrs[0] << ", " << nrs[1] << ", " << nrs[2] << ", " <<
		  nrs[3] << ", " << nrs[4] << ", " << nrs[5] << ", " << nrs[6] << "..." << std::endl;
		pr2_robot_actions::MoveArmGoal g;
		setupGoalEEf(link, nrs, g);
		if (move_arm.execute(g, feedback, ros::Duration(allowed_time)) != robot_actions::SUCCESS)
		  std::cerr << "Failed achieving goal" << std::endl;
		else
		  std::cout << "Success!" << std::endl;
		

		    
		}
		
		
		if (err)
		    std::cout << "Configuration '" << config << "' not found" << std::endl;
	      }
	
	    else
	    {
		std::cout << "Moving to " << config << "..." << std::endl;
		if (move_arm.execute(goals[config], feedback, ros::Duration(allowed_time)) != robot_actions::SUCCESS)
		    std::cerr << "Failed achieving goal" << std::endl;
		else
		    std::cout << "Success!" << std::endl;
	    }
	}
	else
	if (cmd.length() > 5 && cmd.find_first_of("[") != std::string::npos)
	{
	    std::size_t p1 = cmd.find_first_of("[");
	    std::size_t p2 = cmd.find_last_of("]");
	    std::size_t p3 = cmd.find_first_of("=");
	    if (p1 != std::string::npos && p2 != std::string::npos && p3 != std::string::npos && p2 > p1 + 1 && p3 > p2)
	    {
		std::string config = cmd.substr(0, p1);
		boost::trim(config);
		if (goals.find(config) == goals.end())
		    std::cout << "Configuration '" << config << "' not found" << std::endl;
		else
		{
		    std::stringstream ss(cmd.substr(p1 + 1, p2 - p1 - 1));
		    unsigned int joint = names.size();
		    if (ss.good() && !ss.eof())
			ss >> joint;
		    if (joint >= names.size())
			std::cerr << "Joint index out of bounds" << std::endl;
		    else
		    {
			std::stringstream ss(cmd.substr(p3 + 1));
			if (ss.good() && !ss.eof())
			{
			    double value;
			    ss >> value;
			    setConfigJoint(joint, value, goals[config]);
			}
			else
			    std::cerr << "Cannot parse value: '" << ss.str() << "'" << std::endl;
		    }
		}
	    }
	    else
		std::cerr << "Incorrect command format" << std::endl;
	}
	else
	if (cmd.length() > 2 && cmd.find_first_of("=") != std::string::npos)
	{
	    std::size_t p = cmd.find_first_of("=");
	    std::string c1 = cmd.substr(0, p);
	    std::string c2 = cmd.substr(p + 1);
	    boost::trim(c1);
	    boost::trim(c2);
	    if (goals.find(c2) != goals.end())
		goals[c1] = goals[c2];
	    else
		std::cout << "Configuration '" << c2 << "' not found" << std::endl;
	}
	else
	if (cmd.length() > 5 && cmd.substr(0, 5) == "grip ")
	{
	    std::stringstream ss(cmd.substr(5));
	    if (ss.good() && !ss.eof())
	    {
		std_msgs::Float64 g, fb;
		ss >> g.data;
		if (gripper.execute(g, fb, ros::Duration(allowed_time)) != robot_actions::SUCCESS)
		    std::cerr << "Failed achieving goal" << std::endl;
		else
		    std::cout << "Success!" << std::endl;
	    }
	    else
		std::cerr << "A floating point value expected but '" << cmd.substr(5) << "' was given" << std::endl;
	}
	else
	if (goals.find(cmd) != goals.end())
	{
	    printConfig(goals[cmd]);
	    std::cout << std::endl;
	    btTransform p = effPosition(km, goals[cmd]);
	    printPose(p);
	}	
	else
	    std::cerr << "Unknown command. Try 'help'" << std::endl;
    }
    
    return 0;
}
