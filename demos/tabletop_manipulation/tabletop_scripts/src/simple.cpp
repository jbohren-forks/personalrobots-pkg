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
#include <mapping_msgs/ObjectInMap.h>
#include <mapping_msgs/AttachedObject.h>

#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>
#include <pr2_robot_actions/ActuateGripperState.h>
#include <std_msgs/Float64.h>

#include <boost/thread/thread.hpp>
#include <recognition_lambertian/FindObjectPoses.h>
#include <visualization_msgs/Marker.h>

class CleanTable
{

public:
    
    CleanTable(void) :  move_arm("move_right_arm"), gripper("actuate_gripper_right_arm")
    {    
	pubObj = nh.advertise<mapping_msgs::ObjectInMap>("object_in_map", 1);
	pubAttach = nh.advertise<mapping_msgs::AttachedObject>("attach_map", 1);
	
	vmPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10240);    
    }
    
    void sendPoint(const roslib::Header &header, double x, double y, double z)
    {
	visualization_msgs::Marker mk;
	
	mk.header = header;
	mk.ns = "test_recognition";
	mk.id = 1;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;
	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;
	
	mk.scale.x = mk.scale.y = mk.scale.z = 0.03;
	
	mk.color.a = 1.0;
	mk.color.r = 0.5;
	mk.color.g = 0.4;
	mk.color.b = 0.04;
	
	vmPub.publish(mk);
    }
    
    bool findObject(recognition_lambertian::TableTopObject &obj)
    {
	recognition_lambertian::FindObjectPoses::Request  req;
	recognition_lambertian::FindObjectPoses::Response res;
	
	ros::ServiceClient client = nh.serviceClient<recognition_lambertian::FindObjectPoses>("table_top/find_object_poses");
	if (client.call(req, res))
	{
	    ROS_INFO("Found %d objects", (int)res.objects.size());
	    if (res.objects.empty())
		return false;
	    
	    obj = res.objects[0];
	    for (unsigned int i = 1 ; i < res.objects.size() ; ++i)
	    {
		if (res.objects[i].grasp_pose.pose.position.y < obj.grasp_pose.pose.position.y)
		    obj = res.objects[i];
	    }
	    
	    // hack for stereo error
	    double dx = 0.055;
	    double dy = 0.02;
	    double dz = -0.04;
	    
	    obj.object_pose.pose.position.x += dx;
	    obj.object_pose.pose.position.y += dy;
	    obj.object_pose.pose.position.z += dz;
	    obj.object_pose.pose.orientation.x = 0;
	    obj.object_pose.pose.orientation.y = 0;
	    obj.object_pose.pose.orientation.z = 0;
	    obj.object_pose.pose.orientation.w = 1;
	    
	    
	    obj.grasp_pose.pose.position.x += dx;
	    obj.grasp_pose.pose.position.y += dy;
	    obj.grasp_pose.pose.position.z += dz;
	    obj.grasp_pose.pose.orientation.x = 0;
	    obj.grasp_pose.pose.orientation.y = 0;
	    obj.grasp_pose.pose.orientation.z = 0;
	    obj.grasp_pose.pose.orientation.w = 1;
	    
	    obj.grasp_pose.pose.position.x -= 0.16;
	    
	    ROS_INFO("pose of object: %f %f %f", obj.object_pose.pose.position.x, obj.object_pose.pose.position.y, obj.object_pose.pose.position.z);
	    ROS_INFO("point to grasp: %f %f %f", obj.grasp_pose.pose.position.x, obj.grasp_pose.pose.position.y, obj.grasp_pose.pose.position.z);
	    
	    return true;	
	}
	else
	{
	    ROS_ERROR("Unable to call service for finding object");	
	    return false;
	}
    }
    
    bool moveTo(recognition_lambertian::TableTopObject &obj)
    {
	pr2_robot_actions::MoveArmGoal goal;
	int32_t                        feedback;
	
	goal.goal_constraints.set_pose_constraint_size(1);
	goal.goal_constraints.pose_constraint[0].pose = obj.grasp_pose;
	goal.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
	
	goal.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.005;
	goal.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.005;
	
	goal.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.01;
	goal.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.01;
	
	goal.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.02;
	goal.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.02;
	
	goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.1;
	goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.1;
	goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.1;
	goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.1;
	goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.1;
	goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.1;
	
	goal.goal_constraints.pose_constraint[0].orientation_importance = 0.1;
	goal.goal_constraints.pose_constraint[0].type =
	    motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z + 
	    motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y;
	
	sendPoint(obj.grasp_pose.header, obj.grasp_pose.pose.position.x, obj.grasp_pose.pose.position.y, obj.grasp_pose.pose.position.z);
	
	if (move_arm.execute(goal, feedback, ros::Duration(60.0)) != robot_actions::SUCCESS)
	{
	    ROS_ERROR("Failed achieving goal");
	    return false;
	}
	else
	    return true;
    }

    bool gripClose(void)
    {	
	std_msgs::Float64 g, fb;
	g.data = -80;
	if (gripper.execute(g, fb, ros::Duration(5.0)) != robot_actions::SUCCESS)
	    return true;
	else
	{
	    ROS_ERROR("Gripper close failed");
	    return false;
	}
    }
    
    bool gripOpen(void)
    {
	std_msgs::Float64 g, fb;
	g.data = 80;
	if (gripper.execute(g, fb, ros::Duration(5.0)) != robot_actions::SUCCESS)
	    return true;
	else
	{
	    ROS_ERROR("Gripper open failed");
	    return false;
	}	
    }

    bool graspObject(recognition_lambertian::TableTopObject &obj)
    {
	if (gripClose())
	{
	    mapping_msgs::AttachedObject ao;
	    ao.header = obj.object_pose.header;
	    ao.link_name = "r_wrist_roll_link";
	    ao.objects.push_back(obj.object);
	    ao.poses.push_back(obj.object_pose.pose);
	    pubAttach.publish(ao);
	    return true;
	}
	else return false;
    }
    
    bool releaseObject(void)
    {
	if (gripOpen())
	{
	    mapping_msgs::AttachedObject ao;
	    ao.header.stamp = ros::Time::now();
	    ao.header.frame_id = "r_wrist_roll_link";
	    ao.link_name = "r_wrist_roll_link";
	    pubAttach.publish(ao);
	    return true;
	}
	else return false;
    }
    
private:
    
    robot_actions::ActionClient<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t> move_arm;
    robot_actions::ActionClient<std_msgs::Float64, pr2_robot_actions::ActuateGripperState, std_msgs::Float64> gripper;
    
    ros::NodeHandle nh;
    ros::Publisher  pubObj;
    ros::Publisher  pubAttach;
    ros::Publisher  vmPub;
    
};

    
void spinThread(void)
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_recognition_planning");
    boost::thread th(&spinThread);
    
    CleanTable ct;
    
    recognition_lambertian::TableTopObject obj;
    
    if (ct.findObject(obj))
    {
	if (ct.moveTo(obj))
	{
	    ct.graspObject(obj);
	    recognition_lambertian::TableTopObject obj2 = obj;
	    obj2.grasp_pose.pose.position.y -= 0.25;
	    if (ct.moveTo(obj2))
	    {
		ct.releaseObject();
	    }
	}
    }
    
    
    
    /*
    mapping_msgs::ObjectInMap o1;
    o1.header.frame_id = "/base_link";
    o1.header.stamp = ros::Time::now();
    o1.id = "Part2";
    o1.action = mapping_msgs::ObjectInMap::ADD;
    o1.object.type = mapping_msgs::Object::MESH;
    //    o1.object.dimensions.resize(3);
    //    o1.object.dimensions[0] = 0.3;
    //    o1.object.dimensions[1] = 0.3;
    //    o1.object.dimensions[2] = 0.3;
    o1.pose.position.x = 0.8;
    o1.pose.position.y = -0.4;
    o1.pose.position.z = 0.6;
    
    o1.pose.orientation.x = 0.0;
    o1.pose.orientation.y = 0.0;
    o1.pose.orientation.z = 0.0;
    o1.pose.orientation.w = 1.0;
    
    o1.object.vertices.resize(3);
    o1.object.vertices[0].x = 0;
    o1.object.vertices[0].y = 0;
    o1.object.vertices[0].z = 0;

    o1.object.vertices[1].x = 1;
    o1.object.vertices[1].y = 0;
    o1.object.vertices[1].z = 0;

    o1.object.vertices[2].x = 0;
    o1.object.vertices[2].y = 1;
    o1.object.vertices[2].z = 0;

    o1.object.triangles.resize(3);
    o1.object.triangles[0] = 0;
    o1.object.triangles[1] = 1;
    o1.object.triangles[2] = 2;

    sleep(1);
    
    //    while(1){
    pub.publish(o1); 

	//    }
    th.join();
    
    return 0;
    */
    
    ROS_INFO("Done");
    
    th.join();
    
    return 0;
}
