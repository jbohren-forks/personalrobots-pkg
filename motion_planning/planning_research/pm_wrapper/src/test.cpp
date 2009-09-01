#include <ros/ros.h>
#include "ros/node_handle.h"
#include <pm_wrapper/pm_wrapper.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <mapping_msgs/CollisionMap.h>
#include <geometric_shapes/shapes.h>

pm_wrapper *pm;
btTransform pose;
btVector3 origin;
	
void callback(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
	shapes::Box *box = new shapes::Box(1.5, 1.5, 1.5);	
	pose.setIdentity();

	origin.setX(1.0);
	origin.setY(0.0);
	origin.setZ(0.0);
	pose.setOrigin(origin);
	
	ROS_INFO("calling removeObject()");

	pm->removeObject(box, pose);
	
	ROS_INFO("removeObject() finished");
	
	shapes::Box *box2 = new shapes::Box(.05, .5, .8);	

	pose.setOrigin(btVector3(1,.2,1));

	ROS_INFO("calling addObject()");

	pm->addObject("points", box2, pose);
	
	ROS_INFO("addedObject() finished");

	pm->publishInternalCollisionMap();
		
	delete box;
	//note: pm deletes box2
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_pm_wrapper", ros::init_options::AnonymousName);
	
	ros::NodeHandle nh;
	tf::TransformListener tf;

	std::vector<std::string> links;
	links.push_back("r_gripper_palm_link");
	links.push_back("r_gripper_l_finger_link");
	links.push_back("r_gripper_r_finger_link");
	links.push_back("r_gripper_l_finger_tip_link");
	links.push_back("r_gripper_r_finger_tip_link");
	
	pm = new pm_wrapper;
	
	pm->initPlanningMonitor(links, &tf, "base_link");

	ROS_INFO("initialized PM()");

	ros::Subscriber map_sub = nh.subscribe<mapping_msgs::CollisionMap>("collision_map_occ", 1, &callback);

	ros::spin();
	
	return 0;
}



