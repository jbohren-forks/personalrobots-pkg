#include <iostream>

// ROS Stuff
#include <ros/node.h>

// Msgs
#include <std_msgs/Empty.h>
#include "robot_msgs/PoseStamped.h"
#include <robot_actions/NoArgumentsActionState.h>

#include <robot_actions/action_client.h>

int main(int argc, char** argv)
{ 
  ros::init(argc,argv);
 
  ros::Node node("exec");

  // Use a client to test the action
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> client("tiny");
  ros::Duration duration(2);
  duration.sleep();

  std_msgs::Empty a;
  std_msgs::Empty b;

  robot_actions::ResultStatus result;

  result = client.execute(a, b, ros::Duration(0.5));
  std::cout << "Result " << result << "\n";

  result = client.execute(a, b, ros::Duration(1.5));
  std::cout << "Result " << result << "\n";

  // result = client.execute(g, f, ros::Duration().fromSec(0.0001));
}
