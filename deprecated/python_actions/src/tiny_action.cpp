#include <iostream>

// ROS Stuff
#include <ros/node.h>

// Msgs
#include <std_msgs/Empty.h>
#include "geometry_msgs/PoseStamped.h"

#include <robot_actions/NoArgumentsActionState.h>

// Robot Action Stuff
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>

class TinyAction: public robot_actions::Action<std_msgs::Empty, std_msgs::Empty> {
public:

  TinyAction(): robot_actions::Action<std_msgs::Empty, std_msgs::Empty>("tiny") {}

private:

  virtual robot_actions::ResultStatus execute(const std_msgs::Empty& goal, std_msgs::Empty& feedback) {

    int count = 1;

    while (!isPreemptRequested() && count <= 10) {
      ros::Duration duration(0.1);
      duration.sleep();

      std::cout << "HELLO, count is " << count << "\n";

      count += 1;
      update(feedback);
    }

    if (isPreemptRequested()) {
      return robot_actions::PREEMPTED;
    } else {
      return robot_actions::SUCCESS;
    }
  }
};

int main(int argc, char** argv)
{ 
  ros::init(argc,argv);
 
  ros::Node node("tiny");
  TinyAction tiny;
 
  robot_actions::ActionRunner runner(10.0);
  runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(tiny);
  runner.run();
 
  node.spin();
  return 0;
}
