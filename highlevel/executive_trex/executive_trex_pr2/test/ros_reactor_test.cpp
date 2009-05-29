#include <executive_trex_pr2/ros_reactor.h>


int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node n("trex_test_client");
  executive_trex_pr2::ExecuteGoals::Request req;
  executive_trex_pr2::ExecuteGoals::Response resp;
  ROS_ASSERT(ros::service::call("execute_goals", req, resp));
}
