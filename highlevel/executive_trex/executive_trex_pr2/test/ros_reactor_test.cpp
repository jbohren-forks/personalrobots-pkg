#include <executive_trex_pr2/ros_reactor.h>

int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node n("trex_test_client");

  // This request is for all outlets that are reachable
  executive_trex_pr2::ExecuteGoals::Request req;
  req.outlet_ids.push_back(1);
  req.outlet_ids.push_back(4);
  req.outlet_ids.push_back(6);
  req.outlet_ids.push_back(16);
  req.outlet_ids.push_back(20);
  req.outlet_ids.push_back(21);
  req.outlet_ids.push_back(25);
  req.outlet_ids.push_back(26);
  req.outlet_ids.push_back(27);
  req.outlet_ids.push_back(38);
  req.outlet_ids.push_back(39);
  req.outlet_ids.push_back(40);

  executive_trex_pr2::ExecuteGoals::Response resp;
  ROS_ASSERT(ros::service::call("execute_goals", req, resp));
}
