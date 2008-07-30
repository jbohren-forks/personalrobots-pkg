#include "graspModuleNode.h"
#include "ros/node.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv);
	grasp_module::GraspModuleNode node;
	node.spin();
	ros::fini();
	return 0;
}
