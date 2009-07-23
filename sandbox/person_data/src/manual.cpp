#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
using namespace std;

//first arg is name of topic
//optional second arg is name of node

int main(int argc, char ** argv)
{
	string topic;
	string nodename;

	if (argc >= 2)
		topic = argv[1];

	if (argc >= 3)
		nodename = argv[2];
	else
		nodename = topic+"_publisher";

	if (argc < 2 && argc > 3)
		return -1;
	
	ros::init(argc, argv, nodename.c_str());
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>(topic.c_str(),100);
	ros::Rate loop_rate(5);
	int count = 0;
	while (n.ok())
	{
		assert(false); //todo-- get line of input from  user and publish it. use this to test morgan's mux nodes
		std::stringstream ss;
		ss << "Hello there! This is message [" << count << "]";
		std_msgs::String msg;
		msg.data = ss.str();
		chatter_pub.publish(msg);
		ROS_INFO("I published [%s]",ss.str().c_str());
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
}
