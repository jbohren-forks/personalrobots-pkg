//pointhead.py doesn't seem to work from a launch file and only sends
//one message, so I am writing a node that calls it repeatedly

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
using namespace std;

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "headhack");
	ros::NodeHandle n;
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",100);
	ros::Rate loop_rate(1);


	std::string command = "export PERSON_DATA=`rospack find person_data`; ";
	command += "export POINT_HEAD=`rospack find pr2_mechanism_controllers`/scripts/pointhead.py; ";
	command += "$POINT_HEAD `cat ${PERSON_DATA}/headpan.txt` `cat ${PERSON_DATA}/headtilt.txt`";

	while (n.ok())
	{
		FILE * output = popen(command.c_str(), "r");
		fclose(output);
		//std::stringstream ss;
		//ss << "Hello there! This is message [" << count << "]";
		//std_msgs::String msg;
		//msg.data = ss.str();
		//chatter_pub.publish(msg);
		//ROS_INFO("I published [%s]",ss.str().c_str());
		//ros::spinOnce();
		//loop_rate.sleep();
		//++count;
	}
}
