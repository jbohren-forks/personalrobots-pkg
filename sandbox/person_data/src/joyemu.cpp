//emulates the joystick for offline testing

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "joy/Joy.h"
#include <iostream>
using namespace std;

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "joyemu");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<joy::Joy>("joy",100);
	ros::Rate loop_rate(5);


	joy::Joy joy_msg;

	joy_msg.set_buttons_size(100);

	while (n.ok())
	{
	  for (int i = 0; i < 100; i++)
	    joy_msg.buttons[i] = 0;

	  cout << "1. Send record stop" << endl;
	  cout << "2. Send record start" << endl;
	  cout << "3. Quit" << endl;

	  joy_msg.buttons[6] = 1; //press head button to enable record starting/stopping
	  

	  int choice;
	  cin >> choice;

	  if (choice == 3)
	    return 0;

	  assert(choice == 1 || choice == 2);

	  if (choice == 1)
	    joy_msg.buttons[0] = 1;
	  else
	    joy_msg.buttons[2] = 1;


	  chatter_pub.publish(joy_msg);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
}
