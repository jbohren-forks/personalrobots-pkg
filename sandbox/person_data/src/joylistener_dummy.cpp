//Author: Ian Goodfellow
//When deadman switch is not pressed and head switch is pressed,
//button 0 stops recording and button 2 starts recording
//Assumes that recording is started initially

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "joy/Joy.h"
#include <string>

const static int headButton = 6;
const static int deadmanButton = 4;
const static int recordButton = 2;
const static int stopButton = 0;

static bool recording = true;
static bool headButtonPressed = false;
static bool deadmanButtonPressed = false;

ros::Publisher debug_pub;

void IA3N_INFO(const char * msg)
{
  //std_msgs::String rmsg;
  //rmsg.data = std::string(msg);
  //debug_pub.publish(rmsg);
  ROS_INFO(msg);
}

void startRecording()
{
	system("bash `rospack find person_data`/startRecordingDummy.sh 1> /dev/null 2>&1");
	//pclose(result);
	recording = true;
}

void stopRecording()
{
	system("bash `rospack find person_data`/stopRecordingDummy.sh 1> /dev/null 2>&1");
	recording = false;
}



void joyCallback(const joy::Joy::ConstPtr & msg)
{
        //IA3N_INFO("Received joystick message!");

	int n = msg->get_buttons_size();
	
	//messages are sized so that they are just long enough to include the last 1

	
	if (headButton < n && msg->buttons[headButton])
	{
		//IA3N_INFO("Head button pushed down");
		headButtonPressed = true;
	}
	else
	{
		//IA3N_INFO("Head button released");
		headButtonPressed = false;
	}

	if (deadmanButton < n && msg->buttons[deadmanButton])
	{
		//IA3N_INFO("Deadman button pushed down");
		deadmanButtonPressed = true;
	}
	else
	{
		//IA3N_INFO("Deadman button released");
		deadmanButtonPressed = false;
	}


	if (headButtonPressed && ! deadmanButtonPressed)
	{
		if (recordButton < n && msg->buttons[recordButton])
		{
			if (!recording)
			{
				IA3N_INFO("Restarted recording");
				startRecording();
			}
			else
			{
				IA3N_INFO("You are already recording!");
			}
		}

		if (stopButton < n && msg->buttons[stopButton])
		{
			if (recording)
			{
				IA3N_INFO("Stopped recording");
				stopRecording();
			}
			else
			{
				IA3N_INFO("You have already stopped recording");
			}
		}
	}


	//for ( int i = 0; i < n; i++)
	//{
	//	if (msg->buttons[i])
	//		IA3N_INFO("Button %d pressed",i);
	//}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "joylistener_dummy");
	ros::NodeHandle n;
	debug_pub = n.advertise<std_msgs::String>("ia3n_debug",100);
	ros::Subscriber chatter_sub = n.subscribe("joy",100,joyCallback);
	ros::spin();
}
