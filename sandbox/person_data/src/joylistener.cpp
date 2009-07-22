//Author: Ian Goodfellow
//When deadman switch is not pressed and head switch is pressed,
//button 0 stops recording and button 2 starts recording
//Assumes that recording is started initially

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "joy/Joy.h"

const static int headButton = 6;
const static int deadmanButton = 4;
const static int recordButton = 2;
const static int stopButton = 0;

static bool recording = true;
static bool headButtonPressed = false;
static bool deadmanButtonPressed = false;


void startRecording()
{
	FILE * result = popen("bash `rospack find person_data`/startRecording.sh","r");
	fclose(result);
	recording = true;
}

void stopRecording()
{
	FILE * result = popen("bash `rospack find person_data`/stopRecording.sh","r");
	fclose(result);
	recording = false;
}

void joyCallback(const joy::Joy::ConstPtr & msg)
{
	int n = msg->get_buttons_size();
	
	if (headButton >= n || deadmanButton >= n || recordButton >= n || stopButton >= n)
	{
		ROS_INFO("Received message with too few buttons");
		return;
	}

	
	if (msg->buttons[headButton])
	{
		ROS_INFO("Head button pushed down");
		headButtonPressed = true;
	}
	else
	{
		ROS_INFO("Head button released");
		headButtonPressed = false;
	}

	if (msg->buttons[deadmanButton])
	{
		ROS_INFO("Deadman button pushed down");
		deadmanButtonPressed = true;
	}
	else
	{
		ROS_INFO("Deadman button released");
		deadmanButtonPressed = false;
	}


	if (headButtonPressed && ! deadmanButtonPressed)
	{
		if (msg->buttons[recordButton])
		{
			if (!recording)
			{
				startRecording();
			}
			else
			{
				ROS_INFO("You are already recording!");
			}
		}

		if (msg->buttons[stopButton])
		{
			if (recording)
			{
				stopRecording();
			}
			else
			{
				ROS_INFO("You have already stopped recording");
			}
		}
	}


	//for ( int i = 0; i < n; i++)
	//{
	//	if (msg->buttons[i])
	//		ROS_INFO("Button %d pressed",i);
	//}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "joylistener");
	ros::NodeHandle n;
	ros::Subscriber chatter_sub = n.subscribe("joy",100,joyCallback);
	ros::spin();
}
