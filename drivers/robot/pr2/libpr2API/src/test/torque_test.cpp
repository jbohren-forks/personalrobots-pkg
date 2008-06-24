
#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Core.h>
#include <math.h>

#include <time.h>

using namespace PR2;
using namespace std;


#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int kfd = 0;
struct termios cooked, raw;
PR2::PR2Robot myPR2;


void keyboardLoop()
{
	char c;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("---------------------------");

	for(;;)
	{
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		switch(c)
		{
			case '1':
				printf("You pressed 1\n");
				break;
			case '2':
				printf("You pressed 2\n");
				break;
			case '3':
				printf("You pressed 3\n");
				break;
			case '4':
				printf("You pressed 4\n");
				break;
			default:
				printf("You pressed Something Else\n");
				break;
		}
	}
}


void init_robot()
{
	// test pr2API
	myPR2.InitializeRobot();
	// set random numbers to base cartesian control
	myPR2.SetBaseControlMode(PR2_CARTESIAN_CONTROL);
	myPR2.SetArmControlMode(PR2_RIGHT_ARM, PR2_CARTESIAN_CONTROL);
	myPR2.SetArmControlMode(PR2_LEFT_ARM, PR2_CARTESIAN_CONTROL);

//	myPR2.SetJointControlMode(ARM_R_SHOULDER_PITCH,PD_CONTROL);
//	myPR2.SetJointServoCmd(ARM_R_SHOULDER_PITCH, 1.5, 0);

	myPR2.hw.SetJointControlMode(ARM_R_SHOULDER_PITCH,TORQUE_CONTROL);
	myPR2.hw.SetJointTorque(ARM_R_SHOULDER_PITCH, -500);
}


int main(int argc, char **argv)
{
	init_robot();
	keyboardLoop();
	return 0;
}


