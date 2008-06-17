
#include "kinematic_controllers.h"
#include <math.h>

using namespace KDL;
using namespace PR2;
using namespace std;


#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int kfd = 0;
struct termios cooked, raw;

kinematic_controllers myKinCon;

void top();
void object_pose();
void go_down();
void close_gripper();
void open_gripper();


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
				open_gripper();
				top();
				break;
			case '2':
				printf("You pressed 2\n");
				object_pose();
				break;
			case '3':
				printf("You pressed 3\n");
				go_down();
				break;
			case '4':
				printf("You pressed 4\n");
				close_gripper();
				break;
			case '5':
				printf("You pressed 4\n");
				object_pose();
				break;
			case '6':
				printf("You pressed 4\n");
				top();
				break;
			default:
				printf("You pressed Something Else\n");
				break;
		}
	}
}

void top()
{
	Rotation r = Rotation::RotZ(DTOR(0));
	Vector v(0.3,-0.2,0.56);
	myKinCon.go(v,r,0.05);
}

void object_pose()
{
	Rotation r = Rotation::RotZ(DTOR(90));
	Vector v(0.578,0.01,0.06);
	myKinCon.go(v,r,0.05);
}

void go_down()
{
	Rotation r = Rotation::RotZ(DTOR(90))*Rotation::RotY(DTOR(30));
	Vector v(0.578,0.01,-0.01);
	myKinCon.go(v,r,0.05);
}

void close_gripper()
{
	myKinCon.myPR2->CloseGripper(PR2::PR2_RIGHT_GRIPPER, 0.0, 10000);
}

void open_gripper()
{
	myKinCon.myPR2->CloseGripper(PR2::PR2_RIGHT_GRIPPER, 0.15, 500);
}

void init_robot()
{
	myKinCon.init();
}



int main(int argc, char **argv)
{
	init_robot();
	keyboardLoop();
	return 0;
}


