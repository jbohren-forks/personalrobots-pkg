
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

void go();


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
				go();
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

void go()
{
	Rotation r = Rotation::RotZ(DTOR(90));
	Vector v(0.568,0.01,0.06);
	myKinCon.go(v,r,0.05);
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


