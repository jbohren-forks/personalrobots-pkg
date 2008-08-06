
#include <libpr2API/pr2API.h>
#include <pr2Core/pr2Core.h>
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
PR2::PR2Robot myPR2;

void object_pose();
void open_gripper();
void close_gripper();
void go_down();


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

	// get arm into a starting position.
	JntArray pr2_config = JntArray(myPR2.pr2_kin.nJnts);
	pr2_config(0) = 0.1, pr2_config(1) = -1, pr2_config(2)=0.3, pr2_config(3)=0.3;
	pr2_config(4) = 0.2, pr2_config(5) = 0.5, pr2_config(6) = 0.0;

	Frame f;
	myPR2.pr2_kin.FK(pr2_config,f);
	// send command to robot

	(*myPR2.pr2_kin.q_IK_guess)(0) = 0.0, (*myPR2.pr2_kin.q_IK_guess)(1) = 0.0, (*myPR2.pr2_kin.q_IK_guess)(2) = 0.0;
	(*myPR2.pr2_kin.q_IK_guess)(3) = 0.0,	(*myPR2.pr2_kin.q_IK_guess)(4) = 0.0, (*myPR2.pr2_kin.q_IK_guess)(5) = 0.0;
	(*myPR2.pr2_kin.q_IK_guess)(6) = 0.0;

	bool reachable;
	myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f, reachable);

}

void close_gripper()
{
	myPR2.hw.CloseGripper(PR2::PR2_RIGHT_GRIPPER, 0.05, 500);
}

void open_gripper()
{
	myPR2.hw.CloseGripper(PR2::PR2_RIGHT_GRIPPER, 0.15, 500);
}

void go_down()
{
	// called after object_pose so q_IK_guess should already be correct.
	Rotation r = Rotation::RotZ(DTOR(90));
	Vector v(0.568,0.01,-0.01);
	Frame f(r,v);

	bool reachable;
	myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f, reachable);
}

void object_pose()
{
	Rotation r = Rotation::RotZ(DTOR(90));
	
//	Vector v(0.56,0.01,-0.02);
	Vector v(0.568,0.01,0.06);

	Frame f(r,v);

	(*myPR2.pr2_kin.q_IK_guess)(0) = DTOR(-20); // turret
	(*myPR2.pr2_kin.q_IK_guess)(1) = DTOR(-20); // shoulder pitch
	(*myPR2.pr2_kin.q_IK_guess)(2) = DTOR(40);  // shoulder roll
	(*myPR2.pr2_kin.q_IK_guess)(3) = DTOR(60);  // elbow pitch
	(*myPR2.pr2_kin.q_IK_guess)(4) = DTOR(20);  // elbow roll
	(*myPR2.pr2_kin.q_IK_guess)(5) = DTOR(30);  // wrist pitch
	(*myPR2.pr2_kin.q_IK_guess)(6) = 0.0;       // wrist roll

	bool reachable;
	myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f, reachable);
}


int main(int argc, char **argv)
{
	init_robot();
	keyboardLoop();
	return 0;
}


