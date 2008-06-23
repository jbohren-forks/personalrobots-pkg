
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

JntArray object_pose();
void open_gripper();
void close_gripper();
void go_down(const JntArray &guess);


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

	JntArray q = JntArray(myPR2.pr2_kin.nJnts);
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
				q = object_pose();
				break;
			case '3':
				printf("You pressed 3\n");
				go_down(q);
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
	JntArray q_init = JntArray(myPR2.pr2_kin.nJnts);
	q_init(0) = 0.0, q_init(1) = 0.0, q_init(2) = 0.0, q_init(3) = 0.0;
	q_init(4) = 0.0, q_init(5) = 0.0, q_init(6) = 0.0;
	JntArray q_out = JntArray(myPR2.pr2_kin.nJnts);
	myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f,q_init,q_out);

}

void close_gripper()
{
	myPR2.hw.CloseGripper(PR2::PR2_RIGHT_GRIPPER, 0.05, 500);
}

void open_gripper()
{
	myPR2.hw.CloseGripper(PR2::PR2_RIGHT_GRIPPER, 0.15, 500);
}

void go_down(const JntArray &guess)
{
	Rotation r = Rotation::RotZ(DTOR(90));
	Vector v(0.568,0.01,-0.01);
	Frame f(r,v);
	JntArray q_out = JntArray(myPR2.pr2_kin.nJnts);
	myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f,guess,q_out);
}

JntArray object_pose()
{
	Rotation r = Rotation::RotZ(DTOR(90));
	
//	Vector v(0.56,0.01,-0.02);
	Vector v(0.568,0.01,0.06);

	Frame f(r,v);
	JntArray q_init = JntArray(myPR2.pr2_kin.nJnts);
	q_init(0) = DTOR(-30); // turret
	q_init(1) = DTOR(-20); // shoulder pitch
	q_init(2) = DTOR(60);  // shoulder roll
	q_init(3) = DTOR(60);  // elbow pitch
	q_init(4) = DTOR(20);  // elbow roll
	q_init(5) = DTOR(30);  // wrist pitch
	q_init(6) = 0.0;       // wrist roll

//	myPR2.pr2_kin.FK(q_init,f);
//	cout<<"end effector frame: "<<f<<endl;

	q_init(0) = DTOR(-20); // turret
	q_init(1) = DTOR(-20); // shoulder pitch
	q_init(2) = DTOR(40);  // shoulder roll
	q_init(3) = DTOR(60);  // elbow pitch
	q_init(4) = DTOR(20);  // elbow roll
	q_init(5) = DTOR(30);  // wrist pitch
	q_init(6) = 0.0;       // wrist roll

	JntArray q_out = JntArray(myPR2.pr2_kin.nJnts);
	myPR2.SetArmCartesianPosition(PR2::PR2_RIGHT_ARM,f,q_init,q_out);
	return q_out;

/*
	end effector frame: [[   +0.592488,   +0.135547,   +0.794094;
	+0.741583,   +0.293264,   -0.603367;
	-0.314663,   +0.946374,  +0.0732355]
		[   +0.698411,   -0.130820,  +0.0626092]]
*/

//	double jntArr[7] = {q_init(0),q_init(1),q_init(2),q_init(3),q_init(4),q_init(5),q_init(6)};
//	double jntSpd[7] = {0,0,0,0,0,0,0};
//	myPR2.SetArmJointPosition(PR2::PR2_RIGHT_ARM, jntArr,jntSpd);

}


int main(int argc, char **argv)
{
	init_robot();
	keyboardLoop();
	return 0;
}


