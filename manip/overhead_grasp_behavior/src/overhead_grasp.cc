#include <termios.h>
#include <signal.h>
#include <math.h>

#include <ros/node.h>
#include <libpr2API/pr2API.h>
#include <pr2_msgs/EndEffectorState.h>

#include <std_msgs/Float64.h>
#include <std_msgs/PR2Arm.h>
#include <unistd.h>

using namespace KDL;

// coordinates of arm's origin in gazebo/arm coordinate frame (the axes of both frames are parallel)
Vector gazebo_to_arm_vector = Vector(1.040-0.7987,-0.15,0.8269);


void quit(int sig);
void keyboardLoop(ros::node &n);


void positionArmCartesian(ros::node &n, Vector v, Rotation r)
{
	pr2_msgs::EndEffectorState efs;
	efs.set_rot_size(9);
	efs.set_trans_size(3);
	for(int i = 0; i < 9; i++) { 
		efs.rot[i] = r.data[i];
	} 
	for(int i = 0; i < 3; i++) {
		efs.trans[i] = v.data[i];
	}

	n.publish("right_pr2arm_set_end_effector",efs);
}


void overheadGraspPosture(ros::node &n)
{
	std_msgs::PR2Arm rightarm;
	rightarm.turretAngle = deg2rad*-20;
	rightarm.shoulderLiftAngle = deg2rad*-20;
	rightarm.upperarmRollAngle = deg2rad*180;
	rightarm.elbowAngle        = deg2rad*-30;
	rightarm.forearmRollAngle  = 0;
	rightarm.wristPitchAngle   = 0;
	rightarm.wristRollAngle    = 0;
	rightarm.gripperForceCmd   = 0;
	rightarm.gripperGapCmd     = 0.2;

	n.publish("cmd_rightarmconfig",rightarm);
}


// Vector v is in gazebo coordinate frame.
void positionEyecamOverObject(ros::node &n, Vector v)
{
	Vector v_arm = v - gazebo_to_arm_vector;
	v_arm.data[2] += 0.3; // I want end effector to be 0.5m above object.
	Rotation r = Rotation::RotY(DTOR(90)); // look down vertically
	cout<<"Going to: "<<v_arm<<endl;
	positionArmCartesian(n, v_arm, r);
}


int main(int argc, char **argv)
{  
	ros::init(argc, argv);
	ros::node mynode("overhead_grasp");

	mynode.advertise<std_msgs::PR2Arm>("cmd_rightarmconfig");
	mynode.advertise<pr2_msgs::EndEffectorState>("right_pr2arm_set_end_effector");
	mynode.advertise<std_msgs::Float64>("interpolate_step_size");
	mynode.advertise<std_msgs::Float64>("interpolate_wait_time");

	std_msgs::Float64 float64_msg;
	sleep(1);

	double interpolate_step_size = 0.05;
	float64_msg.data = interpolate_step_size;
	mynode.publish("interpolate_step_size", float64_msg);
	
	double interpolate_wait_time = 1.0;
	float64_msg.data = interpolate_wait_time;
	mynode.publish("interpolate_wait_time", float64_msg);

//---- now for the keyboard driven state machine ------

	while(1)
	{
    signal(SIGINT,quit);
		keyboardLoop(mynode);
	}

	return 0;    
}



//-------- for keyboard driven state machine ----------
int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	ros::fini();
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}


void keyboardLoop(ros::node &n)
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
	printf("Numbers starting from 1 to step through the states\n");
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
				overheadGraspPosture(n);
				break;
			case '2':
				{
					Vector v(.835,-.45,0.70);
					positionEyecamOverObject(n, v);
				}
				break;
			default:
				break;
		}
	}
}


