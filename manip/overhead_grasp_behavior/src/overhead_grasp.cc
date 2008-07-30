#include <termios.h>
#include <signal.h>
#include <math.h>

#include <ros/node.h>
#include <libpr2API/pr2API.h>
#include <pr2_msgs/EndEffectorState.h>

#include <std_msgs/Point3DFloat32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/PR2Arm.h>
#include <unistd.h>

using namespace KDL;



class OverheadGrasper : public ros::node
{
	public:
		// coordinates of arm's origin in gazebo/arm coordinate frame (the axes of both frames are parallel)
		Vector gazebo_to_arm_vector;
		std_msgs::Point3DFloat32 objectPosMsg;

	public:
		OverheadGrasper(void) : ros::node("overhead_grasper")
		{
			advertise<std_msgs::PR2Arm>("cmd_rightarmconfig");
			advertise<pr2_msgs::EndEffectorState>("right_pr2arm_set_end_effector");
			advertise<std_msgs::Float64>("interpolate_step_size");
			advertise<std_msgs::Float64>("interpolate_wait_time");
			subscribe("object_position", objectPosMsg, &OverheadGrasper::objectPosition_cb);

			gazebo_to_arm_vector = Vector(1.040-0.7987,-0.15,0.8269);
		}

		void positionArmCartesian(Vector v, Rotation r)
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

			publish("right_pr2arm_set_end_effector",efs);
		}


		void overheadGraspPosture()
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

			publish("cmd_rightarmconfig",rightarm);
		}


		// Vector v is in gazebo coordinate frame.
		void positionEyecamOverObject(Vector v)
		{
			Vector v_arm = v - gazebo_to_arm_vector;
			v_arm.data[2] += 0.4; // I want end effector to be 0.5m above object.
			Rotation r = Rotation::RotY(DTOR(90)); // look down vertically
			cout<<"Going to: "<<v_arm<<endl;
			positionArmCartesian(v_arm, r);
		}

		void objectPosition_cb(void)
		{
		}

};


void quit(int sig);
void keyboardLoop(OverheadGrasper &n);


int main(int argc, char **argv)
{  
	ros::init(argc, argv);
	OverheadGrasper ohGrasper;

	std_msgs::Float64 float64_msg;
	sleep(1);

	double interpolate_step_size = 0.05;
	float64_msg.data = interpolate_step_size;
	ohGrasper.publish("interpolate_step_size", float64_msg);
	
	double interpolate_wait_time = 1.0;
	float64_msg.data = interpolate_wait_time;
	ohGrasper.publish("interpolate_wait_time", float64_msg);

//---- now for the keyboard driven state machine ------

	while(1)
	{
    signal(SIGINT,quit);
		keyboardLoop(ohGrasper);
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


void keyboardLoop(OverheadGrasper &n)
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
				n.overheadGraspPosture();
				break;
			case '2':
				{
					Vector v(n.objectPosMsg.x,n.objectPosMsg.y,n.objectPosMsg.z);
					n.positionEyecamOverObject(v);
				}
				break;
			case '3':
				printf("x: %.4f y: %.4f z: %.4f\n", n.objectPosMsg.x, n.objectPosMsg.y, n.objectPosMsg.z);
				break;
			default:
				break;
		}
	}
}


