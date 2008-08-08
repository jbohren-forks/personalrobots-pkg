#include <termios.h>
#include <signal.h>
#include <math.h>

#include <ros/node.h>
#include <libpr2API/pr2API.h>
#include <pr2_msgs/EndEffectorState.h>

#include <pr2_kinematic_controllers/Float64Int32.h>

#include <std_msgs/Point3DFloat32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/PR2Arm.h>
#include <unistd.h>

#include <rosgazebo/GripperCmd.h>
#include <std_msgs/PR2Arm.h>


using namespace KDL;

class OverheadGrasper : public ros::node
{
	public:
		// coordinates of arm's origin in gazebo/arm coordinate frame (the axes of both frames are parallel)
		Vector gazebo_to_arm_vector;
		std_msgs::Point3DFloat32 objectPosMsg;
    pr2_msgs::EndEffectorState rightEndEffectorMsg;
		Frame right_tooltip_frame;
		Vector objectPosition;
		const static double PR2_GRIPPER_LENGTH = 0.155;
		Vector CAMERA_ENDEFFECTOR; // position of camera relative to end effector in end effector frame.

	public:
		OverheadGrasper(void) : ros::node("overhead_grasper")
		{
			advertise<std_msgs::PR2Arm>("cmd_rightarmconfig");
			advertise<pr2_msgs::EndEffectorState>("right_pr2arm_set_end_effector"); // this should actually be a service.
			advertise<std_msgs::Float64>("interpolate_step_size");
			advertise<std_msgs::Float64>("interpolate_wait_time");
			subscribe("object_position", objectPosMsg, &OverheadGrasper::objectPosition_cb);
			subscribe("rightarm_tooltip_cartesian", rightEndEffectorMsg, &OverheadGrasper::currentRightArmPosCartesian_cb);

//			gazebo_to_arm_vector = Vector(1.020-0.7987,-0.15,0.8269);
			gazebo_to_arm_vector = Vector(0.81-0.82025,-0.20,0.739675);

			CAMERA_ENDEFFECTOR = Vector(0.0, 0.,0.05); // position of camera relative to end effector in base frame.
		}

		void KDL_to_EndEffectorStateMsg(const Frame& f, pr2_msgs::EndEffectorState &efs)
		{
			efs.set_rot_size(9);
			efs.set_trans_size(3);
			for(int i = 0; i < 9; i++)
				efs.rot[i] = f.M.data[i];
			for(int i = 0; i < 3; i++)
				efs.trans[i] = f.p.data[i];
		}

		void EndEffectorStateMsg_to_KDL(const pr2_msgs::EndEffectorState &efs, Frame& f)
		{
			for(int i = 0; i < 9; i++)
				f.M.data[i] = efs.rot[i];
			for(int i = 0; i < 3; i++)
				f.p.data[i] = efs.trans[i];
		}

		void positionArmCartesian(const Vector& v, const Rotation& r)
		{
			pr2_msgs::EndEffectorState efs;
			Frame f(r,v);
			KDL_to_EndEffectorStateMsg(f,efs);

			publish("right_pr2arm_set_end_effector",efs);
		}

		void overheadGraspPosture()
		{
			std_msgs::PR2Arm rightarm;
//			rightarm.turretAngle = deg2rad*0;
//			rightarm.shoulderLiftAngle = deg2rad*0;
//			rightarm.upperarmRollAngle = deg2rad*0;
//			rightarm.elbowAngle        = deg2rad*0;
//			rightarm.forearmRollAngle  = 0;
//			rightarm.wristPitchAngle   = deg2rad*90;
//			rightarm.wristRollAngle    = 0;
//			rightarm.gripperForceCmd   = 50;
//			rightarm.gripperGapCmd     = 0.;

			rightarm.turretAngle = deg2rad*-20;
			rightarm.shoulderLiftAngle = deg2rad*-20;
			rightarm.upperarmRollAngle = deg2rad*180;
			rightarm.elbowAngle        = deg2rad*-30;
			rightarm.forearmRollAngle  = 0;
			rightarm.wristPitchAngle   = 0;
			rightarm.wristRollAngle    = 0;
			rightarm.gripperForceCmd   = 50;
			rightarm.gripperGapCmd     = 0.;

			publish("cmd_rightarmconfig",rightarm);
		}

		// Vector v is shoulder coordinate frame.
//		void positionEyecamOverObject(Vector v)
		void positionEyecamOverObject()
		{
			Vector v_arm = objectPosition;
			v_arm.data[2] += 0.3; // I want end effector to be 0.4m above object.
			Rotation r = Rotation::RotY(deg2rad*90) * Rotation::RotY(deg2rad*90); // look down vertically
			cout<<"Going to: "<<v_arm<<endl;
			positionArmCartesian(v_arm, r);
		}

		void moveArmSegmentation()
		{
//			gmmseg::hrl_grasp:request req;
//			gmmseg::hrl_grasp:response res;
//			req.height = right_tooltip_frame.p[2] - objectPosition[2] - CAMERA_ENDEFFECTOR[0];
//			if (ros::service::call("hrl_grasp", req, res)==false)
//			{
//				printf("[overhead_grasp_behavior] <overhead_grasp.cpp> {segmentObject} hrl_grasp service failed.\nExiting..\n");
//				exit(0);
//			}
//			Vector move(-1*res.y, -1*res.x, 0);
//			Rotation r = Rotation::RotY(deg2rad*90)*Rotation::RotX(res.theta); // look down vertically, with correct wrist roll

			Vector move(0,0,0);
			move += Rotation::RotY(deg2rad*90)*CAMERA_ENDEFFECTOR;
			Rotation r = Rotation::RotY(deg2rad*90)*Rotation::RotY(deg2rad*90); // look down vertically, with correct wrist roll
			Vector goto_point = right_tooltip_frame.p+move;
			cout<<"Going to: "<<goto_point<<endl;
			positionArmCartesian(goto_point, r);
		}

		void objectPosition_cb(void)
		{
//			cout<<"object's z coord: "<<objectPosMsg.z<<"\n";
			objectPosition = Vector(objectPosMsg.x,objectPosMsg.y,objectPosMsg.z) - gazebo_to_arm_vector;
		}

		void currentRightArmPosCartesian_cb(void)
		{
			EndEffectorStateMsg_to_KDL(this->rightEndEffectorMsg, this->right_tooltip_frame);
		}

		void OpenGripper(void)
		{
			rosgazebo::GripperCmd::request req;
			rosgazebo::GripperCmd::response res;
			req.gap=0.3;
			req.force=50;
			if (ros::service::call("operate_right_gripper", req, res)==false)
			{
				printf("[overhead_grasp_behavior] <overhead_grasp.cpp> {OpenGripper} operate_right_gripper service failed.\nExiting..\n");
				exit(0);
			}
		}

		void CloseGripper(void)
		{
			rosgazebo::GripperCmd::request req;
			rosgazebo::GripperCmd::response res;
			req.gap=0.0;
			req.force=200;
			if (ros::service::call("operate_right_gripper", req, res)==false)
			{
				printf("[overhead_grasp_behavior] <overhead_grasp.cpp> {CloseGripper} operate_right_gripper service failed.\nExiting..\n");
				exit(0);
			}
		}

		void pickUpObject(void)
		{
			pr2_kinematic_controllers::Float64Int32::request req;
			pr2_kinematic_controllers::Float64Int32::response res;
			req.f = -1*PR2_GRIPPER_LENGTH;
			ros::service::call("move_along_gripper", req, res);
		}

		void printTooltipTransformation(void)
		{
			cout<<"End effector transformation: "<<this->right_tooltip_frame<<"\n";
		}

		void graspFromAbove(void)
		{
			pr2_kinematic_controllers::Float64Int32::request req;
			pr2_kinematic_controllers::Float64Int32::response res;
			printf("arm tip z: %.4f\nobjectPosition.z: %.4f\n", right_tooltip_frame.p.data[2],objectPosition.data[2]);
			req.f = right_tooltip_frame.p.data[2] - objectPosition.data[2] - PR2_GRIPPER_LENGTH;
			printf("amount to move by: %.4f\n", req.f);
			ros::service::call("move_along_gripper", req, res);
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

	double interpolate_step_size = 0.01;
	float64_msg.data = interpolate_step_size;
	ohGrasper.publish("interpolate_step_size", float64_msg);
	
	double interpolate_wait_time = .3;
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
					n.positionEyecamOverObject();
				}
				break;
			case '3':
				n.graspFromAbove();
				break;
			case '4':
				n.pickUpObject();
				break;

			case 't':
				n.moveArmSegmentation();
				break;

			case 'e':
				n.printTooltipTransformation();
				break;

			case 's':
				{
					pr2_kinematic_controllers::Float64Int32::request req;
					pr2_kinematic_controllers::Float64Int32::response res;
					req.f=0.01;
					ros::service::call("move_along_gripper", req, res);
				}

				//				printf("x: %.4f y: %.4f z: %.4f\n", n.objectPosMsg.x, n.objectPosMsg.y, n.objectPosMsg.z);
				break;
			case 'o':
			case 'O':
				n.OpenGripper();
				break;
			case 'c':
			case 'C':
				n.CloseGripper();
				break;
			default:
				break;
		}
	}
}


