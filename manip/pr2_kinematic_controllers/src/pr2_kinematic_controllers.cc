#include <ros/node.h>

#include <std_msgs/PR2Arm.h>
#include <std_msgs/Float64.h>
#include <pr2_msgs/EndEffectorState.h>
#include <pr2_kinematic_controllers/Float64Int32.h>
#include <rosgazebo/VoidVoid.h>
#include <rosgazebo/MoveCartesian.h>

#include <libpr2API/pr2API.h>
//#include <libKDL/kdl_kinematics.h>

#include <robot_kinematics/robot_kinematics.h>
#include <robot_kinematics/serial_chain.h>

#include <kdl/rotational_interpolation_sa.hpp>

#include <unistd.h>

using namespace std_msgs;
using namespace PR2;
using namespace KDL;

using namespace robot_kinematics;


class Pr2KinematicControllers : public ros::node
{
	public:

		Pr2KinematicControllers(void) : ros::node("pr2_kinematic_controller") 
		{
			char *c_filename = getenv("ROS_PACKAGE_PATH");
			std::stringstream filename;
			filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;
			pr2_kin.loadXML(filename.str());
			right_arm = pr2_kin.getSerialChain("rightArm");

			step_size = 0.05;
			wait_time = 1;
			advertise<pr2_msgs::EndEffectorState>("cmd_leftarm_cartesian");

			advertise<pr2_msgs::EndEffectorState>("rightarm_tooltip_cartesian"); // position of tip of right arm in cartesian coord.
			advertise_service("move_along_gripper", &Pr2KinematicControllers::moveAlongGripper);

			subscribe("right_pr2arm_set_end_effector", _rightEndEffectorGoal, &Pr2KinematicControllers::setRightEndEffector);	
			subscribe("left_pr2arm_set_end_effector", _leftEndEffectorGoal, &Pr2KinematicControllers::setLeftEndEffector);
			subscribe("left_pr2arm_pos",  leftArmPosMsg,  &Pr2KinematicControllers::currentLeftArmPos);  // configuration of left arm.
			subscribe("right_pr2arm_pos", rightArmPosMsg, &Pr2KinematicControllers::currentRightArmPos); // configuration of right arm.

			subscribe("interpolate_step_size", float64_msg, &Pr2KinematicControllers::setStepSize);	
			subscribe("interpolate_wait_time", float64_msg, &Pr2KinematicControllers::setWaitTime);	
		}

		void setWaitTime(void)
		{
			wait_time = float64_msg.data;
		}

		void setStepSize(void)
		{
			step_size = float64_msg.data;
		}

		void setRightEndEffector(void) 
		{
			KDL::Frame f;
			for(int i = 0; i < 9; i++)
				f.M.data[i] = _rightEndEffectorGoal.rot[i];

			for(int i = 0; i < 3; i++)
				f.p.data[i] = _rightEndEffectorGoal.trans[i];

			RunControlLoop(true, f, rightArmPosMsg);
		}

		void setLeftEndEffector(void) 
		{
			KDL::Frame f;
			for(int i = 0; i < 9; i++)
				f.M.data[i] = _leftEndEffectorGoal.rot[i];

			for(int i = 0; i < 3; i++)
				f.p.data[i] = _leftEndEffectorGoal.trans[i];

			RunControlLoop(false, f, leftArmPosMsg);
		}


		void currentLeftArmPos(void)
		{
		}

		void currentRightArmPos(void)
		{
			JntArray q = JntArray(right_arm->num_joints_);
			q(0) = rightArmPosMsg.turretAngle;
			q(1) = rightArmPosMsg.shoulderLiftAngle;
			q(2) = rightArmPosMsg.upperarmRollAngle;
			q(3) = rightArmPosMsg.elbowAngle;
			q(4) = rightArmPosMsg.forearmRollAngle;
			q(5) = rightArmPosMsg.wristPitchAngle;
			q(6) = rightArmPosMsg.wristRollAngle;

			Frame f;
			right_arm->computeFK(q,f);
			pr2_msgs::EndEffectorState efs;
			KDL_to_EndEffectorStateMsg(f, efs);
			publish("rightarm_tooltip_cartesian",efs);
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

		void publishFrame(bool isRightArm, const Frame& f)
		{
			pr2_msgs::EndEffectorState efs;
			KDL_to_EndEffectorStateMsg(f, efs);

			if(isRightArm)
				publish("cmd_rightarm_cartesian",efs);
			else
				publish("cmd_leftarm_cartesian",efs);
		}

		void RunControlLoop(bool isRightArm, const Frame& r, const std_msgs::PR2Arm& arm)
		{
			rosgazebo::VoidVoid::request req;
			rosgazebo::VoidVoid::response res;

			cout<<"RunControlLoop: rotation: "<<r.M<<"\n";

			if (ros::service::call("reset_IK_guess", req, res)==false)
			{
				printf("[pr2_kinematic_controllers] <pr2_kinematic_controllers.cpp> reset_IK_guess service failed.\nExiting..\n");
				exit(0);
			}

			JntArray q = JntArray(right_arm->num_joints_);
			q(0) = arm.turretAngle;
			q(1) = arm.shoulderLiftAngle;
			q(2) = arm.upperarmRollAngle;
			q(3) = arm.elbowAngle;
			q(4) = arm.forearmRollAngle;
			q(5) = arm.wristPitchAngle;
			q(6) = arm.wristRollAngle;

			Frame f;
			right_arm->computeFK(q,f);
			Vector start = f.p;
			Vector move = r.p-start;
			double dist = move.Norm();
			move = move/dist;

			RotationalInterpolation_SingleAxis rotInterpolater;
			rotInterpolater.SetStartEnd(f.M, r.M);
			double total_angle = rotInterpolater.Angle();
			//	printf("Angle: %f\n", rotInterpolater.Angle());

			Vector target;
			int nSteps = (int)(dist/step_size);
			double angle_step = total_angle/nSteps;
			bool reachable = true;
			for(int i=0;i<nSteps && reachable==true;i++)
			{
				printf("[pr2_kinematic_controllers] interpolating...\n");
				f.p = start+(i+1)*move*step_size;
				f.M = rotInterpolater.Pos(angle_step*(i+1));

				if (isRightArm)
					reachable=SetRightArmCartesian(f); // services.
				else
					publishFrame(isRightArm, f); // old way of doing things. from interpolated_kinematic_controller.

				usleep(wait_time*1e6);
			}

			f.p = r.p;
			f.M = r.M;
			if (isRightArm)
				reachable=SetRightArmCartesian(f); // services.
			else
				publishFrame(isRightArm, f); // old way of doing things. from interpolated_kinematic_controller.

			if (reachable==false)
				printf("[pr2_kinematic_controllers] reachable became FALSE.\n");
		}

		bool SetRightArmCartesian(const Frame &f)
		{
			rosgazebo::MoveCartesian::request req;
			rosgazebo::MoveCartesian::response res;
			KDL_to_EndEffectorStateMsg(f, req.e);

			if (ros::service::call("set_rightarm_cartesian", req, res)==false)
			{
				printf("[pr2_kinematic_controllers] <pr2_kinematic_controllers.cpp> set_rightarm_cartesian service failed.\nExiting..\n");
				exit(0);
			}

			return (res.reachable==-1) ? false : true;
		}

		bool moveAlongGripper(pr2_kinematic_controllers::Float64Int32::request &req, pr2_kinematic_controllers::Float64Int32::response &res)
		{
			moveAlongGripper(req.f);
			return true;
		}

		void moveAlongGripper(double dist)
		{
			JntArray q = JntArray(right_arm->num_joints_);

			q(0) = rightArmPosMsg.turretAngle;
			q(1) = rightArmPosMsg.shoulderLiftAngle;
			q(2) = rightArmPosMsg.upperarmRollAngle;
			q(3) = rightArmPosMsg.elbowAngle;
			q(4) = rightArmPosMsg.forearmRollAngle;
			q(5) = rightArmPosMsg.wristPitchAngle;
			q(6) = rightArmPosMsg.wristRollAngle;
			Frame f;
			right_arm->computeFK(q,f);
			cout<<"current end effector position: "<<f.p<<"\n";
			cout<<"current end effector rotation: "<<f.M<<"\n";
			Vector v(0,0,dist);
			v = f*v;
			cout<<"final end effector position: "<<v<<"\n";
			cout<<"final end effector rotation: "<<f.M<<"\n";
			f.p=v;
			RunControlLoop(true, f, rightArmPosMsg);
		}

	private:

		pr2_msgs::EndEffectorState _leftEndEffectorGoal;
		pr2_msgs::EndEffectorState _rightEndEffectorGoal;
		std_msgs::PR2Arm leftArmPosMsg, rightArmPosMsg;
		double step_size;
		double wait_time;
		std_msgs::Float64 float64_msg;
		
		RobotKinematics pr2_kin;
		SerialChain *right_arm;

};

int main(int argc, char **argv)
{  
	ros::init(argc, argv);
	Pr2KinematicControllers easy;
	easy.spin();
	easy.shutdown();
	return 0;    
}

