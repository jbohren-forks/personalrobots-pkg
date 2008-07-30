#include <ros/node.h>
#include <rosthread/mutex.h>

#include <std_msgs/Empty.h>
#include <std_msgs/PR2Arm.h>
#include <std_msgs/Float64.h>
#include <pr2_msgs/EndEffectorState.h>

#include <pr2_kinematic_controllers/Float32Int32.h>

#include <libpr2API/pr2API.h>

#include <kdl/rotational_interpolation_sa.hpp>
#include <rosgazebo/VoidVoid.h>


using namespace std_msgs;
using namespace PR2;
using namespace KDL;


class Pr2KinematicControllers : public ros::node
{
	public:

		Pr2KinematicControllers(void) : ros::node("pr2_kinematic_controller") 
		{
			step_size = 0.05;
			wait_time = 1.;
			advertise<pr2_msgs::EndEffectorState>("cmd_leftarm_cartesian");
			advertise<pr2_msgs::EndEffectorState>("cmd_rightarm_cartesian");
//			advertise<std_msgs::Empty>("reset_IK_guess"); // to tell RosGazeboNode to make q_IK_guess = current manipulator config.

			subscribe("right_pr2arm_set_end_effector", _rightEndEffectorGoal, &Pr2KinematicControllers::setRightEndEffector);	
			subscribe("left_pr2arm_set_end_effector", _leftEndEffectorGoal, &Pr2KinematicControllers::setLeftEndEffector);
			subscribe("left_pr2arm_pos",  leftArmPosMsg,  &Pr2KinematicControllers::currentLeftArmPos);
			subscribe("right_pr2arm_pos", rightArmPosMsg, &Pr2KinematicControllers::currentRightArmPos);

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
			// don't need to do anything -- we already have the data
		}

		void currentRightArmPos(void)
		{
			// don't need to do anything -- we already have the data
		}

		void publishFrame(bool isRightArm, const Frame& f)
		{
			pr2_msgs::EndEffectorState efs;
			efs.set_rot_size(9);
			efs.set_trans_size(3);
			std::cout << "Publishing rot ";
			for(int i = 0; i < 9; i++)
			{
				efs.rot[i] = f.M.data[i];
				std::cout << efs.rot[i] << " ";
			}
			std::cout << " trans ";
			for(int i = 0; i < 3; i++)
			{
				efs.trans[i] = f.p.data[i];
				std::cout << efs.trans[i] << " ";
			}
			std::cout << std::endl;
			if(isRightArm)
				publish("cmd_rightarm_cartesian",efs);
			else
				publish("cmd_leftarm_cartesian",efs);
		}

		void RunControlLoop(bool isRightArm, const Frame& r, const std_msgs::PR2Arm& arm)
		{

			std_msgs::Empty emp;
			rosgazebo::VoidVoid::request req;
			rosgazebo::VoidVoid::response res;
//			publish("reset_IK_guess",emp);
			if (ros::service::call("reset_IK_guess", req, res))
				printf("Success!\n");
			else
			{
				printf("reset_IK_guess service failed.\nExiting..\n");
				exit(0);
			}

//			if (ros::service::call("reset_IK_guess", req, res)==false)
//			{
//				printf("reset_IK_guess service failed.\nExiting..\n");
//				exit(0);
//			}


			PR2_kinematics pr2_kin;
			JntArray q = JntArray(pr2_kin.nJnts);

			q(0) = arm.turretAngle;
			q(1) = arm.shoulderLiftAngle;
			q(2) = arm.upperarmRollAngle;
			q(3) = arm.elbowAngle;
			q(4) = arm.forearmRollAngle;
			q(5) = arm.wristPitchAngle;
			q(6) = arm.wristRollAngle;

			Frame f;
			pr2_kin.FK(q,f);
			Vector start = f.p;
			Vector move = r.p-start;
			double dist = move.Norm();
			move = move/dist;

			std::cout << "Starting trans " << f.p.data[0] << " " << f.p.data[1] << " " << f.p.data[2] << std::endl;

			RotationalInterpolation_SingleAxis rotInterpolater;
			rotInterpolater.SetStartEnd(f.M, r.M);
			double total_angle = rotInterpolater.Angle();
			//	printf("Angle: %f\n", rotInterpolater.Angle());

			Vector target;
			int nSteps = (int)(dist/step_size);
			double angle_step = total_angle/nSteps;
			for(int i=0;i<nSteps;i++)
			{
				f.p = start+(i+1)*move*step_size;
				f.M = rotInterpolater.Pos(angle_step*(i+1));
				std::cout << "Starting trans " << f.p.data[0] << " " << f.p.data[1] << " " << f.p.data[2] << std::endl;
				publishFrame(isRightArm, f);
				usleep(wait_time*1e6);
			}
			f.p = r.p;
			f.M = r.M;
			publishFrame(isRightArm, f);
		}

	private:

		pr2_msgs::EndEffectorState _leftEndEffectorGoal;
		pr2_msgs::EndEffectorState _rightEndEffectorGoal;
		std_msgs::PR2Arm leftArmPosMsg, rightArmPosMsg;
		double step_size;
		double wait_time;
		std_msgs::Float64 float64_msg;

};

int main(int argc, char **argv)
{  
	ros::init(argc, argv);
	Pr2KinematicControllers easy;
	easy.spin();
	easy.shutdown();
	return 0;    
}

