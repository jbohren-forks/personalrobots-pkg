

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm/MoveArmAction.h>
#include <move_arm/ActuateGripperAction.h>

#include <mapping_msgs/AttachedObject.h>

#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <motion_planning_msgs/KinematicPath.h>
#include <manipulation_msgs/JointTraj.h>
#include <manipulation_srvs/IKService.h>
#include <experimental_controllers/TrajectoryStart.h>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <move_arm_tools/ArmCtrlCmd.h>
#include <motion_planning_msgs/GetMotionPlan.h>

class MoveArmTools
{
	public:
		
		MoveArmTools(){};
		
		~MoveArmTools();
		
		bool init(std::string arm);
		
	private:
		
		ros::NodeHandle nh_;
		
		std::string group_;
		
		ros::Publisher view_;
		
		ros::Publisher pubAttach_;
		
		ros::ServiceServer arm_ctrl_service_;

		std::map<std::string, move_arm::MoveArmGoal> goals_;
    
		std::vector<std::string> names_;
		
		planning_environment::RobotModels *rm_;
		
		ros::ServiceClient arm_ctrl_;
		
		ros::ServiceClient clientPlan_;
		
		actionlib::SimpleActionClient<move_arm::MoveArmAction> *move_arm_;
		
		actionlib::SimpleActionClient<move_arm::ActuateGripperAction> *gripper_;
		    
		boost::thread th_;
		
		tf::TransformListener tf_;
		
		planning_environment::KinematicModelStateMonitor *km_;

		double allowed_time_;
	
		/** ------ */
		
		bool callback(move_arm_tools::ArmCtrlCmd::Request &req, move_arm_tools::ArmCtrlCmd::Response &res);
				
		int goToPlan(ros::NodeHandle &nh, planning_environment::KinematicModelStateMonitor &km, move_arm::MoveArmGoal &goal, const std::vector<std::string> &names);
		
		void printHelp(void);

		void printJoints(const planning_environment::KinematicModelStateMonitor &km, const std::vector<std::string> &names);

		void printPose(const btTransform &p);

		void goalToState(const move_arm::MoveArmGoal &goal, planning_models::KinematicState &sp);

		btTransform effPosition(const planning_environment::KinematicModelStateMonitor &km, const move_arm::MoveArmGoal &goal);

		void printConfig(const move_arm::MoveArmGoal &goal);
		
		void printConfigs(const std::map<std::string, move_arm::MoveArmGoal> &goals);

		void setupGoal(const std::vector<std::string> &names, move_arm::MoveArmGoal &goal);

		void setupGoalEEf(const std::string &link, const std::vector<double> &pz, move_arm::MoveArmGoal &goal);

		void getIK(bool r, ros::NodeHandle &nh, planning_environment::KinematicModelStateMonitor &km, move_arm::MoveArmGoal &goal, planning_models::KinematicState &sp, const std::vector<std::string> &names, double x, double y, double z);

		void goToIK(ros::NodeHandle &nh,  planning_environment::KinematicModelStateMonitor &km, const std::vector<std::string> &names, double x, double y, double z);

		void diffConfig(const planning_environment::KinematicModelStateMonitor &km, move_arm::MoveArmGoal &goal);

		void viewState(ros::Publisher &view, const planning_environment::KinematicModelStateMonitor &km, const planning_models::KinematicState &st);

		void setConfigJoint(const unsigned int pos, const double value, move_arm::MoveArmGoal &goal);
		
		void setConfig(const planning_models::KinematicState *_sp, const std::vector<std::string> &names, move_arm::MoveArmGoal &goal);
				
		void spinThread(void);
};
