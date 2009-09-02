#include <move_arm/MoveArmAction.h>
#include <manipulation_srvs/IKService.h>
#include <manipulation_msgs/JointTraj.h>
#include <experimental_controllers/TrajectoryStart.h>

#include "LinearMath/btQuaternion.h"

#include "ArmController.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
ArmController::ArmController(ros::NodeHandle &nh)
  : nodeHandle_(nh), tf_(nodeHandle_), rm_("robot_description"), km_(&rm_, &tf_)
{
  // Subscribe to the cmd topic
  armCmdSubscriber_ = nodeHandle_.subscribe("~/cmd", 10, 
      &ArmController::CmdCB, this);

  jointStateSubscriber_ = nodeHandle_.subscribe("joint_states", 100, 
      &ArmController::MechanismStateCB, this);

  gripperAction_ = new actionlib::SimpleActionClient<move_arm::ActuateGripperAction>(nodeHandle_,"actuate_gripper_right_arm");


  // Create a publisher for outputting status of the controller
  statusPublisher_ = nodeHandle_.advertise<hanoi::ArmStatus>("~/status",1,true);

  positionArmPublisher_ = nodeHandle_.advertise<move_arm::MoveArmActionGoal>(
      "/move_right_arm/goal", 1, true);

  arm_ = "right_arm";
	names_.push_back( arm_ + "_shoulder_pan_joint" );
	names_.push_back( arm_ + "_shoulder_lift_joint");
	names_.push_back( arm_ + "_upper_arm_roll_joint");
	names_.push_back( arm_ + "_elbow_flex_joint");
	names_.push_back( arm_ + "_forearm_roll_joint");
	names_.push_back( arm_ + "_wrist_flex_joint");
	names_.push_back( arm_ + "_wrist_roll_joint");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ArmController::~ArmController()
{
}

////////////////////////////////////////////////////////////////////////////////
// Arm controller command
void ArmController::CmdCB(const hanoi::ArmCmdConstPtr &msg)
{
  float shoulderX, shoulderY, shoulderZ;
  float dist;

  armCmd_ = *msg;

  shoulderX = -0.05;
  shoulderY = -0.18;
  shoulderZ = 0.745;

  dist = sqrt( pow(armCmd_.x - shoulderX, 2) + 
               pow(armCmd_.y - shoulderY, 2) + 
               pow(armCmd_.z - shoulderZ, 2) );

  printf("Dist[%f]\n",dist);

  if (dist > .82)
  {
    char msg[256];
    sprintf(msg,"Arm Pos[%f %f %f] is invalid",armCmd_.x,armCmd_.y,armCmd_.z);
    ROS_ERROR("%s\n",msg);

    hanoi::ArmStatus status;
    status.msg = msg;
    status.status = -1;

    statusPublisher_.publish( status );
    return;
  }

  btQuaternion q(armCmd_.yaw, armCmd_.pitch, armCmd_.roll);

  if (msg->mode == "plan")
  {
    this->PlanToGoal(armCmd_.x, armCmd_.y, armCmd_.z, 
                     q.getX(), q.getY(), q.getZ(), q.getW());
  }
  else if (msg->mode == "ik")
  {
    this->IKToGoal(armCmd_.x, armCmd_.y, armCmd_.z, 
                   q.getX(), q.getY(), q.getZ(), q.getW());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Plan to a goal location
void ArmController::PlanToGoal(float x, float y, float z, 
                float qx, float qy, float qz, float qw)
{
  move_arm::MoveArmActionGoal g;

  g.header.stamp = ros::Time::now();
  moveArmGoalId_ = ros::Time::now();

  std::cout << "Move Arm Goal Id[" << moveArmGoalId_.toNSec() << "]\n";

  g.goal.goal_constraints.pose_constraint.resize(1);
  g.goal.goal_constraints.pose_constraint[0].type = 
    motion_planning_msgs::PoseConstraint::POSITION_X + 
    motion_planning_msgs::PoseConstraint::POSITION_Y + 
    motion_planning_msgs::PoseConstraint::POSITION_Z +
    motion_planning_msgs::PoseConstraint::ORIENTATION_R + 
    motion_planning_msgs::PoseConstraint::ORIENTATION_P + 
    motion_planning_msgs::PoseConstraint::ORIENTATION_Y;

  g.goal.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
  g.goal.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
  g.goal.goal_constraints.pose_constraint[0].pose.header.frame_id = "/base_link";
  g.goal.goal_constraints.pose_constraint[0].pose.pose.position.x = x;
  g.goal.goal_constraints.pose_constraint[0].pose.pose.position.y = y;
  g.goal.goal_constraints.pose_constraint[0].pose.pose.position.z = z;

  g.goal.goal_constraints.pose_constraint[0].pose.pose.orientation.x = qx;
  g.goal.goal_constraints.pose_constraint[0].pose.pose.orientation.y = qy;
  g.goal.goal_constraints.pose_constraint[0].pose.pose.orientation.z = qz;
  g.goal.goal_constraints.pose_constraint[0].pose.pose.orientation.w = qw;

  g.goal.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.04;
  g.goal.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.04;
  g.goal.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.04;
  g.goal.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.04;
  g.goal.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.04;
  g.goal.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.04;

  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.35;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.35;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.35;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.35;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.35;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.35;
  g.goal.goal_constraints.pose_constraint[0].orientation_importance = 0.2;

  positionArmPublisher_.publish(g);

  hanoi::ArmStatus msg;
  msg.status = 0;
  msg.msg = "Moving to planed goal";

  statusPublisher_.publish( msg );
}

////////////////////////////////////////////////////////////////////////////////
// IK to goal
void ArmController::IKToGoal(float x, float y, float z, 
                             float qx, float qy, float qz, float qw)
{
  ros::ServiceClient client;
  ros::ServiceClient arm_ctrl;
  manipulation_srvs::IKService::Request request;
  manipulation_srvs::IKService::Response response;
  manipulation_msgs::JointTraj traj;

  client = nodeHandle_.serviceClient<manipulation_srvs::IKService>("arm_ik");
  arm_ctrl = nodeHandle_.serviceClient<experimental_controllers::TrajectoryStart>("/r_arm_joint_waypoint_controller/TrajectoryStart", true);

  request.data.pose_stamped.header.stamp = ros::Time::now();
  request.data.pose_stamped.header.frame_id = km_.getFrameId();
  request.data.pose_stamped.pose.position.x = x;
  request.data.pose_stamped.pose.position.y = y;
  request.data.pose_stamped.pose.position.z = z;
  request.data.pose_stamped.pose.orientation.x = qx;
  request.data.pose_stamped.pose.orientation.y = qy;
  request.data.pose_stamped.pose.orientation.z = qz;
  request.data.pose_stamped.pose.orientation.w = qw;
  request.data.joint_names = names_;

  experimental_controllers::TrajectoryStart::Request  send_traj_start_req;
  experimental_controllers::TrajectoryStart::Response send_traj_start_res;

  planning_models::StateParams robot_state(*km_.getRobotState());
  for(unsigned int i = 0; i < names_.size() ; ++i)
  {
    const unsigned int u = km_.getKinematicModel()->getJoint(names_[i])->usedParams;
    for (unsigned int j = 0 ; j < u ; ++j)
      request.data.positions.push_back(*(robot_state.getParamsJoint(names_[i])));
  }

  if (client.call(request, response))
  {
    ROS_INFO("Received an IK solution");
    traj.names = names_;
    traj.points.resize(2);
    traj.points[0].time = 0.2;
    traj.points[1].time = 0.8;

    for(unsigned  k = 0; k < names_.size(); ++k)
    {
      traj.points[0].positions.push_back(request.data.positions[k]);
      traj.points[1].positions.push_back(response.solution[k]);
    }

    send_traj_start_req.traj = traj;	    
    send_traj_start_req.hastiming = 0;
    send_traj_start_req.requesttiming = 0;

    if (arm_ctrl.call(send_traj_start_req, send_traj_start_res))
    {
      int trajectoryId = send_traj_start_res.trajectoryid;
      if (trajectoryId < 0)
        ROS_ERROR("Invalid trajectory id: %d", trajectoryId);
      else
        ROS_INFO("Sent trajectory %d to controller", trajectoryId);
    }
    else
      ROS_ERROR("Unable to start trajectory controller");

    std::cout << "Success!" << std::endl;
  }
  else
    std::cerr << "IK Failed" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
/// Joint state callback
void ArmController::MechanismStateCB(const pr2_mechanism_msgs::MechanismStateConstPtr &msg)
{
  jointstates_ = *msg;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the gripper is open
bool ArmController::GripperIsOpen()
{
  if (jointstates_.joint_states.size() <= 0)
    return false;

  for (unsigned int i=0; i < jointstates_.joint_states.size(); i++)
  {
    if (jointstates_.joint_states[i].name == "r_gripper_joint")
      if (jointstates_.joint_states[i].position >= 0.05)
        return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the gripper is closed
bool ArmController::GripperIsClosed()
{
  if (jointstates_.joint_states.size() <= 0)
    return false;

  for (unsigned int i=0; i < jointstates_.joint_states.size(); i++)
  {
    if (jointstates_.joint_states[i].name == "r_gripper_joint")
      if (jointstates_.joint_states[i].position <= 0.002)
        return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
/// Close the gripper
void ArmController::CloseGripper()
{
  move_arm::ActuateGripperGoal g;
  g.data = -50.0;

  gripperAction_->sendGoal(g);

  bool status = gripperAction_->waitForGoalToFinish(ros::Duration(5));
  if (status)
    std::cout << "Gripper State:" << gripperAction_->getTerminalState().toString() << "\n";
  else
    std::cout << "Gripper unable to reach goal\n";
}

////////////////////////////////////////////////////////////////////////////////
// Open the gripper
void ArmController::OpenGripper()
{
  move_arm::ActuateGripperGoal g;
  g.data = 50.0;

  gripperAction_->sendGoal(g);

  bool status = gripperAction_->waitForGoalToFinish(ros::Duration(5));
  if (status)
    std::cout << "Gripper State:" << gripperAction_->getTerminalState().toString() << "\n";
  else
    std::cout << "Gripper unable to reach goal\n";

}
