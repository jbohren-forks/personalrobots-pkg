#include <move_arm/MoveArmAction.h>
#include <manipulation_srvs/IKService.h>
#include <manipulation_msgs/JointTraj.h>
#include <experimental_controllers/TrajectoryStart.h>

#include <visualization_msgs/Marker.h>

#include "LinearMath/btQuaternion.h"

#include "ArmController.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
ArmController::ArmController(ros::NodeHandle &nh)
  : nodeHandle_(nh), tf_(nodeHandle_), rm_("robot_description")
{
  arm_ = "r";
	names_.push_back( arm_ + "_shoulder_pan_joint" );
	names_.push_back( arm_ + "_shoulder_lift_joint");
	names_.push_back( arm_ + "_upper_arm_roll_joint");
	names_.push_back( arm_ + "_elbow_flex_joint");
	names_.push_back( arm_ + "_forearm_roll_joint");
	names_.push_back( arm_ + "_wrist_flex_joint");
	names_.push_back( arm_ + "_wrist_roll_joint");

  if (!rm_.loadedModels())
  {
    ROS_ERROR("Failed to load the robot models");
    return;
  }

  printf("Waiting for state...\n");
  km_ = new planning_environment::KinematicModelStateMonitor(&rm_, &tf_);
  km_->waitForState();
  printf("got state\n");

  this->PrintJoints();

  // Subscribe to the cmd topic
  armCmdSubscriber_ = nodeHandle_.subscribe("~/cmd", 10, 
      &ArmController::CmdCB, this);

  moveArmStatusSubscriber_ = nodeHandle_.subscribe("/move_right_arm/status", 10,
      &ArmController::MoveArmStatusCB, this);

  jointStateSubscriber_ = nodeHandle_.subscribe("joint_states", 100, 
      &ArmController::MechanismStateCB, this);

  gripperAction_ = new actionlib::SimpleActionClient<move_arm::ActuateGripperAction>(nodeHandle_,"actuate_gripper_right_arm");

  visPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>( 
      "visualization_marker", 0 );
  
  // Create a publisher for outputting status of the controller
  statusPublisher_ = nodeHandle_.advertise<hanoi::ArmStatus>("~/status",1,true);

  positionArmPublisher_ = nodeHandle_.advertise<move_arm::MoveArmActionGoal>(
      "/move_right_arm/goal", 1, true);

  this->IKToGoal(0.4, -0.6, 0.9, 0,0,0,1);

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
  if (msg->action == "open")
  {
    this->OpenGripper();
  }
  else if (msg->action == "close")
  {
    this->CloseGripper();
  }
  else if (msg->action == "move")
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
      status.status = "invalid";

      statusPublisher_.publish( status );
      return;
    }

    btQuaternion q(armCmd_.yaw, armCmd_.pitch, armCmd_.roll);

    goalx_ = armCmd_.x;
    goaly_ = armCmd_.y;
    goalz_ = armCmd_.z;

    goalqx_ = q.getX();
    goalqy_ = q.getY();
    goalqz_ = q.getZ();
    goalqw_ = q.getW();

    if (msg->mode == "plan")
    {
      myState_ = "planning";
      printf("Planning to goal\n");
      this->PlanToGoal(armCmd_.x, armCmd_.y, armCmd_.z, 
          q.getX(), q.getY(), q.getZ(), q.getW());
    }
    else if (msg->mode == "ik")
    {
      myState_ = "moving";
      printf("IK move\n");
      this->IKToGoal(armCmd_.x, armCmd_.y, armCmd_.z, 
          q.getX(), q.getY(), q.getZ(), q.getW());
    }
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
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "hanoi";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = qx;
  marker.pose.orientation.y = qy;
  marker.pose.orientation.z = qz;
  marker.pose.orientation.w = qw;
  marker.scale.x = .2;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  visPublisher_.publish( marker );

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

  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.035;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.035;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.035;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.035;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.035;
  g.goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.035;
  g.goal.goal_constraints.pose_constraint[0].orientation_importance = 0.2;

  positionArmPublisher_.publish(g);

  hanoi::ArmStatus msg;
  msg.status = "moving";

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

  //client = nodeHandle_.serviceClient<manipulation_srvs::IKService>("arm_ik");
  client = nodeHandle_.serviceClient<manipulation_srvs::IKService>("/pr2_ik_right_arm/ik_service");
  arm_ctrl = nodeHandle_.serviceClient<experimental_controllers::TrajectoryStart>("/r_arm_joint_waypoint_controller/TrajectoryStart", true);

  request.data.pose_stamped.header.stamp = ros::Time::now();
  request.data.pose_stamped.header.frame_id = km_->getFrameId();
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

  planning_models::StateParams robot_state(*km_->getRobotState());
  for(unsigned int i = 0; i < names_.size() ; ++i)
  {
    const unsigned int u = km_->getKinematicModel()->getJoint(names_[i])->usedParams;

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
void ArmController::MechanismStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
  jointstates_ = *msg;
}

////////////////////////////////////////////////////////////////////////////////
// Get the move arm status message
void ArmController::MoveArmStatusCB(const actionlib_msgs::GoalStatusArrayConstPtr &msg)
{
  /*moveArmStatus_ = *msg;

  printf("Status[%d]\n",moveArmStatus_.status_list[0].status);

  // Move arm failed, try ik
  if (myState_ == "moving" && moveArmStatus_.status_list[0].status == 4)
  {
    printf("Plan failed...trying IK\n");
    //this->IKToGoal(goalx_, goaly_, goalz_, goalqx_, goalqy_, goalqz_, goalqw_);
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the gripper is open
bool ArmController::GripperIsOpen()
{
  if (jointstates_.name.size() <= 0)
    return false;

  for (unsigned int i=0; i < jointstates_.name.size(); i++)
  {
    if (jointstates_.name[i] == "r_gripper_joint")
      if (jointstates_.position[i] >= 0.05)
        return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the gripper is closed
bool ArmController::GripperIsClosed()
{
  if (jointstates_.name.size() <= 0)
    return false;

  for (unsigned int i=0; i < jointstates_.name.size(); i++)
  {
    if (jointstates_.name[i] == "r_gripper_joint")
      if (jointstates_.position[i] <= 0.002)
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

  bool gripstatus = gripperAction_->waitForGoalToFinish(ros::Duration(3));
  if (gripstatus)
    std::cout << "Gripper State:" 
      << gripperAction_->getTerminalState().toString() << "\n";
  else
    std::cout << "Gripper unable to reach goal\n";

  hanoi::ArmStatus status;
  status.status = "closed";
  statusPublisher_.publish( status );
}

////////////////////////////////////////////////////////////////////////////////
// Open the gripper
void ArmController::OpenGripper()
{
  move_arm::ActuateGripperGoal g;
  g.data = 50.0;

  gripperAction_->sendGoal(g);

  bool gripstatus = gripperAction_->waitForGoalToFinish(ros::Duration(3));
  if (gripstatus)
    std::cout << "Gripper State:" << 
      gripperAction_->getTerminalState().toString() << "\n";
  else
    std::cout << "Gripper unable to reach goal\n";

  hanoi::ArmStatus status;
  status.status = "open";
  statusPublisher_.publish( status );
}

void ArmController::PrintJoints()
{
  const planning_models::KinematicModel::ModelInfo &mi = km_->getKinematicModel()->getModelInfo();

  for (unsigned int i = 0 ; i < names_.size(); ++i)
  {
    int idx = km_->getKinematicModel()->getJointIndex(names_[i]);
    std::cout << "  " << i << " = " << names_[i] << "  [" 
      << mi.stateBounds[idx * 2] << ", " << mi.stateBounds[idx * 2 + 1] << "]" 
      << std::endl;
  }
}

