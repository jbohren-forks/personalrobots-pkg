#include <ros/node.h>
#include <std_msgs/PoseStamped.h>
#include <robarm3d/PlanPathSrv.h>

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

#include <pr2_msgs/GraspPoint.h>
#include <pr2_mechanism_controllers/GraspPointSrv.h>
 
#include <tf/tf.h>

static int done = 0;
static const int num_joints = 7;

static const double GRIPPER_OPEN = 0.6;
static const double GRIPPER_CLOSE = 0.1;

void finalize(int donecare)
{
  done = 1;
}

void actuateGripper(int open)
{
  pr2_mechanism_controllers::TrajectoryStart::request  req_gripper_traj_start;
  pr2_mechanism_controllers::TrajectoryStart::response res_gripper_traj_start;

  pr2_mechanism_controllers::TrajectoryQuery::request  req_gripper_traj_query;
  pr2_mechanism_controllers::TrajectoryQuery::response res_gripper_traj_query;

  req_gripper_traj_start.traj.set_points_size(1);
  req_gripper_traj_start.traj.points[0].set_positions_size(1);

  if(open)
    req_gripper_traj_start.traj.points[0].positions[0] = GRIPPER_OPEN;
  else
    req_gripper_traj_start.traj.points[0].positions[0] = GRIPPER_CLOSE;

  if (ros::service::call("gripper_trajectory_controller/TrajectoryStart", req_gripper_traj_start, res_gripper_traj_start))
  {
    ROS_INFO("Actuated gripper");
  }

  int done = -1;
  req_gripper_traj_query.trajectoryid = res_gripper_traj_start.trajectoryid;
  while(!(done == res_gripper_traj_query.State_Done || done == res_gripper_traj_query.State_Failed))
  {
    if(ros::service::call("gripper_trajectory_controller/TrajectoryQuery", req_gripper_traj_query, res_gripper_traj_query))  
    {
      done = res_gripper_traj_query.done;
    }
    else
    {
      ROS_ERROR("Trajectory query failed");
    }
  } 
}

void goHome()
{
  double go_home[7] = {0,0.2,0.0,-1.25,0,0,0};

  pr2_mechanism_controllers::TrajectoryStart::request  go_home_traj_start_req;
  pr2_mechanism_controllers::TrajectoryStart::response go_home_traj_start_res;

  pr2_mechanism_controllers::TrajectoryQuery::request  go_home_traj_query_req;
  pr2_mechanism_controllers::TrajectoryQuery::response go_home_traj_query_res;

  go_home_traj_start_req.traj.set_points_size(1);
  go_home_traj_start_req.traj.points[0].set_positions_size(num_joints);

  for(int i=0; i<num_joints; i++)
  {
    go_home_traj_start_req.traj.points[0].positions[i] = go_home[i];
  }

  if (ros::service::call("right_arm_trajectory_controller/TrajectoryStart", go_home_traj_start_req, go_home_traj_start_res))
  {
    ROS_INFO("Done");
  }
  go_home_traj_query_req.trajectoryid =  go_home_traj_start_res.trajectoryid;
  while(!(done == go_home_traj_query_res.State_Done || done == go_home_traj_query_res.State_Failed))
  {
    if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery",  go_home_traj_query_req,  go_home_traj_query_res))  
    {
      done =  go_home_traj_query_res.done;
    }
    else
    {
      ROS_ERROR("Trajectory query failed");
    }
  } 
}


void sendTrajectory(const pr2_mechanism_controllers::JointTraj &traj)
{
  pr2_mechanism_controllers::TrajectoryStart::request  send_traj_start_req;
  pr2_mechanism_controllers::TrajectoryStart::response send_traj_start_res;

  pr2_mechanism_controllers::TrajectoryQuery::request  send_traj_query_req;
  pr2_mechanism_controllers::TrajectoryQuery::response send_traj_query_res;

  send_traj_start_req.traj = traj;
  int traj_done = -1;
  if (ros::service::call("right_arm_trajectory_controller/TrajectoryStart", send_traj_start_req, send_traj_start_res))
  {
    ROS_INFO("Done");
  }
  send_traj_query_req.trajectoryid =  send_traj_start_res.trajectoryid;
  while(!(traj_done == send_traj_query_res.State_Done || traj_done == send_traj_query_res.State_Failed))
  {
    if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery",  send_traj_query_req,  send_traj_query_res))  
    {
      traj_done =  send_traj_query_res.done;
    }
    else
    {
      ROS_ERROR("Trajectory query failed");
    }
  } 
}

void getGraspTrajectory(const std_msgs::PoseStamped &transform, pr2_mechanism_controllers::JointTraj &traj)
{
  pr2_mechanism_controllers::GraspPointSrv::request  req;
  pr2_mechanism_controllers::GraspPointSrv::response res;

  req.transform = transform;

  req.transform.header.stamp = ros::Time::now();
  req.transform.header.frame_id = "r_shoulder_pan_link";

  if (ros::service::call("/grasp_point_node/SetGraspPoint", req, res))
  {
    ROS_INFO("Done");
  }
  else
  {
    ROS_ERROR("No grasp trajectory returned");
    exit(-1);
  }
  traj = res.traj;

  return;
}


void getGoalTransform(double roll, double pitch, double yaw, double x, double y, double z, std_msgs::PoseStamped &transform)
{
      tf::Quaternion quat_trans = tf::Quaternion(yaw,pitch,roll);

      transform.pose.orientation.x = quat_trans.x();
      transform.pose.orientation.y = quat_trans.y();
      transform.pose.orientation.z = quat_trans.z();
      transform.pose.orientation.w = quat_trans.w();

      transform.pose.position.x = x;
      transform.pose.position.y = y;
      transform.pose.position.z = z;
      return;
}


int main(int argc, char *argv[])
{

//  int num_joints = 7;

  if(argc < 4)
  {
    printf("Usage:: test_plan_path x y z\n");
    return -1;
  }

/*********** Initialize ROS  ****************/
  ros::init(argc,argv);
  ros::node *node = new ros::node("test_arm_trajectory_controller"); 

  signal(SIGINT,  finalize);
  signal(SIGQUIT, finalize);
  signal(SIGTERM, finalize);

  robarm3d::PlanPathSrv::request  req_plan_path;
  robarm3d::PlanPathSrv::response res_plan_path;
  req_plan_path.start.set_positions_size(num_joints);

  pr2_mechanism_controllers::TrajectoryStart::request  req_traj_start;
  pr2_mechanism_controllers::TrajectoryStart::response res_traj_start;

  pr2_mechanism_controllers::TrajectoryQuery::request  req_traj_query;
  pr2_mechanism_controllers::TrajectoryQuery::response res_traj_query;

  double grasp_standoff_distance = 0.2;

  req_traj_query.trajectoryid = 0;

  ROS_INFO("Going home");

  goHome();
  actuateGripper(1);

  ROS_INFO("Home");

  if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery", req_traj_query, res_traj_query))  
  {
    for(int i=0; i<num_joints; i++)
    {
      req_plan_path.start.positions[i] = res_traj_query.jointpositions[i];
      ROS_INFO("%d %f",i,res_traj_query.jointpositions[i]);
    }
  }
  else
  {
    ROS_ERROR("Could not get initial joint angles");
    ros::fini();
    exit(-1);
  }

  std_msgs::PoseStamped first_grasp_point;
  std_msgs::PoseStamped intermediate_point;

  double roll = 0.0;
  double pitch = M_PI/2;
  double yaw = 0;

  double x = 0.75;
  double y = -0.3;
  double z = 0.0;

  getGoalTransform(roll,pitch,yaw,x,y,z,first_grasp_point);
  intermediate_point = first_grasp_point;
  intermediate_point.pose.position.z = first_grasp_point.pose.position.z + grasp_standoff_distance + 1.0;

  pr2_mechanism_controllers::JointTraj firstTraj;
  getGraspTrajectory(first_grasp_point, firstTraj);

  req_plan_path.goal.x = intermediate_point.pose.position.x;
  req_plan_path.goal.y = intermediate_point.pose.position.y;
  req_plan_path.goal.z = intermediate_point.pose.position.z;

  if(ros::service::call("plan_path_node/GetPlan",req_plan_path,res_plan_path))
  {
/*    for(int i=0; i < (int) res_plan_path.traj.get_points_size(); i++)
    {
      for(int j=0; j < (int) res_plan_path.traj.points[0].get_positions_size(); j++)
      {
        printf("%f",res_plan_path.traj.points[i].positions[j]);
      }
      printf("\n");
      }*/
    req_traj_start.traj = res_plan_path.traj;    
  }
  else
  {
    ROS_ERROR("Could not get path");
    ros::fini();    
    exit(-1);
  }

  ROS_INFO("Sending grasp trajectory");
  sendTrajectory(res_plan_path.traj);

  ROS_INFO("Completing grasp trajectory");
  firstTraj.points[1].time = 4.0;
  sendTrajectory(firstTraj);

  sleep(2);

  actuateGripper(0);

  intermediate_point.pose.position.y = 0.3;

  first_grasp_point.pose.position.y = -first_grasp_point.pose.position.y;

  req_plan_path.goal.x = first_grasp_point.pose.position.x;
  req_plan_path.goal.y = first_grasp_point.pose.position.y;
  req_plan_path.goal.z = first_grasp_point.pose.position.z + 1.0;

  if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery", req_traj_query, res_traj_query))  
  {
    for(int i=0; i<num_joints; i++)
    {
      req_plan_path.start.positions[i] = res_traj_query.jointpositions[i];
      ROS_INFO("%d %f",i,res_traj_query.jointpositions[i]);
    }
  }
  else
  {
    ROS_ERROR("Could not get initial joint angles");
    ros::fini();
    exit(-1);
  }

  pr2_mechanism_controllers::JointTraj secondTraj;
  getGraspTrajectory(first_grasp_point, secondTraj);

  if(ros::service::call("plan_path_node/GetPlan",req_plan_path,res_plan_path))
  {
    req_traj_start.traj = res_plan_path.traj;    
  }
  else
  {
    ROS_ERROR("Could not get path");
    ros::fini();    
    exit(-1);
  }

  ROS_INFO("Sending grasp trajectory");
  sendTrajectory(res_plan_path.traj);

  ROS_INFO("Ready to drop object");
  secondTraj.points[1].time = 4.0;
  sendTrajectory(secondTraj);

  actuateGripper(1);

  ros::fini();
  return 1;
}
