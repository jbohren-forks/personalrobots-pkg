#include <ros/node.h>
#include <robarm3d/PlanPathSrv.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>


static int done = 0;

static const double GRIPPER_OPEN = 0.5;
static const double GRIPPER_CLOSE = 0.0;


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


int main(int argc, char *argv[])
{

  int num_joints = 7;

/*********** Initialize ROS  ****************/
  ros::init(argc,argv);
  ros::Node *node = new ros::Node("test_arm_trajectory_controller"); 

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

  req_traj_query.trajectoryid = 0;

  if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery", req_traj_query, res_traj_query))  
  {
    for(int i=0; i<num_joints; i++)
    {
      req_plan_path.start.positions[i] = res_traj_query.jointpositions[i];
    }
  }
  else
  {
    ROS_ERROR("Could not get initial joint angles");
    ros::fini();
    exit(-1);
  }

  req_plan_path.goal.x = 0.75;
  req_plan_path.goal.y = 0.0;
  req_plan_path.goal.z = 1.0;

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

  if (ros::service::call("right_arm_trajectory_controller/TrajectoryStart", req_traj_start, res_traj_start))
  {
    ROS_INFO("Done");
  }
  else
  {
    ROS_ERROR("Trajectory not executed");
  }

  ros::fini();
  return 1;
}


