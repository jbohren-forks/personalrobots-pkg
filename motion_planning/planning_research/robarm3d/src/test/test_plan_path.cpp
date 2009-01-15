#include <ros/node.h>
#include <robarm3d/PlanPathSrv.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>


static int done = 0;

void finalize(int donecare)
{
  done = 1;
}

int main(int argc, char *argv[])
{

  int num_joints = 7;

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
  req_plan_path.goal.z = 0.0;

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
