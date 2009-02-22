#include <sbpl_arm_planner_node/sbpl_arm_planner_node.h>

using namespace std;
using namespace sbpl_arm_planner_node;

SBPLArmPlannerNode::SBPLArmPlannerNode(std::string node_name):ros::Node(node_name),node_name_(node_name)
{
  param ("~allocated_time", allocated_time_, 1.0);
  param ("~forward_search", forward_search_, true);
  param ("~search_mode", search_mode_, true);
  param ("~num_joints", num_joints_, 7);
  advertiseService(node_name + "/plan_path/GetPlan", &SBPLArmPlannerNode::planPath, this);
  subscribe(collision_map_topic_,collision_map_, &SBPLArmPlannerNode::collisionMapCallback,1);
};

SBPLArmPlannerNode::~SBPLArmPlannerNode()
{
  unsubscribe(collision_map_topic_);
  unadvertiseService(node_name + "/plan_path/GetPlan");
};

void PrintUsage(char *argv[])
{
    printf("USAGE: %s <cfg file>\n", argv[0]);
}

bool SBPLArmPlannerNode::initializePlanner()
{

  if(!env_pr2_arm_.InitializeEnv(filename_.c_str()))
  {
    printf("ERROR: InitializeEnv failed\n");
    return false;
  }

  if(!env_pr2_arm_.InitializeMDPCfg(&mdp_cfg_))  //Initialize MDP Info
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    return false;
  }

  planner_.init(&env_pr2_arm_, forward_search_);

  //set epsilon
  planner_.set_initialsolution_eps(env_pr2_arm_.GetEpsilon());

  //set search mode (true - settle with first solution)
  planner_.set_search_mode(search_mode_);
}

int SBPLArmPlannerNode::collisionMapCallback()
{
   int num_boxes = (int) collision_map_.boxes.size();
   double sbpl_boxes_[num_boxes][6];

   for(int i=0; i < num_boxes; i++)
   {
      sbpl_boxes[i][0] = collision_map_.boxes[i].center.x;
      sbpl_boxes[i][1] = collision_map_.boxes[i].center.y;
      sbpl_boxes[i][2] = collision_map_.boxes[i].center.z;

      sbpl_boxes[i][0] = collision_map_.boxes[i].extents.x;
      sbpl_boxes[i][1] = collision_map_.boxes[i].extents.y;
      sbpl_boxes[i][2] = collision_map_.boxes[i].extents.z;
   }
   pr2_arm_env_.AddObstaclesToEnv(sbpl_boxes,num_boxes);
}

int SBPLArmPlannerNode::replan(robot_msgs::JointTraj &arm_path)
{
  int b_ret(-1);

  clock_t start_time = clock();  //plan a path
  vector<int> solution_state_ids_v;

  ROS_INFO("Start planning.");
  b_ret = planner_.replan(allocated_time_, &solution_state_ids_v);

  ROS_INFO("Planning completed in %.4f seconds", double(clock()-start_time) / CLOCKS_PER_SEC);
  ROS_INFO("Size of solution = %d",solution_state_ids_v.size());

  if(b_ret)
  {
    arm_path.set_points_size(solution_state_ids_v.size());
    for(i = 0; i < solution_state_ids_v.size(); i++)
      arm_path.points[i].set_positions_size(num_joints);
    for(i = 0; i < solution_state_ids_v.size(); i++) 
    {
      environment_robarm.StateID2Angles(solution_state_ids_v[i], angles_r);
      for (unsigned int p = 0; p < num_joints_; p++)
        arm_path.points[i].positions[p] = angles_r[p];
      arm_path.points[i].time = 0.0;
    }
  }
  return b_ret;
}

int SBPLArmPlannerNode::setStart(const robot_msgs::JointTrajPoint &start)
{
  double sbpl_start[num_joints_];

  for(int i=0; i< num_joints_; i++)
  {
    sbpl_start[i] = start.positions[i];
  }

  env_pr2_arm_.SetStartJointConfig(sbpl_start, true);

  if(planner_.set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state\n");
    return false;
  }
}

bool SBPLArmPlannerNode::setGoal(const std_msgs::Pose &goal)
{
  double sbpl_goal[12];

  sbpl_goal[0] = goal.position.x;
  sbpl_goal[1] = goal.position.y;
  sbpl_goal[2] = goal.position.z;

  env_pr2_arm_.SetEndEffGoal(sbpl_goal, 3);

  if(planner_.set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state\n");
    return false;
  }

}

bool SBPLArmPlannerNode::planPath(SBPLAArmPlannerNode::PlanPathSrv::Request &req, SBPLArmPlannerNode::PlanPathSrv::Response &resp)
{
  robot_msgs::JointTraj traj; 
  if(setStart(req.start))
  {
     if(setGoal(req.cartesian_goal))
     {
        if(replan(traj))
        {
           ROS_INFO("Planning successful");
           resp.traj = traj;
           return true;
        }
        else
        {
           ROS_INFO("Planning unsuccessful");
        }
     }
     else
     {
        ROS_INFO("Set goal unsuccessful");
     }
  }
  else
  {
     ROS_INFO("Set start unsuccessful");
  }
  return false;
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv); 

  SBPLArmPlannerNode node("plan_path_node");

  try {
    node.spin();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }
  
  return(0);
}

