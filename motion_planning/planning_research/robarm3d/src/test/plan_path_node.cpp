#include <robarm3d/plan_path_node.h>

using namespace std;
using namespace plan_path_node;

PlanPathNode::PlanPathNode(std::string node_name):ros::Node(node_name)
{
};

void PlanPathNode::init()
{
  this->advertiseService("plan_path_node/GetPlan", &PlanPathNode::planPath, this);
}

PlanPathNode::~PlanPathNode()
{
  this->unadvertiseService("plan_path_node/GetPlan");
};

void PrintUsage(char *argv[])
{
    printf("USAGE: %s <cfg file>\n", argv[0]);
}

int PlanPathNode::planrobarmROS(const pr2_mechanism_controllers::JointTrajPoint &start, const std_msgs::Point &goal, pr2_mechanism_controllers::JointTraj &armpath)
{
  double start_pos[NUMOFLINKS];
  double goal_pos[3];

  for(int i=0; i<NUMOFLINKS; i++)
  {
    start_pos[i] = start.positions[i];
  }

  goal_pos[0] = goal.x;
  goal_pos[1] = goal.y;
  goal_pos[2] = goal.z;

  int bRet = 0;
  double allocated_time_secs = MAX_RUNTIME; //in seconds
  unsigned int i = 0;
  MDPConfig MDPCfg;

  //Initialize Environment (should be called before initializing anything else)
  EnvironmentROBARM environment_robarm;

//NOTE: If you want to set a goal from here, you MUST do it before you run InitializeEnv
//     environment_robarm.SetEndEffGoal(array, length of array(either 3 or 7));
  environment_robarm.SetEndEffGoal(goal_pos, 3);

  environment_robarm.SetStartAngles(start_pos, 1);

  if(!environment_robarm.InitializeEnv(filename_.c_str()))
  {
    printf("ERROR: InitializeEnv failed\n");
    exit(1);
  }

  //Initialize MDP Info
  if(!environment_robarm.InitializeMDPCfg(&MDPCfg))
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  //plan a path
  clock_t starttime = clock();

  vector<int> solution_stateIDs_V;
  bool bforwardsearch = true;
  ARAPlanner planner(&environment_robarm, bforwardsearch);

  if(planner.set_start(MDPCfg.startstateid) == 0)
  {
    printf("ERROR: failed to set start state\n");
    exit(1);
  }

  if(planner.set_goal(MDPCfg.goalstateid) == 0)
  {
    printf("ERROR: failed to set goal state\n");
    exit(1);
  }

  //set epsilon
  planner.set_initialsolution_eps(environment_robarm.GetEpsilon());

  //set search mode (true - settle with first solution)
  planner.set_search_mode(true);

  printf("start planning...\n");
  bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);

  printf("completed in %.4f seconds.\n", double(clock()-starttime) / CLOCKS_PER_SEC);

  printf("done planning\n");
  std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

#if !USE_DH
//  environment_robarm.CloseKinNode();
#endif

#if VERBOSE
  printf("printing statistics");
//  environment_robarm.OutputPlanningStats();
#endif

/*    // create filename with current time
      string outputfile = "sol";
      outputfile.append(".txt");

      FILE* fSol = fopen(outputfile.c_str(), "w");
      for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
      environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);
      }
      fclose(fSol);
*/  
  //create a ROS JointTraj message for path that was generated
  double angles_r[NUMOFLINKS];
//   pr2_mechanism_controllers::JointTraj armpath;
   printf("ready to fill trajectory");

   if(bRet)
  {
    armpath.set_points_size(solution_stateIDs_V.size());

   printf("filling trajectory");

    for(i = 0; i < solution_stateIDs_V.size(); i++)
      armpath.points[i].set_positions_size(NUMOFLINKS);

    for(i = 0; i < solution_stateIDs_V.size(); i++) 
    {
      environment_robarm.StateID2Angles(solution_stateIDs_V[i], angles_r);

      for (unsigned int p = 0; p < 7; p++)
        armpath.points[i].positions[p] = angles_r[p];

      armpath.points[i].time = 0.0;
    }
  }
  //send out the message....
   printf("Returning from function call");
  return bRet;
}


bool PlanPathNode::planPath(robarm3d::PlanPathSrv::request &req, robarm3d::PlanPathSrv::response &resp)
{
  pr2_mechanism_controllers::JointTraj traj; 
  int bRet = planrobarmROS(req.start,req.goal,traj);
  if(bRet)
  {
   resp.traj = traj;
    return true;
  }
  else
    return false;
}

using namespace plan_path_node;

int main(int argc, char *argv[])
{
  if(argc != 2)
  {
    PrintUsage(argv);
    exit(1);
  }

  ros::init(argc, argv);
  PlanPathNode node("plan_path_node");
  node.init();
  node.filename_ = std::string(argv[1]);

  try {
    node.spin();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }
  ros::fini();

  return 0;
}

