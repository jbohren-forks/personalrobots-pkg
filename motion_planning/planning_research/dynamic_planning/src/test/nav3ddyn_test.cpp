#include "../headers.h" 




void testPrecomputeActions(char* env){
  printf("**********************************************\n");
  printf("Executing testPrecomputeActions...\n");
  EnvironmentNAV3DDYN environment_nav3Ddyn;
  
  assert(environment_nav3Ddyn.InitializeEnv(env));
  time_t current_time = time(NULL);
  tm *formatted_time = gmtime(&current_time);
  char filename[64];
  sprintf(filename, "logs/%d-%d-%d-%d-%d-%d_actions.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  environment_nav3Ddyn.PrintActionsToFile(filename);					  
  printf("Done.\n");
  printf("**********************************************\n");
}

void testCalculateFootprintForPose(){


}

void testRemoveDuplicatesFromFootprint(){

  EnvironmentNAV3DDYN environment_nav3Ddyn;
  printf("**********************************************\n");
  printf("Executing testRemoveDuplicatesFromFootprint....\n");
  vector<EnvNAV3DDYN2Dpt_t> testFootprint;
  EnvNAV3DDYN2Dpt_t pt1;
  pt1.x = 1; 
  pt1.y = 1;

  EnvNAV3DDYN2Dpt_t pt2;
  pt2.x = 1; 
  pt2.y = 2;

  EnvNAV3DDYN2Dpt_t pt3;
  pt3.x = 2; 
  pt3.y = 1;

  EnvNAV3DDYN2Dpt_t pt4;
  pt4.x = 3; 
  pt4.y = 3;

  EnvNAV3DDYN2Dpt_t pt5;
  pt5.x = 2; 
  pt5.y = 3;

  EnvNAV3DDYN2Dpt_t pt6;
  pt6.x = 1; 
  pt6.y = 1;

  testFootprint.push_back(pt1);
  testFootprint.push_back(pt2);  
  testFootprint.push_back(pt3);  
  testFootprint.push_back(pt4);
  testFootprint.push_back(pt5);

  environment_nav3Ddyn.RemoveDuplicatesFromFootprint(&testFootprint);

  assert(testFootprint.size() == 5);

  testFootprint.push_back(pt6);
  assert(testFootprint.size() == 6);

  environment_nav3Ddyn.RemoveDuplicatesFromFootprint(&testFootprint);
  assert(testFootprint.size() == 5);
  
  printf("Final footprint: \n");
  for(unsigned int i=0; i<testFootprint.size(); i++){
    printf("\t%f %f\n", testFootprint.at(i).x, testFootprint.at(i).y);
  }

  printf("PASSED\n");
  printf("**********************************************\n");

}

void testCalculateFootprintFromBoundaries(){


}



int testPlan(char* env){
  
  int bRet = 0;
  double allocated_time_secs = 1.0; //in seconds

  EnvironmentNAV3DDYN environment_nav3Ddyn;
  MDPConfig MDPCfg;

  if(!environment_nav3Ddyn.InitializeEnv(env)){
    printf("ERROR: InitializeEnv failed\n");
    exit(1);
  }

  time_t current_time = time(NULL);
  tm *formatted_time = gmtime(&current_time);
  char filename[64];
  sprintf(filename, "logs/%d-%d-%d-%d-%d-%d_actions.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  environment_nav3Ddyn.PrintActionsToFile(filename);	

  if(!environment_nav3Ddyn.InitializeMDPCfg(&MDPCfg)){
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  vector<int> solution_stateIDs_V;
  ARAPlanner planner(&environment_nav3Ddyn, false);

  if(planner.set_start(MDPCfg.startstateid) == 0){
    printf("ERROR: failed to set start state\n");
    exit(1);
  }
  
  if(planner.set_goal(MDPCfg.goalstateid) == 0){
    printf("ERROR: failed to set goal state\n");
    exit(1);
  }
  
  printf("Start planning....\n");
  bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
  printf("Done planning\n");

  environment_nav3Ddyn.PrintTimeStat(stdout);

  //FILE* fSol = fopen("sol.txt", "w");
  for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
    environment_nav3Ddyn.PrintState(solution_stateIDs_V[i], true, NULL);
  }
  char solFilename[64];
  sprintf(solFilename, "logs/%d-%d-%d-%d-%d-%d_solution.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);

  vector<EnvNAV3DDYNContPose_t> path;
  environment_nav3Ddyn.GetContPathFromStateIds(solution_stateIDs_V, &path);
  FILE* fSol = fopen(solFilename, "w");

  for(unsigned int i=0; i<path.size(); i++){
    fprintf(fSol, "%f %f %f\n", path[i].X, path[i].Y, path[i].Theta);
  }
  fclose(fSol);
  
}

/*
int plan_and_navigate(){
  double allocated_time_secs_foreachplan = 0.5;

  EnvironmentNAV3DDYN environment_nav3Ddyn;
  MDPConfig MDPCfg;

  int size_x = -1;
  int size_y = -1;

  int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
  int x,y;

  vector<int> preds_of_changededgesIDV;
  vector<nav2dcell_t> changedcellsV;

  nav2dcell_t nav2dcell;

  if(!environment_nav3Ddyn.InitializeEnv()){
    printf("ERROR: InitializeEnv failed\n");
    exit(1);
  }

  if(!environment_nav3Ddyn.InitializeMDPCfg(&MDPCfg)){
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  ARAPlanner planner(&environment_nav3Ddyn);

  if(planner.set_start(MDPCfg.startstateid) == 0){
    printf("ERROR: failed to set start state\n");
    exit(1);
  }
  
  if(planner.set_goal(MDPCfg.goalstateid) == 0){
    printf("ERROR: failed to set goal state\n");
    exit(1);
  }

  
  //main loop
  int goalx_c = CONTXY2DISC(goalx, cellsize_m);
  int goaly_c = CONTXY2DISC(goaly, cellsize_m);
  int goaltheta_c = ContTheta2Disc(goaltheta, NAV3DDYN_THETADIRS);

  while(fabs(startx - goalx) > goaltol_x || 
	fabs(starty - goaly) > goaltol_y ||
	fabs(starttheta - goaltheta) > goaltol_theta){
    
    bool bChanges = false;
    preds_of_changededgesIDV.clear();
    changedcellsV.clear();

    for(i=0; i<8; i++){
      int x = CONTXY2DISC(startx, cellsize_m) + dx[i];
      int y = CONTXY2DISC(starty, cellsize_m) + dy[i];
      
      // if(x<0 || x>=size_x || y<0 || y>=size_y) continue;
      int index = x + y*size_x;
      if(map[index] != 1 && trueenvironment_nav3Ddyn.IsObstacle(x, y)){
	map[index] = 1;

	environment_nav3Ddyn.UpdateCost(x,y,map[index]);
	printf("setting cost [%d][%d] to %d\n", x, y, map[index]);
	bChanges = true;

	nav2dcell.x = x;
	nav2dcell.y = y;
	changedcellsV.push_back(nav2dcell);
      }

    }
    
    if(bChanges){
      planner.costs_changed();
      
      int startx_c = CONTXY2DISC(startx, cellsize_m);
      int starty_c = CONTXY2DISC(starty, cellsize_m);
      int starttheta_c = ContTheta2Disc(starttheta, NAV3DDYN_THETADIRS);

      printf("Current state: %d %d %d\n", startx_c, starty_c, starttheta_c);

      //plan
      bool bPlanExists = false;
      while(bPlanExists == false){
	printf("Replanning...\n");
	bPlanExists = (planner.replan(allocated_time_secs_foreachplan, 
				      &solution_stateIDs_V) == 1);

	printf("done with the solution of size=%d\n", solution_state_IDs_V.size());
	
	environment_nav3Ddyn.PrintTimeState(stdout);
      }

      

    }
  }

       
  

  }*/

int main(int argc, char* argv[]){



  if(argc < 2){
    printf("Usage: %s <cfg file>\n", argv[0]);
    exit(1);
  }

  //  testPrecomputeActions(argv[1]);
  
  //testRemoveDuplicatesFromFootprint();

  testPlan(argv[1]);

}
