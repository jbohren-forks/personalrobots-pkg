#include "../headers.h" 
#include <stdlib.h>

void testCrap(){

  vector <int> node_list;
  for(int i=1; i<5; i++){
    node_list.push_back(i);
  }

  printf("Size before clear: %d\n", node_list.size());
  node_list.clear();
  printf("Size after clear: %d\n", node_list.size());
}


void testPrecomputeActions(char* env, char* opt){
  printf("**********************************************\n");
  printf("Executing testPrecomputeActions...\n");
  EnvironmentNAV3DDYN environment_nav3Ddyn;
  int option = atoi(opt);
  if(option==1){
    environment_nav3Ddyn.EnableOptimizeFootprint();
  }else{
    environment_nav3Ddyn.DisableOptimizeFootprint();
  }


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

void testFootprintCostCalculation(char* env){

  EnvironmentNAV3DDYN environment_nav3Ddyn_opt;
  EnvironmentNAV3DDYN environment_nav3Ddyn_noopt;
  MDPConfig MDPCfg;

  environment_nav3Ddyn_opt.EnableOptimizeFootprint();
  environment_nav3Ddyn_noopt.DisableOptimizeFootprint();


  if(!environment_nav3Ddyn_opt.InitializeEnv(env) ||
     !environment_nav3Ddyn_noopt.InitializeEnv(env)){
    printf("ERROR: InitializeEnv failed\n");
    exit(1);
  }
  
  time_t current_time = time(NULL);
  tm *formatted_time = gmtime(&current_time);
  char filename_opt[64];
  char filename_noopt[64];
  sprintf(filename_opt, "logs/%d-%d-%d-%d-%d-%d_actions_opt.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  sprintf(filename_noopt, "logs/%d-%d-%d-%d-%d-%d_actions_noopt.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  environment_nav3Ddyn_opt.PrintActionsToFile(filename_opt);	
  environment_nav3Ddyn_noopt.PrintActionsToFile(filename_noopt);


  if(!environment_nav3Ddyn_opt.InitializeMDPCfg(&MDPCfg) ||
     !environment_nav3Ddyn_noopt.InitializeMDPCfg(&MDPCfg)){
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  vector<int> SuccIDV_opt;
  vector<int> CostV_opt;
  vector<int> SuccIDV_noopt;
  vector<int> CostV_noopt;

  environment_nav3Ddyn_opt.GetSuccs(MDPCfg.startstateid, &SuccIDV_opt, &CostV_opt);
  environment_nav3Ddyn_noopt.GetSuccs(MDPCfg.startstateid, &SuccIDV_noopt, &CostV_noopt);
  
  bool match = true;

  if(SuccIDV_opt.size() != SuccIDV_noopt.size()){
    match = false;
  }else{
    for(unsigned int i=0; i<SuccIDV_opt.size(); i++){
      if(SuccIDV_opt[i]!=SuccIDV_noopt[i] || CostV_opt[i] != CostV_noopt[i]){
	match = false;
	break;
      }
    }
  }

  if(!match){
    printf("Successors do not match.  See logs for error.\n");
    char filename2_opt[64];
    sprintf(filename2_opt, "logs/%d-%d-%d-%d-%d-%d_succs_opt.log", 
	    formatted_time->tm_year + 1900,
	    formatted_time->tm_mon + 1,
	    formatted_time->tm_mday,
	    formatted_time->tm_hour,
	    formatted_time->tm_min,
	    formatted_time->tm_sec);
    FILE* fp = fopen(filename2_opt, "w");
    fprintf(fp, "Print successors for state %d\n", MDPCfg.startstateid);
    for(int i=0; i<SuccIDV_opt.size(); i++){
      fprintf(fp, "Id: %d, Cost: %d\n", SuccIDV_opt[i], CostV_opt[i]);
    }
    
    fclose(fp);
    
    char filename2_noopt[64];
    sprintf(filename2_noopt, "logs/%d-%d-%d-%d-%d-%d_succs_noopt.log", 
	    formatted_time->tm_year + 1900,
	    formatted_time->tm_mon + 1,
	    formatted_time->tm_mday,
	    formatted_time->tm_hour,
	    formatted_time->tm_min,
	    formatted_time->tm_sec);
    fp = fopen(filename2_noopt, "w");
    fprintf(fp, "Print successors for state %d\n", MDPCfg.startstateid);
    for(int i=0; i<SuccIDV_noopt.size(); i++){
      fprintf(fp, "Id: %d, Cost: %d\n", SuccIDV_noopt[i], CostV_noopt[i]);
    }
    
    fclose(fp);
  }else{
    printf("Successors match! Test passed.\n");
  }

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

void testCalculateFootprintFromBoundaries(char* env){

  EnvironmentNAV3DDYN environment_nav3Ddyn;
  MDPConfig MDPCfg;

  if(!environment_nav3Ddyn.InitializeEnv(env)){
    printf("ERROR: InitializeEnv failed\n");
    exit(1);    
  }
  
  time_t current_time = time(NULL);
  tm *formatted_time = gmtime(&current_time);
  char filename[64];
  sprintf(filename, "logs/%d-%d-%d-%d-%d-%d_footprint.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);

  environment_nav3Ddyn.PrintActionsToFile(filename);

}

int testPlan(char* env, char* opt){
  
  int bRet = 0;
  double allocated_time_secs = 4000.0; //in seconds

  EnvironmentNAV3DDYN* environment_nav3Ddyn = new EnvironmentNAV3DDYN();
  int option = atoi(opt);
  if(option==1){
    environment_nav3Ddyn->EnableOptimizeFootprint();
  }else{
    environment_nav3Ddyn->DisableOptimizeFootprint();
  }
  environment_nav3Ddyn->SetObsThresh(254);
  MDPConfig MDPCfg;

  if(!environment_nav3Ddyn->InitializeEnv(env)){
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
  environment_nav3Ddyn->PrintActionsToFile(filename);	

  if(!environment_nav3Ddyn->InitializeMDPCfg(&MDPCfg)){
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  vector<int> solution_stateIDs_V;
  ARAPlanner* planner = new ARAPlanner(environment_nav3Ddyn, false);
  planner->set_initialsolution_eps(1.0);

  if(planner->set_start(MDPCfg.startstateid) == 0){
    printf("ERROR: failed to set start state\n");
    exit(1);
  }
  
  if(planner->set_goal(MDPCfg.goalstateid) == 0){
    printf("ERROR: failed to set goal state\n");
    exit(1);
  }
  
  printf("Start planning....\n");
  bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
  printf("Done planning\n");

  environment_nav3Ddyn->PrintTimeStat(stdout);

  //FILE* fSol = fopen("sol.txt", "w");
  for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
    environment_nav3Ddyn->PrintState(solution_stateIDs_V[i], true, NULL);
  }
  char solFilename[64];
  sprintf(solFilename, "logs/%d-%d-%d-%d-%d-%d_solution.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);

  vector<EnvNAV3DDYNContPoseWAction_t> path;
  vector<int> action_ids;
  environment_nav3Ddyn->GetContPathFromStateIds(solution_stateIDs_V, &path);
  FILE* fSol = fopen(solFilename, "w");

  for(unsigned int i=0; i<path.size(); i++){
    fprintf(fSol, "%d %d %f %f %f\n", path[i].ThetaId, path[i].ActionId, path[i].X, path[i].Y, path[i].Theta);
  }
  fclose(fSol);

  char filename2[64];
  sprintf(filename2, "logs/%d-%d-%d-%d-%d-%d_cfg.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  environment_nav3Ddyn->PrintConfigurationToFile(filename2);

  environment_nav3Ddyn->PrintCheckedCells(stdout);

  delete(planner);
  delete(environment_nav3Ddyn);
}


int testOptVsNoOptPlan(char* env){
  
  int bRet = 0;
  double allocated_time_secs = 100000.0; //in seconds

  EnvironmentNAV3DDYN environment_nav3Ddyn_noopt;
  environment_nav3Ddyn_noopt.DisableOptimizeFootprint();
  EnvironmentNAV3DDYN environment_nav3Ddyn_opt;
  environment_nav3Ddyn_opt.EnableOptimizeFootprint();
  
  MDPConfig MDPCfg;

  if(!environment_nav3Ddyn_opt.InitializeEnv(env) ||
     !environment_nav3Ddyn_noopt.InitializeEnv(env)){
    printf("ERROR: InitializeEnv failed\n");
    exit(1);
  }

  time_t current_time = time(NULL);
  tm *formatted_time = gmtime(&current_time);
  char filename_noopt[64];
  sprintf(filename_noopt, "logs/%d-%d-%d-%d-%d-%d_actions_noopt.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  environment_nav3Ddyn_noopt.PrintActionsToFile(filename_noopt);	

  char filename_opt[64];
  sprintf(filename_opt, "logs/%d-%d-%d-%d-%d-%d_actions_opt.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  environment_nav3Ddyn_opt.PrintActionsToFile(filename_opt);	

  if(!environment_nav3Ddyn_noopt.InitializeMDPCfg(&MDPCfg) ||
     !environment_nav3Ddyn_opt.InitializeMDPCfg(&MDPCfg)){
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  vector<int> solution_stateIDs_V_noopt;
  vector<int> solution_stateIDs_V_opt;
  ARAPlanner planner_noopt(&environment_nav3Ddyn_noopt, false);
  ARAPlanner planner_opt(&environment_nav3Ddyn_opt, false);

  if(planner_noopt.set_start(MDPCfg.startstateid) == 0 ||
     planner_opt.set_start(MDPCfg.startstateid) == 0){
    printf("ERROR: failed to set start state\n");
    exit(1);
  }
  
  if(planner_noopt.set_goal(MDPCfg.goalstateid) == 0 ||
     planner_opt.set_goal(MDPCfg.goalstateid) == 0){
    printf("ERROR: failed to set goal state\n");
    exit(1);
  }
  
  printf("Start planning opt....\n");
  bRet = planner_opt.replan(allocated_time_secs, &solution_stateIDs_V_opt);
  printf("Done planning opt\n");

  printf("Start planning noopt....\n");
  bRet = planner_noopt.replan(allocated_time_secs, &solution_stateIDs_V_noopt);

  environment_nav3Ddyn_opt.PrintTimeStat(stdout);
  environment_nav3Ddyn_noopt.PrintTimeStat(stdout);

  

  FILE* fSol = fopen("sol.txt", "w");
  fprintf(fSol, "Optimized solution:\n");
  for(unsigned int i=0; i<solution_stateIDs_V_opt.size(); i++){
    fprintf(fSol, "State: %d\n", solution_stateIDs_V_opt[i]);
  }
  fprintf(fSol, "\nUnoptimized solution:\n");
  for(unsigned int i=0; i<solution_stateIDs_V_noopt.size(); i++){
    fprintf(fSol, "State: %d\n", solution_stateIDs_V_noopt[i]);
  }
  /* for(unsigned int i = 0; i < solution_stateIDs_V_noopt.size(); i++) {
    environment_nav3Ddyn_noopt.PrintState(solution_stateIDs_V_noopt[i], true, NULL);
  }
  for(unsigned int i = 0; i < solution_stateIDs_V_opt.size(); i++) {
    environment_nav3Ddyn_opt.PrintState(solution_stateIDs_V_opt[i], true, NULL);
    }
  
  char solFilename_noopt[64];
  sprintf(solFilename_noopt, "logs/%d-%d-%d-%d-%d-%d_solution_noopt.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);

  vector<EnvNAV3DDYNContPoseWAction_t> path_nopt;
  vector<int> action_ids_noopt;
  environment_nav3Ddyn.GetContPathFromStateIds(solution_stateIDs_V, &path);
   FILE* fSol = fopen(solFilename, "w");

  for(unsigned int i=0; i<path.size(); i++){
    fprintf(fSol, "%d %d %f %f %f\n", path[i].ThetaId, path[i].ActionId, path[i].X, path[i].Y, path[i].Theta);
  }
  fclose(fSol);
  
  char filename2[64];
  sprintf(filename2, "logs/%d-%d-%d-%d-%d-%d_cfg.log", 
	  formatted_time->tm_year + 1900,
	  formatted_time->tm_mon + 1,
	  formatted_time->tm_mday,
	  formatted_time->tm_hour,
	  formatted_time->tm_min,
	  formatted_time->tm_sec);
  environment_nav3Ddyn.PrintConfigurationToFile(filename2);
  */
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

void testErase(){

  vector<int> int_list;
  
  for(int i=0; i<5; i++){
    int_list.push_back(i);
  }

  for(int i=0; i<5; i++){
    int_list.push_back(i);
  }

  printf("Size: %d\n", int_list.size());
  
  std::sort(int_list.begin(), int_list.end());
  vector<int>::iterator new_end;
  new_end = unique(int_list.begin(), int_list.end());
  int_list.erase(new_end, int_list.end());

  printf("New Size: %d\n", int_list.size());
  for(int i=0; i<int_list.size(); i++){
    printf("Int %d: %d\n", i, int_list[i]);
  }

}

int main(int argc, char* argv[]){



  if(argc < 3){
    printf("Usage: %s <cfg file> <optimize>\n", argv[0]);
    exit(1);
  }

  //  testCrap();

  // testFootprintCostCalculation(argv[1]);
  
  //  testPrecomputeActions(argv[1], argv[2]);
  //testErase();
  //testRemoveDuplicatesFromFootprint();
  //testOptVsNoOptPlan(argv[1]);
    testPlan(argv[1], argv[2]);

    //testCalculateFootprintFromBoundaries(argv[1]);

}
