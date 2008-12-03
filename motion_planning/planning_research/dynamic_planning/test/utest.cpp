#include <fstream>
#include <iostream>
#include <string.h>
#include <gtest/gtest.h>

#include "../src/headers.h"

static const std::string PATH_PREFIX("test/");
static const std::string ENV_PATH_PREFIX("test/envs/");
static const std::string VALID_PATH_PREFIX("test/valid/");

void diffTest(const std::string& outputStr, const std::string& validOutputStr){

  cout << "Comparing " << outputStr << " with " <<  validOutputStr;

  std::ifstream validFile(validOutputStr.c_str());
  // If there is no valid file then generate one.
  if(!validFile.good()){
    FAIL() << "File " << validOutputStr << " does not exist for comparison." << endl;
  }
  else { // Verify output against the valid file.
    std::stringbuf newStrBuf, validStrBuf;
    std::ifstream fNew(outputStr.c_str()), fValid(validOutputStr.c_str());
    fNew >> &newStrBuf;
    fValid >> &validStrBuf;
    
    //ASSERT_EQ(newStrBuf.str() == validStrBuf.str(), true);
    ASSERT_EQ(newStrBuf.str(), validStrBuf.str());
  }
}

void testPrecomputeActions(const std::string& env){

  try{
    EnvironmentNAV3DDYN environment_nav3Ddyn;
    std::string envStr = ENV_PATH_PREFIX + env;

    //TODO: maybe prefix problem
    ASSERT_TRUE(environment_nav3Ddyn.InitializeEnv(envStr.c_str()));

    std::string actionOutputFile = PATH_PREFIX + env + ".out";
    
    // PrintActionsToFile takes a non-const string for some reason...
    char* tmpstr = strdup(actionOutputFile.c_str());
    environment_nav3Ddyn.PrintActionsToFile(tmpstr);
    free(tmpstr);

    std::string validActionFile = VALID_PATH_PREFIX + env + ".actions";

    FILE* fp2 = fopen(actionOutputFile.c_str(), "r");
    FILE* fp1 = fopen(validActionFile.c_str(), "r");

    //read the action file and ensure they are the same
    unsigned int tind, aind, pind, num_actions;
    unsigned int temp1, temp2;
    double dtemp1, dtemp2;
    int itemp1, itemp2;
    int a=1, b=2;

    for(tind = 0; tind < NAV3DDYN_THETADIRS; tind++){
      //tind
      fread(&temp1, sizeof(temp1), 1, fp1);
      fread(&temp2, sizeof(temp2), 1, fp2);

      ASSERT_EQ(temp1, temp2);

      //num_actiosn
      fread(&temp1, sizeof(temp1), 1, fp1);
      fread(&temp2, sizeof(temp2), 1, fp2);

      ASSERT_EQ(temp1, temp2);

      num_actions = temp1;
      
      for(aind = 0; aind < num_actions; aind++){
	//tv
	fread(&dtemp1, sizeof(dtemp1), 1, fp1);
	fread(&dtemp2, sizeof(dtemp2), 1, fp2);
	ASSERT_EQ(dtemp1, dtemp2);

	//rv
	fread(&dtemp1, sizeof(dtemp1), 1, fp1);
	fread(&dtemp2, sizeof(dtemp2), 1, fp2);
	ASSERT_EQ(dtemp1, dtemp2);

	//time
	fread(&itemp1, sizeof(itemp1), 1, fp1);
	fread(&itemp2, sizeof(itemp2), 1, fp2);
	ASSERT_EQ(itemp1, itemp2);

	//path length
	fread(&temp1, sizeof(temp1), 1, fp1);
	fread(&temp2, sizeof(temp2), 1, fp2);
	ASSERT_EQ(temp1, temp2);

	for(pind = 0; pind < temp1; pind++){
	  //x
	  fread(&itemp1, sizeof(itemp1), 1, fp1);
	  fread(&itemp2, sizeof(itemp2), 1, fp2);
	  ASSERT_EQ(itemp1, itemp2);

	  //y
	  fread(&itemp1, sizeof(itemp1), 1, fp1);
	  fread(&itemp2, sizeof(itemp2), 1, fp2);
	  ASSERT_EQ(itemp1, itemp2);
	  
	  //theta
	  fread(&itemp1, sizeof(itemp1), 1, fp1);
	  fread(&itemp2, sizeof(itemp2), 1, fp2);
	  ASSERT_EQ(itemp1, itemp2);

	}	  
	
	//path length
	fread(&temp1, sizeof(temp1), 1, fp1);
	fread(&temp2, sizeof(temp2), 1, fp2);
	ASSERT_EQ(temp1, temp2);

	for(pind = 0; pind < temp1; pind++){
	  //x
	  fread(&dtemp1, sizeof(dtemp1), 1, fp1);
	  fread(&dtemp2, sizeof(dtemp2), 1, fp2);
	  ASSERT_EQ(dtemp1, dtemp2);
	  
	  //y
	  fread(&dtemp1, sizeof(dtemp1), 1, fp1);
	  fread(&dtemp2, sizeof(dtemp2), 1, fp2);
	  ASSERT_EQ(dtemp1, dtemp2);
	  
	  //theta
	  fread(&dtemp1, sizeof(dtemp1), 1, fp1);
	  fread(&dtemp2, sizeof(dtemp2), 1, fp2);
	  ASSERT_EQ(dtemp1, dtemp2);
	  
	}

	//footprint size
	fread(&temp1, sizeof(temp1), 1, fp1);
	fread(&temp2, sizeof(temp2), 1, fp2);
	ASSERT_EQ(temp1, temp2);

	for(pind = 0; pind < temp1; pind++){
	  ASSERT_EQ(a, 1);
	  ASSERT_EQ(b,2);
	  
	  //x
	  fread(&dtemp1, sizeof(dtemp1), 1, fp1);
	  fread(&dtemp2, sizeof(dtemp2), 1, fp2);
	  ASSERT_EQ(dtemp1, dtemp2);
	  
	  //y
	  fread(&dtemp1, sizeof(dtemp1), 1, fp1);
	  fread(&dtemp2, sizeof(dtemp2), 1, fp2);
	  ASSERT_EQ(dtemp1, dtemp2);
	  
	}
      } 
    }
    

    fclose(fp1);
    fclose(fp2);
    
    //TODO: cleanup
  }catch(...){
    FAIL() << "Uncaught exception: " << "This is OK on OS X";
  }
}

	   


void testARAPlanner(const std::string& problem) {

   try{
     double allocated_time_secs = 1.0;
    MDPConfig MDPCfg;

    EnvironmentNAV3DDYN environment_nav3Ddyn;

    std::string problemStr = ENV_PATH_PREFIX + problem;
    ASSERT_TRUE(environment_nav3Ddyn.InitializeEnv(problemStr.c_str()));
    ASSERT_TRUE(environment_nav3Ddyn.InitializeMDPCfg(&MDPCfg));

    //plan a path
    vector<int> solution_stateIDs_V;
    ARAPlanner ara_planner(&environment_nav3Ddyn, false);
    ara_planner.set_search_mode(false); 
    ASSERT_TRUE(ara_planner.set_start(MDPCfg.startstateid));
    ASSERT_TRUE(ara_planner.set_goal(MDPCfg.goalstateid));
    ASSERT_TRUE(ara_planner.replan(allocated_time_secs, &solution_stateIDs_V));
    environment_nav3Ddyn.PrintTimeStat(stdout);
    

    //output the path
    std::string outputStr(PATH_PREFIX + problem + ".out");
    vector<EnvNAV3DDYNContPose_t> path;
    environment_nav3Ddyn.GetContPathFromStateIds(solution_stateIDs_V, &path); 
    
    FILE* fSol = fopen(outputStr.c_str(), "w");
    for(unsigned int i=0; i < path.size(); i++){
      fprintf(fSol, "%f %f %f\n", path[i].X, path[i].Y, path[i].Theta);
    }

    fclose(fSol);

    std::string validOutputStr(VALID_PATH_PREFIX + problem + ".solution");
    diffTest(outputStr, validOutputStr);

  }catch(...){
    FAIL() << "Uncaught exception: " << "This is OK on OS X";
   }
}

void testManualConfigurationARAPlanner(){
  
  try{
    double allocated_time_secs = 1.0;
    MDPConfig MDPCfg;

    EnvironmentNAV3DDYN environment_nav3Ddyn;

    int width = 15;
    int height = 15;
    int startx = 1;
    int starty = 1;
    int starttheta = 0;
    int goalx = 1;
    int goaly = 6;
    int goaltheta = 0;

    vector<sbpl_2Dpt_t> perimeterptsV;
    sbpl_2Dpt_t pt;
    pt.x = -0.5;
    pt.y = -0.5;
    perimeterptsV.push_back(pt);
    pt.x = -0.5;
    pt.y = 0.5;
    perimeterptsV.push_back(pt);
    pt.x = 0.5;
    pt.y = 0.5;
    perimeterptsV.push_back(pt);
    pt.x = 0.5;
    pt.y = -0.5;
    perimeterptsV.push_back(pt);

    double cellsize_m = 0.5;
    double nominalvel_mpersecs = 1.0;
    double timetoturn45degsinplace_secs = 2.0;

    const unsigned char mapdata[15*15] = {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
						 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}; 
    
      

    ASSERT_TRUE(environment_nav3Ddyn.InitializeEnv( width, height,
						    mapdata,
						    startx, starty, starttheta,
						    goalx, goaly, goaltheta,
						    0, 0, 0, //what are these?
						    perimeterptsV, 
						    cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs));
    ASSERT_TRUE(environment_nav3Ddyn.InitializeMDPCfg(&MDPCfg));
    
    vector<int> solution_stateIDs_V;
    ARAPlanner ara_planner(&environment_nav3Ddyn, false);
    ara_planner.set_search_mode(false);
    ASSERT_TRUE(ara_planner.set_start(MDPCfg.startstateid));
    ASSERT_TRUE(ara_planner.set_goal(MDPCfg.goalstateid));
    ASSERT_TRUE(ara_planner.replan(allocated_time_secs, &solution_stateIDs_V));
    environment_nav3Ddyn.PrintTimeStat(stdout);
    
    //output the path
    std::string outputStr = PATH_PREFIX + "testEnv1.cfg.out";
    vector<EnvNAV3DDYNContPose_t> path;
    environment_nav3Ddyn.GetContPathFromStateIds(solution_stateIDs_V, &path); 
    
    FILE* fSol = fopen(outputStr.c_str(), "w");
    for(unsigned int i=0; i < path.size(); i++){
      fprintf(fSol, "%f %f %f\n", path[i].X, path[i].Y, path[i].Theta);
    }

    fclose(fSol);

    std::string validOutputStr(VALID_PATH_PREFIX + "testEnv1.cfg.solution");
    diffTest(outputStr, validOutputStr);

  }catch(...){
    FAIL() << "Uncaught exception: " << "This is OK on OS X";
  }
}

void testRemoveDuplicatesFromFootprint(){

}


//test planner
TEST(DynamicPlanning, PrecomputeActions){
  testPrecomputeActions("testEnv1.cfg");
}

TEST(DynamicPlanning, ARAPlanner){
  testARAPlanner("testEnv1.cfg");
}

TEST(DynamicPlanning, RemoveDuplicatesFromFootprint){
  testRemoveDuplicatesFromFootprint();
}

TEST(DynamicPlanning, ManualConfiguration) {
  testManualConfigurationARAPlanner();
}

int main(int argc, char* argv[]){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
