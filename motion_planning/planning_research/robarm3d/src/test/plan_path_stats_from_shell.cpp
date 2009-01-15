#include <iostream>
#include "../headers.h"

#define VERBOSE 1
#define MAX_RUNTIME 180.0

void PrintUsage(char *argv[])
{
    printf("USAGE: %s <cfg file>\n", argv[0]);
}

int planrobarm(int argc, char *argv[])
{
    int bRet = 0;
    double allocated_time_secs = MAX_RUNTIME; //in seconds
    MDPConfig MDPCfg;

    //Initialize Environment (should be called before initializing anything else)
    EnvironmentROBARM environment_robarm;

//NOTE: If you want to set a goal from here, you MUST do it before you run InitializeEnv
//     environment_robarm.SetEndEffGoal(array, length of array(either 3 or 7));

    if(!environment_robarm.InitializeEnvForStats(argv[1],atoi(argv[2])))
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
//     planner.set_search_mode(true);

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);

    printf("completed in %.4f seconds.\n", double(clock()-starttime) / CLOCKS_PER_SEC);

    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    // create filename with current time
    string outputfile = "sol";
    outputfile.append(".txt");

    FILE* fSol = fopen(outputfile.c_str(), "w");
    for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);
    }
    fclose(fSol);

#if !USE_DH
    environment_robarm.CloseKinNode();
#endif

#if VERBOSE
    environment_robarm.OutputPlanningStats();
#endif

    return bRet;
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        PrintUsage(argv);
        exit(1);
    }

    //robotarm planning
    planrobarm(argc, argv);

    return 0;
}

