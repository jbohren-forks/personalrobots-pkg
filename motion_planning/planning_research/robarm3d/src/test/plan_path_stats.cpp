#include <iostream>
#include "../headers.h"

#define VERBOSE 1
#define MAX_RUNTIME 300.0
#define NUM_RUNS 100

void PrintUsage(char *argv[])
{
    printf("USAGE: %s <cfg file>\n", argv[0]);
}

int planrobarm(int argc, char *argv[], int cntr)
{
    int bRet = 0;
    double allocated_time_secs = MAX_RUNTIME; //in seconds
    MDPConfig MDPCfg;

    //Initialize Environment (should be called before initializing anything else)
    EnvironmentROBARM environment_robarm;

    if(!environment_robarm.InitializeEnvForStats(argv[1],cntr))
    {
        printf("ERROR: InitializeEnv failed\n");
//         exit(1);
        return 0;
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
//         return;
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
/*
    // create filename with current time
    string outputfile = "sol";
    outputfile.append(".txt");

    FILE* fSol = fopen(outputfile.c_str(), "w");
    for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);
    }
    fclose(fSol);
*/
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
    if(argc != 2)
    {
        PrintUsage(argv);
        exit(1);
    }

    for (int i = 0; i < NUM_RUNS; i++)
    {
        printf("---------------\nRUN #%i\n---------------\n", i);
        //robotarm planning
        planrobarm(argc, argv,i);
    }

    return 0;
}

