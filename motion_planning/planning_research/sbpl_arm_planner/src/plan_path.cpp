#include <iostream>
#include <sbpl_arm_planner/headers.h>

#define VERBOSE 1
#define MAX_RUNTIME 60.0

void PrintUsage(char *argv[])
{
    printf("USAGE: %s <cfg file>\n", argv[0]);
}

int planrobarm(int argc, char *argv[])
{
    int bRet = 0;
    double allocated_time_secs = MAX_RUNTIME; //in seconds
    MDPConfig MDPCfg;

    clock_t totaltime = clock();

    //Initialize Environment (should be called before initializing anything else)
    EnvironmentROBARM3D environment_robarm;

    if(!environment_robarm.InitializeEnv(argv[1]))
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

//     vector <vector <double> > obstacles(3,vector<double>(6));

//     obstacles[0][0] = 1.8;
//     obstacles[0][1] = -.3;
//     obstacles[0][2] = -.7;
//     obstacles[0][3] = .1;
//     obstacles[0][4] = .1;
//     obstacles[0][5] = .1;
// 
//     obstacles[1][0] = .5;
//     obstacles[1][1] = -.3;
//     obstacles[1][2] = -1.3;
//     obstacles[1][3] = .3;
//     obstacles[1][4] = .3;
//     obstacles[1][5] = .1;
// 
//     obstacles[2][0] = .7;
//     obstacles[2][1] = .4;
//     obstacles[2][2] = -.3;
//     obstacles[2][3] = .05;
//     obstacles[2][4] = .2;
//     obstacles[2][5] = .1;
// 
//     obstacles.resize(4);
//     obstacles[3].resize(3);
//     obstacles[3][0] = .2;
//     obstacles[3][1] = -.8;
//     obstacles[3][2] = .4;
//     obstacles[3][3] = .2;
//     obstacles[3][4] = .2;
//     obstacles[3][5] = .1;

//     environment_robarm.AddObstacles(obstacles);

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

    printf("setting epsilon....\n");
    //set epsilon
    planner.set_initialsolution_eps(environment_robarm.GetEpsilon());

    //set search mode (true - settle with first solution)
    planner.set_search_mode(true);

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);

    printf("completed in %.4f seconds.\n", double(clock()-starttime) / CLOCKS_PER_SEC);

    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    printf("\ntotal planning time is %.4f seconds.\n", double(clock()-totaltime) / CLOCKS_PER_SEC);

    // create filename with current time
    string outputfile = "sol";
    outputfile.append(".txt");

    FILE* fSol = fopen(outputfile.c_str(), "w");
    for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);
    }
    fclose(fSol);

//     if(environment_robarm.isPathValid(solution_stateIDs_V))
//         printf("Path is valid\n");
//     else
//         printf("Path is INVALID\n");

    //to get the trajectory as an array
//     double angles_r[NUMOFLINKS];
//     for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) 
//     {
//         environment_robarm.StateID2Angles(solution_stateIDs_V[i], angles_r);
//         for (int p = 0; p < 7; p++)
//             printf("% 0.2f  ",angles_r[p]);
//         printf("\n");
//     }

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

    //robotarm planning
    planrobarm(argc, argv);

    return 0;
}

