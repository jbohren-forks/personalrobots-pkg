#include <iostream>
#include "../headers.h"


#define MAX_RUNTIME 40.0

void PrintUsage(char *argv[])
{
    printf("USAGE: %s <cfg file>\n", argv[0]);
}

int planrobarm(int argc, char *argv[])
{
    int bRet = 0;
    double allocated_time_secs = MAX_RUNTIME; //in seconds
    time_t t;
//     time_t seconds;
    time(&t);
    MDPConfig MDPCfg;

    //Initialize Environment (should be called before initializing anything else)
    EnvironmentROBARM environment_robarm;

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
    planner.set_initialsolution_eps(10.0);

    //set search mode (true - settle with first solution)
    planner.set_search_mode(true);

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);

    printf("completed in %.4f seconds.\n", double(clock()-starttime) / CLOCKS_PER_SEC);

    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    // create filename with current time
    string outputfile = "sol";
//     seconds = (time(NULL))/3600;
//     outputfile.append(ctime(&t));
//     outputfile.append(itoa(seconds));
    outputfile.append(".txt");

    FILE* fSol = fopen(outputfile.c_str(), "w");
    for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);
    }
    fclose(fSol);

    environment_robarm.CloseKinNode();

//  	//print a path
//     if(bRet)
//     {
// 		//print the solution
//         printf("Solution is found\n");
//     }
//     else
//         printf("Solution does not exist\n");
// 
//     fflush(NULL);
    
    return bRet;
}

int main(int argc, char *argv[])
{
//     int argc2 = 0;
//     char** argv2 = NULL;
//     ros::init(argc2,argv2);
//     ros::node calcFK_armplanner("calcFK_armplanner");

    if(argc != 2)
    {
        PrintUsage(argv);
        exit(1);
    }

    //robotarm planning
    planrobarm(argc, argv);

//     ros::fini();
//     sleep(1);

    return 0;
}

