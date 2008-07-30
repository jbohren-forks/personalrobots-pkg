#include "../headers.h"

//clock_t time3_addallout = 0;
//clock_t time_gethash = 0;
//clock_t time_createhash = 0;


void PrintUsage(char *argv[])
{
	printf("USAGE: %s <cfg file>\n", argv[0]);
}


int main(int argc, char *argv[])
{
	int bRet = 0;
	double allocated_time_secs = 60.0*15; //in seconds
	MDPConfig MDPCfg;


	if(argc != 2)
	{
		PrintUsage(argv);
		exit(1);
	}

	
	//Initialize Environment (should be called before initializing anything else)
	EnvironmentNAV2D environment_nav2D;
	if(!environment_nav2D.InitializeEnv(argv[1]))
	{
		printf("ERROR: InitializeEnv failed\n");
		exit(1);
	}

	//Initialize MDP Info
	if(!environment_nav2D.InitializeMDPCfg(&MDPCfg))
	{
		printf("ERROR: InitializeMDPCfg failed\n");
		exit(1);
	}


	//plan a path
	vector<int> solution_stateIDs_V;
	ARAPlanner ara_planner(&environment_nav2D, &MDPCfg);
	bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);


	//print a path
	if(bRet)
	{
		//print the solution
		printf("Solution is found\n");
	}
	else
		printf("Solution does not exist\n");

	fflush(NULL);
	
	return 0;
}





