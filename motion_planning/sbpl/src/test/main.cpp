/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>

#include "../headers.h"


void PrintUsage(char *argv[])
{
	printf("USAGE: %s <cfg file>\n", argv[0]);
}


int main(int argc, char *argv[])
{
	int bRet = 0;
	double allocated_time_secs = 0.5; //in seconds
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
	ARAPlanner ara_planner(&environment_nav2D);

    if(ara_planner.set_start(MDPCfg.startstateid) == 0)
        {
            printf("ERROR: failed to set start state\n");
            exit(1);
        }

    if(ara_planner.set_goal(MDPCfg.goalstateid) == 0)
        {
            printf("ERROR: failed to set goal state\n");
            exit(1);
        }

    printf("start planning...\n");
	bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    FILE* fSol = fopen("sol.txt", "w");
	for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
	  environment_nav2D.PrintState(solution_stateIDs_V[i], true, fSol);
	}

    environment_nav2D.PrintTimeStat(stdout);

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





