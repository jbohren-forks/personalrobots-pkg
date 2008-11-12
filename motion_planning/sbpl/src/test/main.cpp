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


int plan2d(int argc, char *argv[])
{

	int bRet = 0;
	double allocated_time_secs = 0.5; //in seconds
	MDPConfig MDPCfg;
	
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
	bool bforwardsearch = false;
	ARAPlanner planner(&environment_nav2D, bforwardsearch);

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

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    FILE* fSol = fopen("sol.txt", "w");
	for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
	  environment_nav2D.PrintState(solution_stateIDs_V[i], true, fSol);
	}
    fclose(fSol);

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


    return bRet;
}



int planandnavigate2d(int argc, char *argv[])
{

	double allocated_time_secs_foreachplan = 0.5; //in seconds
	MDPConfig MDPCfg;
	EnvironmentNAV2D environment_nav2D;
	EnvironmentNAV2D trueenvironment_nav2D;
    int size_x=-1,size_y=-1;
    int startx = 0, starty = 0;
    int goalx=-1, goaly=-1;
    FILE* fSol = fopen("sol.txt", "w");
    int dx[8] = {-1, -1, -1,  0,  0,  1,  1,  1};
    int dy[8] = {-1,  0,  1, -1,  1, -1,  0,  1};
	bool bPrint = true;
	int x,y;
	vector<int> preds_of_changededgesIDV;
	vector<nav2dcell_t> changedcellsV;
	nav2dcell_t nav2dcell;
	int i;
	const unsigned char obsthresh = 1;
	//srand(0);

    //initialize true map and robot map
	if(!trueenvironment_nav2D.InitializeEnv(argv[1]))
	{
		printf("ERROR: InitializeEnv failed\n");
		exit(1);
	}
	trueenvironment_nav2D.GetEnvParms(&size_x, &size_y, &startx, &starty, &goalx, &goaly);
    unsigned char* map = (unsigned char*)calloc(size_x*size_y, sizeof(unsigned char));

	//print the map
	if(bPrint) printf("true map:\n");
	for(y = 0; bPrint && y < size_y; y++){
		for(x = 0; x < size_x; x++){
			printf("%d ", (int)trueenvironment_nav2D.IsObstacle(x,y));
		}
		printf("\n");
	}
	if(bPrint) system("pause");

	//Initialize Environment (should be called before initializing anything else)
    if(!environment_nav2D.InitializeEnv(size_x, size_y, map, startx, starty, goalx, goaly, obsthresh)){
		printf("ERROR: InitializeEnv failed\n");
		exit(1);
	}

	//Initialize MDP Info
	if(!environment_nav2D.InitializeMDPCfg(&MDPCfg))
	{
		printf("ERROR: InitializeMDPCfg failed\n");
		exit(1);
	}


	//create a planner
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
    //ARAPlanner planner(&environment_nav2D, bforwardsearch);
	ADPlanner planner(&environment_nav2D, bforwardsearch);



    //set the start and goal configurations
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

    //now comes the main loop
    int goalthresh = 0;
    while(abs(startx - goalx) > goalthresh || abs(starty - goaly) > goalthresh){

        //simulate sensor data update
        bool bChanges = false;
		preds_of_changededgesIDV.clear();
		changedcellsV.clear();
        for(i = 0; i < 8; i++){
            int x = startx + dx[i];
            int y = starty + dy[i];
            if(x < 0 || x >= size_x || y < 0 || y >= size_y)
                continue;
            int index = x + y*size_x;
			unsigned char truecost = trueenvironment_nav2D.GetMapCost(x,y);
            if(map[index] != truecost){
                map[index] = truecost;
                environment_nav2D.UpdateCost(x,y,map[index]);
                printf("setting cost[%d][%d] to %d\n", x,y,map[index]);
                bChanges = true;
				//store the changed cells
				nav2dcell.x = x;
				nav2dcell.y = y;
				changedcellsV.push_back(nav2dcell);
            }
        }
		
        if(bChanges){
            //planner.costs_changed(); //use by ARA* planner (non-incremental)

			//get the affected states
			environment_nav2D.GetPredsofChangedEdges(&changedcellsV, &preds_of_changededgesIDV);
			//let know the incremental planner about them
			planner.update_preds_of_changededges(&preds_of_changededgesIDV); //use by AD* planner (incremental)
        }


        fprintf(fSol, "current state: %d %d\n",  startx, starty);

        //plan a path 
        bool bPlanExists = false;
        while(bPlanExists == false){
            printf("new planning...\n");   
            bPlanExists = (planner.replan(allocated_time_secs_foreachplan, &solution_stateIDs_V) == 1);
            printf("done with the solution of size=%d\n", solution_stateIDs_V.size());   
            environment_nav2D.PrintTimeStat(stdout);

            //for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
            //environment_nav2D.PrintState(solution_stateIDs_V[i], true, fSol);
            //}
            //fprintf(fSol, "*********\n");
        }

		//print the map
		int startindex = startx + starty*size_x;
		int goalindex = goalx + goaly*size_x;
		for(y = 0; bPrint && y < size_y; y++){
			for(x = 0; x < size_x; x++){
				int index = x + y*size_x;

				//check to see if it is on the path
				bool bOnthePath = false;
				for(int j = 1; j < (int)solution_stateIDs_V.size(); j++)
				{
					int newx, newy;
					environment_nav2D.GetCoordFromState(solution_stateIDs_V[j], newx, newy);
					if(x == newx && y == newy)
						bOnthePath = true;
				}

				if (index != startindex && index != goalindex && !bOnthePath)
					printf("%3d ", map[index]);
				else if(index == startindex)
					printf("  R ");
				else if(index == goalindex)
					printf("  G ");
				else if (bOnthePath)
					printf("  * ");
				else
					printf("  ? ");
			}
			printf("\n");
		}
		if(bPrint) system("pause");


        //move along the path
        if(bPlanExists && (int)solution_stateIDs_V.size() > 1){
            //get coord of the successor
            int newx, newy;
            environment_nav2D.GetCoordFromState(solution_stateIDs_V[1], newx, newy);

			if(trueenvironment_nav2D.GetMapCost(newx,newy) >= obsthresh)
			{
				printf("ERROR: robot is commanded to move into an obstacle\n");
				exit(1);
			}

            //move
            printf("moving from %d %d to %d %d\n", startx, starty, newx, newy);
            startx = newx;
            starty = newy;
			
            //update the environment
            environment_nav2D.SetStart(startx, starty);
            
            //update the planner
            if(planner.set_start(solution_stateIDs_V[1]) == 0){               
                printf("ERROR: failed to update robot pose in the planner\n");
                exit(1);
            }
        }

    }
	fflush(NULL);


    return 1;
}


int planandnavigate3dkin(int argc, char *argv[])
{

	double allocated_time_secs_foreachplan = 50; //in seconds
	MDPConfig MDPCfg;
	EnvironmentNAV3DKIN environment_nav3Dkin;
	EnvironmentNAV3DKIN trueenvironment_nav3Dkin;
    int size_x=-1,size_y=-1;
    double startx = 0, starty = 0, starttheta = 0;
    double goalx=-1, goaly=-1, goaltheta = -1;
	double goaltol_x = 0.1, goaltol_y = 0.1, goaltol_theta = 0.1;
    FILE* fSol = fopen("sol.txt", "w");
    //int dx[8] = {-1, -1, -1,  0,  0,  1,  1,  1};
    //int dy[8] = {-1,  0,  1, -1,  1, -1,  0,  1};
	const int numofsensingcells = 16;
    int dx[numofsensingcells] = {-2, -2, -2, -2, -2, -1, -1,  0,  0,  1,  1,  2,  2,  2,  2,  2}; //used for defining the cells that are sensed
    int dy[numofsensingcells] = {-2, -1,  0,  1,  2, -2,  2, -2,  2, -2,  2, -2, -1,  0,  1,  2};
	bool bPrint = true, bPrintMap = false;
	int x,y;
	vector<int> preds_of_changededgesIDV;
	vector<nav2dcell_t> changedcellsV;
	nav2dcell_t nav2dcell;
	int i;
	double cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs;
	vector<sbpl_2Dpt_t> perimeterptsV;


	//srand(0);

    //initialize true map and robot map
	if(!trueenvironment_nav3Dkin.InitializeEnv(argv[1]))
	{
		printf("ERROR: InitializeEnv failed\n");
		exit(1);
	}
	trueenvironment_nav3Dkin.GetEnvParms(&size_x, &size_y, &startx, &starty, &starttheta, &goalx, &goaly, &goaltheta, &cellsize_m, &nominalvel_mpersecs, &timetoturn45degsinplace_secs);

    unsigned char* map = (unsigned char*)calloc(size_x*size_y, sizeof(unsigned char));

	//print the map
	if(bPrintMap) printf("true map:\n");
	for(y = 0; bPrintMap && y < size_y; y++){
		for(x = 0; x < size_x; x++){
			printf("%d ", (int)trueenvironment_nav3Dkin.IsObstacle(x,y));
		}
		printf("\n");
	}
	if(bPrint) system("pause");

	printf("start: %f %f %f, goal: %f %f %f\n", startx, starty, starttheta, goalx, goaly, goaltheta);

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	sbpl_2Dpt_t pt_m;
	double side = 0.1;
	pt_m.x = -side;
	pt_m.y = -side;
	perimeterptsV.push_back(pt_m);
	pt_m.x = side;
	pt_m.y = -side;
	perimeterptsV.push_back(pt_m);
	pt_m.x = side;
	pt_m.y = side;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -side;
	pt_m.y = side;
	perimeterptsV.push_back(pt_m);

	//Initialize Environment (should be called before initializing anything else)
    if(!environment_nav3Dkin.InitializeEnv(size_x, size_y, map, startx, starty, starttheta, goalx, goaly, goaltheta, 
		goaltol_x, goaltol_y, goaltol_theta, perimeterptsV,
		cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs)){
		printf("ERROR: InitializeEnv failed\n");
		exit(1);
	}

	//Initialize MDP Info
	if(!environment_nav3Dkin.InitializeMDPCfg(&MDPCfg))
	{
		printf("ERROR: InitializeMDPCfg failed\n");
		exit(1);
	}


	//create a planner
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
    ARAPlanner planner(&environment_nav3Dkin, bforwardsearch);
	//ADPlanner planner(&environment_nav3Dkin);



    //set the start and goal configurations
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

    //now comes the main loop
	int goalx_c = CONTXY2DISC(goalx, cellsize_m);
	int goaly_c = CONTXY2DISC(goaly, cellsize_m);
	int goaltheta_c = ContTheta2Disc(goaltheta, NAV3DKIN_THETADIRS);
	printf("goal_c: %d %d %d\n", goalx_c, goaly_c, goaltheta_c);
    while(fabs(startx - goalx) > goaltol_x || fabs(starty - goaly) > goaltol_y || fabs(starttheta - goaltheta) > goaltol_theta){

        //simulate sensor data update
        bool bChanges = false;
		preds_of_changededgesIDV.clear();
		changedcellsV.clear();
        for(i = 0; i < numofsensingcells; i++){
            int x = CONTXY2DISC(startx,cellsize_m) + dx[i];
            int y = CONTXY2DISC(starty,cellsize_m) + dy[i];
            if(x < 0 || x >= size_x || y < 0 || y >= size_y)
                continue;
            int index = x + y*size_x;
            if(map[index] != 1 && trueenvironment_nav3Dkin.IsObstacle(x,y)){
                map[index] = 1;
                environment_nav3Dkin.UpdateCost(x,y,map[index]);
                printf("setting cost[%d][%d] to %d\n", x,y,map[index]);
                bChanges = true;
				//store the changed cells
				nav2dcell.x = x;
				nav2dcell.y = y;
				changedcellsV.push_back(nav2dcell);
            }
        }
		
        if(bChanges){
            planner.costs_changed(); //use by ARA* planner (non-incremental)

			//get the affected states
			//environment_nav3Dkin.GetPredsofChangedEdges(&changedcellsV, &preds_of_changededgesIDV);
			//let know the incremental planner about them
			//planner.update_preds_of_changededges(&preds_of_changededgesIDV); //use by AD* planner (incremental)
        }

		int startx_c = CONTXY2DISC(startx,cellsize_m);
		int starty_c = CONTXY2DISC(starty,cellsize_m);
		int starttheta_c = ContTheta2Disc(starttheta, NAV3DKIN_THETADIRS);
		

        fprintf(fSol, "current state: %d %d %d\n",  startx_c, starty_c, starttheta_c);

        //plan a path 
        bool bPlanExists = false;

		printf("new planning...\n");   
        bPlanExists = (planner.replan(allocated_time_secs_foreachplan, &solution_stateIDs_V) == 1);
        printf("done with the solution of size=%d\n", solution_stateIDs_V.size());   
        environment_nav3Dkin.PrintTimeStat(stdout);

        //for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        //environment_nav3Dkin.PrintState(solution_stateIDs_V[i], true, fSol);
        //}
        //fprintf(fSol, "*********\n");

		//print the map
		int startindex = startx_c + starty_c*size_x;
		int goalindex = goalx_c + goaly_c*size_x;
		for(y = 0; bPrintMap && y < size_y; y++){
			for(x = 0; x < size_x; x++){
				int index = x + y*size_x;

				//check to see if it is on the path
				bool bOnthePath = false;
				for(int j = 1; j < (int)solution_stateIDs_V.size(); j++)
				{
					int newx, newy, newtheta=0;
					environment_nav3Dkin.GetCoordFromState(solution_stateIDs_V[j], newx, newy, newtheta); //TODO - check against all thetas
					if(x == newx && y == newy)
						bOnthePath = true;
				}

				if (index != startindex && index != goalindex && !bOnthePath)
					printf("%d ", map[index]);
				else if(index == startindex)
					printf("X ");
				else if(index == goalindex)
					printf("G ");
				else if (bOnthePath)
					printf("* ");
				else
					printf("? ");
			}
			printf("\n");
		}


        //move along the path
        if(bPlanExists && (int)solution_stateIDs_V.size() > 1){
            //get coord of the successor
            int newx, newy, newtheta;
            environment_nav3Dkin.GetCoordFromState(solution_stateIDs_V[1], newx, newy, newtheta);

			if(trueenvironment_nav3Dkin.IsObstacle(newx,newy))
			{
				printf("ERROR: robot is commanded to move into an obstacle\n");
				exit(1);
			}

            //move
            printf("moving from %d %d %d to %d %d %d\n", startx_c, starty_c, starttheta_c, newx, newy, newtheta);
            startx = DISCXY2CONT(newx, cellsize_m);
            starty = DISCXY2CONT(newy, cellsize_m);
			starttheta = DiscTheta2Cont(newtheta, NAV3DKIN_THETADIRS);
			
            //update the environment
            environment_nav3Dkin.SetStart(startx, starty,starttheta);
            
            //update the planner
            if(planner.set_start(solution_stateIDs_V[1]) == 0){               
                printf("ERROR: failed to update robot pose in the planner\n");
                exit(1);
            }
        }
		else
		{
			printf("No move is made\n");
		}

		if(bPrint) system("pause");


    }

	printf("goal reached!\n");

	fflush(NULL);


    return 1;
}


int planrobarm(int argc, char *argv[])
{

	int bRet = 0;
	double allocated_time_secs = 5.0; //in seconds
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
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = false;
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

    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    FILE* fSol = fopen("sol.txt", "w");
	for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
	  environment_robarm.PrintState(solution_stateIDs_V[i], true, fSol);
	}
    fclose(fSol);


 	//print a path
	if(bRet)
	{
		//print the solution
		printf("Solution is found\n");
	}
	else
		printf("Solution does not exist\n");

	fflush(NULL);


    return bRet;
}






int main(int argc, char *argv[])
{

	if(argc != 2)
	{
		PrintUsage(argv);
		exit(1);
	}

    //2D planning
    //planandnavigate2d(argc, argv);

    //3D planning
    planandnavigate3dkin(argc, argv);


    //robotarm planning
    //planrobarm(argc, argv);

	
	return 0;
}





