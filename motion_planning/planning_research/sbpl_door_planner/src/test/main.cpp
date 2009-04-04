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
#include <sbpl_door_planner/environment_navxythetadoor.h>
#include <sbpl/headers.h>


void PrintUsage(char *argv[])
{
  printf("USAGE: %s <cfg file>\n", argv[0]);
}


int planxythetadoor(int argc, char *argv[])
{

  int bRet = 0;
  double allocated_time_secs = 100; //in seconds
  MDPConfig MDPCfg;
  bool bsearchuntilfirstsolution = false;

  //set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
  vector<sbpl_2Dpt_t> perimeterptsV;
  sbpl_2Dpt_t pt_m;
  double halfwidth = 0.315;    //0.08; //0.3;
  double halflength = 0.315;  //0.1; //0.45;
  pt_m.x = -halflength;
  pt_m.y = -halfwidth;
  perimeterptsV.push_back(pt_m);
  pt_m.x = halflength;
  pt_m.y = -halfwidth;
  perimeterptsV.push_back(pt_m);
  pt_m.x = halflength;
  pt_m.y = halfwidth;
  perimeterptsV.push_back(pt_m);
  pt_m.x = -halflength;
  pt_m.y = halfwidth;
  perimeterptsV.push_back(pt_m);

  //Initialize Environment (should be called before initializing anything else)
  EnvironmentNAVXYTHETADOOR environment_navxythetadoor;

  if(argc == 3)
  {
    if(!environment_navxythetadoor.InitializeEnv(argv[1], perimeterptsV, argv[2]))
    {
      printf("ERROR: InitializeEnv failed\n");
      exit(1);
    }
  }
  else
  {
    if(!environment_navxythetadoor.InitializeEnv(argv[1], perimeterptsV, NULL))
    {
      printf("ERROR: InitializeEnv failed\n");
      exit(1);
    }
  }

  //Initialize MDP Info
  if(!environment_navxythetadoor.InitializeMDPCfg(&MDPCfg))
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  //Initialize the door information
  environment_navxythetadoor.footprint_.resize(4);
  environment_navxythetadoor.footprint_[0].resize(2);
  environment_navxythetadoor.footprint_[0][0] = halfwidth;
  environment_navxythetadoor.footprint_[0][1] = halfwidth;

  environment_navxythetadoor.footprint_[1].resize(2);
  environment_navxythetadoor.footprint_[1][0] = -halfwidth;
  environment_navxythetadoor.footprint_[1][1] = halfwidth;

  environment_navxythetadoor.footprint_[2].resize(2);
  environment_navxythetadoor.footprint_[2][0] = -halfwidth;
  environment_navxythetadoor.footprint_[2][1] = -halfwidth;

  environment_navxythetadoor.footprint_[3].resize(2);
  environment_navxythetadoor.footprint_[3][0] = halfwidth;
  environment_navxythetadoor.footprint_[3][1] = -halfwidth;

  environment_navxythetadoor.robot_shoulder_position_.resize(2);
  environment_navxythetadoor.robot_shoulder_position_[0] = -0.05;
  environment_navxythetadoor.robot_shoulder_position_[1] = -0.188;

  environment_navxythetadoor.door_handle_pose_.resize(2);
  environment_navxythetadoor.door_handle_pose_[0] = 0.7;
  environment_navxythetadoor.door_handle_pose_[1] = 0;

  environment_navxythetadoor.robot_global_pose_.resize(3);

  environment_navxythetadoor.door_global_pose_.resize(3);
  environment_navxythetadoor.door_global_pose_[0] = 0.25;
  environment_navxythetadoor.door_global_pose_[1] = 1.1;
  environment_navxythetadoor.door_global_pose_[2] = 0.0;

  environment_navxythetadoor.door_thickness_ = 0.01;
  environment_navxythetadoor.pivot_length_ = 0.0;
  environment_navxythetadoor.door_length_ = 0.9;

  environment_navxythetadoor.min_workspace_radius_ = 0.0;
  environment_navxythetadoor.max_workspace_radius_ = 2.0;

  environment_navxythetadoor.max_workspace_angle_ = 3*M_PI/2.0;
  environment_navxythetadoor.min_workspace_angle_ = -3*M_PI/2.0;
  environment_navxythetadoor.delta_angle_ = 0.1;



  //plan a path
  vector<int> solution_stateIDs_V;
  bool bforwardsearch = true;
  ARAPlanner planner(&environment_navxythetadoor, bforwardsearch);

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
  planner.set_initialsolution_eps(3.0);

  //set search mode
  planner.set_search_mode(bsearchuntilfirstsolution);

  printf("start planning...\n");
  bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
  printf("done planning\n");
  std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

  environment_navxythetadoor.PrintTimeStat(stdout);

  /*
    if(planner.set_start(environment_navxythetadoor.SetStart(1.6,0.5,0)) == 0)
    {
    printf("ERROR: failed to set start state\n");
    exit(1);
    }


    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetadoor.PrintTimeStat(stdout);


    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetadoor.PrintTimeStat(stdout);

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetadoor.PrintTimeStat(stdout);

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetadoor.PrintTimeStat(stdout);

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
  */

  FILE* fSol = fopen("sol.txt", "w");
  vector<EnvNAVXYTHETALAT3Dpt_t> xythetaPath;
  environment_navxythetadoor.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath);
  for(unsigned int i = 0; i < xythetaPath.size(); i++) {
    fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta);
  }
  fclose(fSol);

  environment_navxythetadoor.PrintTimeStat(stdout);

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

  if(argc < 2)
  {
    PrintUsage(argv);
    exit(1);
  }

  //xytheta door planning
  planxythetadoor(argc, argv);

  return 0;
}







