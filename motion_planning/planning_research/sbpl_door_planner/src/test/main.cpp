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


using namespace std;
#include <sbpl/headers.h>
#include <sbpl_door_planner/environment_navxythetadoor.h>

void PrintUsage(char *argv[])
{
  printf("USAGE: %s <cfg file>\n", argv[0]);
}

/*
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
  EnvironmentNAVXYTHETADOORLAT environment_navxythetadoor;

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

  door_msgs::Door door;
  door.frame_p1.x = 0.25;
  door.frame_p1.y = 1.1;
  door.frame_p2.x = 1.15;
  door.frame_p2.y = 1.1;
  door.door_p1.x = 0.25;
  door.door_p1.y = 1.1;
  door.door_p2.x = 1.15;
  door.door_p2.y = 1.1;

  door.handle.x = 0.95;
  door.handle.y = 1.1;

  door.travel_dir.x = 1.0;
  door.travel_dir.y = 0.0;
  door.travel_dir.z = 0.0;
  door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  door.hinge = door_msgs::Door::HINGE_P1;
  door.header.frame_id = "base_footprint";

  environment_navxythetadoor.setDoorProperties(door,0.05);

  //Initialize the door information
  environment_navxythetadoor.db_.footprint_.resize(5);
  environment_navxythetadoor.db_.footprint_[0].x = halfwidth;
  environment_navxythetadoor.db_.footprint_[0].y = halfwidth;

  environment_navxythetadoor.db_.footprint_[1].x = -halfwidth;
  environment_navxythetadoor.db_.footprint_[1].y = halfwidth;

  environment_navxythetadoor.db_.footprint_[2].x = -halfwidth;
  environment_navxythetadoor.db_.footprint_[2].y = -halfwidth;

  environment_navxythetadoor.db_.footprint_[3].x = halfwidth;
  environment_navxythetadoor.db_.footprint_[3].y = -halfwidth;

  // environment_navxythetadoor.db_.footprint_[4].x = halfwidth + halfwidth/2.0;
  // environment_navxythetadoor.db_.footprint_[4].y = 0;

  environment_navxythetadoor.db_.robot_shoulder_position_.x = -0.05;
  environment_navxythetadoor.db_.robot_shoulder_position_.y = -0.188;

  environment_navxythetadoor.db_.door_handle_position_.x = 0.756664; //0.7;
  environment_navxythetadoor.db_.door_handle_position_.y =  -0.039704; // 0;

//   SBPLDoorPlanner::Door inputDoor message in odom_combined at time 1.24469e+09
//       - frame (0.718432 -0.412216 0.00891274) -- (0.710015 0.453715 0.014776)
//       - door (0.741994 -0.411668 0.0091865) -- (0.710015 0.453715 0.014776)
//       - handle (0.677668 -0.303299 0.948164)
//       - travel_dir (0.999904 0.00964798 0.00996849)
//       - latch_state 2
//       - hinge side 2
//       - rot_dir 2
//       - angle [deg] 1.55938


  environment_navxythetadoor.db_.door_frame_global_position_.x = 0.718432; //0.25;
  environment_navxythetadoor.db_.door_frame_global_position_.y = -0.412216; //1.1;
  environment_navxythetadoor.db_.door_frame_global_yaw_ = -1.561076; //0.0;

  environment_navxythetadoor.db_.door_thickness_ = 0.01;
  environment_navxythetadoor.db_.pivot_length_ = 0.0;
  environment_navxythetadoor.db_.door_length_ = 0.9;

  environment_navxythetadoor.db_.arm_min_workspace_radius_ = 0.0;
  environment_navxythetadoor.db_.arm_max_workspace_radius_ = 0.8;

  environment_navxythetadoor.db_.arm_max_workspace_angle_ = M_PI/2.0;
  environment_navxythetadoor.db_.arm_min_workspace_angle_ = -M_PI/2.0;
  environment_navxythetadoor.db_.door_angle_discretization_interval_ = 0.175;

  environment_navxythetadoor.db_.global_door_open_angle_ = 0.009720; //M_PI/2.0;
  environment_navxythetadoor.db_.global_door_closed_angle_ = M_PI/2.0; //0.0;

  environment_navxythetadoor.db_.rot_dir_ = 1;
  environment_navxythetadoor.db_.init();

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


//     if(planner.set_start(environment_navxythetadoor.SetStart(1.6,0.5,0)) == 0)
//     {
//     printf("ERROR: failed to set start state\n");
//     exit(1);
//     }
// 
// 
//     printf("start planning...\n");
//     bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
//     printf("done planning\n");
//     std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
// 
//     environment_navxythetadoor.PrintTimeStat(stdout);
// 
// 
//     printf("start planning...\n");
//     bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
//     printf("done planning\n");
//     std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
// 
//     environment_navxythetadoor.PrintTimeStat(stdout);
// 
//     printf("start planning...\n");
//     bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
//     printf("done planning\n");
//     std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
// 
//     environment_navxythetadoor.PrintTimeStat(stdout);
// 
//     printf("start planning...\n");
//     bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
//     printf("done planning\n");
//     std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
// 
//     environment_navxythetadoor.PrintTimeStat(stdout);
// 
//     printf("start planning...\n");
//     bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
//     printf("done planning\n");
//     std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;
  

  FILE* fSol = fopen("sol.txt", "w");
  vector<EnvNAVXYTHETALAT3Dpt_t> xythetaPath;
  vector<unsigned char> doorintervalindPath;
  environment_navxythetadoor.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath, &doorintervalindPath);
  for(unsigned int i = 0; i < xythetaPath.size(); i++) {
    double door_angle;
    double door_angle_cost;
    if(!environment_navxythetadoor.GetMinCostDoorAngle(xythetaPath.at(i).x, xythetaPath.at(i).y, 
						       xythetaPath.at(i).theta, doorintervalindPath.at(i),
						       door_angle,door_angle_cost))
    {
      fprintf(fSol, "%.3f %.3f %.3f %.3f %.3f\n", 
	      xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta,0.0,0.0);
    }
    else
    {
      fprintf(fSol, "%.3f %.3f %.3f %.3f %.3f\n", 
	      xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta,door_angle,door_angle_cost);
    }
  }
  fclose(fSol);

  environment_navxythetadoor.db_.writeToFile("doordata.m");

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
*/

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
  EnvironmentNAVXYTHETADOORLAT environment_navxythetadoor;

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

  //   SBPLDoorPlanner::Door inputDoor message in odom_combined at time 1.24469e+09
//   Door hinge position: 2.135015 2.003715 0.014776
//       Door edge  position: 2.143432 1.137784 0.008913
//       Door global yaw: -1.561076
//       Rotation direction: 1
//       Handle position in door frame: 0.756664 -0.039704
//       Global door open angle: 0.009720

  //door message in global frame
  door_msgs::Door door;
  door.frame_p1.x = 2.135;
  door.frame_p1.y = 2.00;
  door.frame_p2.x = 2.14;
  door.frame_p2.y = 1.13;
  door.door_p1.x = 2.135;
  door.door_p1.y = 2.00;
  door.door_p2.x = 2.14;
  door.door_p2.y = 1.13;

  door.handle.x = 2.13;
  door.handle.y = 1.3;

  //unit vector describing the direction to go through the door way
  door.travel_dir.x = 1.0;
  door.travel_dir.y = 0.0;
  door.travel_dir.z = 0.0;
  door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  door.hinge = door_msgs::Door::HINGE_P1; 
  door.header.frame_id = "base_footprint";

  environment_navxythetadoor.setDoorProperties(door,0.05);

  //Initialize the door information
  environment_navxythetadoor.db_.footprint_.resize(4);
  environment_navxythetadoor.db_.footprint_[0].x = halfwidth;
  environment_navxythetadoor.db_.footprint_[0].y = halfwidth;

  environment_navxythetadoor.db_.footprint_[1].x = -halfwidth;
  environment_navxythetadoor.db_.footprint_[1].y = halfwidth;

  environment_navxythetadoor.db_.footprint_[2].x = -halfwidth;
  environment_navxythetadoor.db_.footprint_[2].y = -halfwidth;

  environment_navxythetadoor.db_.footprint_[3].x = halfwidth;
  environment_navxythetadoor.db_.footprint_[3].y = -halfwidth;

//  environment_navxythetadoor.db_.footprint_[4].x = halfwidth + halfwidth/2.0;
//  environment_navxythetadoor.db_.footprint_[4].y = 0;

  environment_navxythetadoor.db_.robot_shoulder_position_.x = -0.05;
  environment_navxythetadoor.db_.robot_shoulder_position_.y = -0.188;

/*  environment_navxythetadoor.db_.door_handle_position_.x = 0.7;
  environment_navxythetadoor.db_.door_handle_position_.y = 0;

  environment_navxythetadoor.db_.door_frame_global_position_.x = 0.25;
  environment_navxythetadoor.db_.door_frame_global_position_.y = 1.1;
  environment_navxythetadoor.db_.door_frame_global_yaw_ = -1.581783; //0.0;

  environment_navxythetadoor.db_.pivot_length_ = 0.0;
  environment_navxythetadoor.db_.door_length_ = 0.9;
  environment_navxythetadoor.db_.global_door_open_angle_ = M_PI/2.0;
  environment_navxythetadoor.db_.global_door_closed_angle_ = 0.0;
  environment_navxythetadoor.db_.door_thickness_ = 0.01;
  environment_navxythetadoor.db_.init();
  environment_navxythetadoor.db_.rot_dir_ = 1;
*/

  environment_navxythetadoor.db_.arm_min_workspace_radius_ = 0.0;
  environment_navxythetadoor.db_.arm_max_workspace_radius_ = 0.85;

  environment_navxythetadoor.db_.arm_max_workspace_angle_ = M_PI/2.0;
  environment_navxythetadoor.db_.arm_min_workspace_angle_ = -M_PI/2.0;
  environment_navxythetadoor.db_.door_angle_discretization_interval_ = 0.0175;

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
  vector<unsigned char> doorintervalindPath;
  environment_navxythetadoor.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath, &doorintervalindPath);
  for(unsigned int i = 0; i < xythetaPath.size(); i++) {
    double door_angle;
    double door_angle_cost;
    if(!environment_navxythetadoor.GetMinCostDoorAngle(xythetaPath.at(i).x, xythetaPath.at(i).y, 
        xythetaPath.at(i).theta, doorintervalindPath.at(i),
                       door_angle,door_angle_cost))
    {
      fprintf(fSol, "%.3f %.3f %.3f %.3f %.3f\n", 
              xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta,0.0,0.0);
    }
    else
    {
      fprintf(fSol, "%.3f %.3f %.3f %.3f %.3f\n", 
              xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta,door_angle,door_angle_cost);
    }
  }
  fclose(fSol);

  environment_navxythetadoor.db_.writeToFile("doordata.m");

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







