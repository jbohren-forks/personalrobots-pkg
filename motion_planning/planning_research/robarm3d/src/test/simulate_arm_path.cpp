/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <iostream>
#include <fstream>
#include <vector>
#include <ros/node.h>
#include <pr2_mechanism_controllers/JointTraj.h>
#include "robot_msgs/MechanismState.h"

#include "time.h"
#include "robot_msgs/MechanismState.h"

#define MOE .05
#define NUMOFLINKS 7
#define PI_CONST 3.141592653

static int done = 0;

void finalize(int donecare)
{
  done = 1;
}

// parse trajectory file
std::vector<std::vector<float> > ReadTrajectoryFile(FILE* fCfg)
{
    char sTemp[1024];
    int i;

    std::vector<float> curr_angle(7);
    std::vector<std::vector<float> > traj;

    while(!feof(fCfg) && strlen(sTemp) != 0)
    {
        fscanf(fCfg, "%s", sTemp);
        if (strcmp(sTemp, "angles:") == 0)
        {
            for(i = 0; i < 7; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                curr_angle[i] = atof(sTemp);

                if(curr_angle[i] > 180.0)
                    curr_angle[i] = -360.0 + curr_angle[i];

                curr_angle[i] = curr_angle[i]*(PI_CONST/180.0);

//                 printf("%-3.3f  ",curr_angle[i]);
            }
//             printf("\n");


            // bad way to do this (skip text we don't want to parse)
            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);

            // check for gripper commands
            if (strcmp(sTemp, "OPEN") == 0)
            {
                for(i = 0; i < NUMOFLINKS; i++)
                {
                    curr_angle[i] = -2;
                }
                std::cout << "OPEN" << std::endl;
            }
            else if (strcmp(sTemp, "CLOSE") == 0)
            {
                for(i = 0; i < NUMOFLINKS; i++)
                {
                    curr_angle[i] = -1;
                }
                std::cout << "CLOSE" << std::endl;
            }
            else
            {
                for(i = 0; i < NUMOFLINKS; i++)
                    printf("%-3.3f  ",curr_angle[i]);
                printf("\n");
            }

            //add angle vector to vector list
            traj.push_back(curr_angle);
        }
    }
    return traj;
}

int main( int argc, char** argv )
{

    /*********** Initialize ROS  ****************/
    ros::init(argc,argv);
    ros::node *node = new ros::node("arm_trajectory_controller");

    signal(SIGINT,  finalize);
    signal(SIGQUIT, finalize);
    signal(SIGTERM, finalize);

    /*********** Parse the Trajectory file ************/
    // parse the trajectory file
    FILE* fCfg = fopen(argv[1], "r");
    if(fCfg == NULL)
    {
        printf("ERROR: unable to open %s\n", argv[1]);
        exit(1);
    }
    std::vector<std::vector<float> > traj;
    traj = ReadTrajectoryFile(fCfg);
    printf("There are %i steps to this trajectory\n",traj.size());

    /*********** Start moving the robot ************/
    pr2_mechanism_controllers::JointTraj cmd;

//     std_msgs::Float64 GripperPos;
//     GripperPos.data = 0.5;
//     bool open_gripper = 0, close_gripper = 0;
// 
//     node->advertise<std_msgs::Float64>("gripper_left_controller/set_command",10);
//     sleep(.025);

    int num_points = traj.size();
    int num_joints = NUMOFLINKS;

    cmd.set_points_size(num_points);

    for(int i=0; i < num_points; i++)
        cmd.points[i].set_positions_size(num_joints);

    for(int i=0; i < num_points; i++)
    {
        cmd.points[i].positions[0] = traj[i].at(0);
        cmd.points[i].positions[1] = traj[i].at(1);
        cmd.points[i].positions[2] = traj[i].at(2);
        cmd.points[i].positions[3] = traj[i].at(3);
        cmd.points[i].positions[4] = traj[i].at(4);
        cmd.points[i].positions[5] = traj[i].at(5);
        cmd.points[i].positions[6] = traj[i].at(6);
        cmd.points[i].time = 0.0;
    }


  node->advertise<pr2_mechanism_controllers::JointTraj>("arm_trajectory_command",1);
  sleep(1);
  node->publish("arm_trajectory_command",cmd);
  sleep(4);

  ros::Time start_time = ros::Time::now();
  ros::Duration sleep_time = ros::Duration().fromSec(0.01);

/*  while(!done)
  {
    ros::Duration delta_time = ros::Time::now() - start_time;
    sleep_time.sleep();
  }
*/
  ros::fini();
}

class set_arm_traj
{
  public:
    robot_msgs::MechanismState in;
    void doNothing() {}
};
