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
// #include "time.h"
#include <ros/node.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/PoseWithRatesStamped.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include "robot_msgs/MechanismState.h"

#define NUMOFLINKS 7
#define PI_CONST 3.141592653
#define INPUT_IN_DEGREES 1
#define OPEN 1.05
#define CLOSE 0.05

//0.571 0.574 0.997
// #define X_STOP 0.571
// #define Y_STOP 0.574
// #define Z_STOP 0.997
// #define MOE 0.005
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

#if INPUT_IN_DEGREES
                if(curr_angle[i] > 180.0)
                    curr_angle[i] = -360.0 + curr_angle[i];

                curr_angle[i] = curr_angle[i]*(PI_CONST/180.0);
#endif

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

//temporary
class gnd_truth
{
  public:
    std_msgs::PoseWithRatesStamped gripper; 
    void doNothing() {}
};

int main( int argc, char** argv )
{
    /*********** Initialize ROS  ****************/
    ros::init(argc,argv);
    ros::Node *node = new ros::Node("arm_trajectory_controller_client"); 
    
    signal(SIGINT,  finalize);
    signal(SIGQUIT, finalize);
    signal(SIGTERM, finalize);
    double X_STOP = 0.571;
    double Y_STOP = 0.574;
    double Z_STOP = 0.997;
    double MOE = 0.02;

    pr2_mechanism_controllers::TrajectoryStart::request  req;
    pr2_mechanism_controllers::TrajectoryStart::response res;
    
    pr2_mechanism_controllers::TrajectoryQuery::request  req_q;
    pr2_mechanism_controllers::TrajectoryQuery::response res_q;

    std_msgs::Float64 GripperPos;
    GripperPos.data = OPEN;

    node->advertise<std_msgs::Float64>("r_gripper_controller/set_command",10);
    sleep(2);
    node->publish("/r_gripper_controller/set_command",GripperPos);
    sleep(5);

    gnd_truth truth;
    node->subscribe("/r_gripper_palm_pose_ground_truth", truth.gripper, &gnd_truth::doNothing, 10);
    sleep(2);


    /*********** Parse the Trajectory files ************/
    FILE* fCfg = fopen(argv[1], "r");
    if(fCfg == NULL)
    {
        printf("ERROR: unable to open %s\n", argv[1]);
        exit(1);
    }
    std::vector<std::vector<float> > path1;
    path1 = ReadTrajectoryFile(fCfg);
    printf("There are %i steps to this trajectory\n",path1.size());

    //parse the second trajectory
    fCfg = fopen(argv[2], "r");
    if(fCfg == NULL)
    {
        printf("ERROR: unable to open %s\n", argv[1]);
        exit(1);
    }
    std::vector<std::vector<float> > path2;
    path2 = ReadTrajectoryFile(fCfg);
    printf("There are %i steps to this trajectory\n",path2.size());


    /*********** Start Path 1 ************/
    int num_points = path1.size();
    int num_joints = NUMOFLINKS;

    req.traj.set_points_size(num_points);
    req.requesttiming = 1;

    for(int i=0; i<num_points; i++)
        req.traj.points[i].set_positions_size(num_joints);


    for(int i=0; i < num_points; i++)
    {
        req.traj.points[i].positions[0] = path1[i].at(0);
        req.traj.points[i].positions[1] = path1[i].at(1);
        req.traj.points[i].positions[2] = path1[i].at(2);
        req.traj.points[i].positions[3] = path1[i].at(3);
        req.traj.points[i].positions[4] = path1[i].at(4);
        req.traj.points[i].positions[5] = path1[i].at(5);
        req.traj.points[i].positions[6] = path1[i].at(6);
        req.traj.points[i].time = 0.0;
    }


    if (ros::service::call("right_arm_trajectory_controller/TrajectoryStart", req, res))
    {
        ROS_INFO("Done");
    }
    req_q.trajectoryid = atoi(argv[1]);

    bool close_to_goal = 0;

    while(!res_q.done && !close_to_goal )
    {
        if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery", req_q, res_q))
        {
                sleep(.005);
                if((fabs(truth.gripper.pos.position.x - X_STOP)) < MOE && 
                    (fabs(truth.gripper.pos.position.y - Y_STOP)) < MOE && 
                    (fabs(truth.gripper.pos.position.z - Z_STOP)) < MOE)
                {
                    printf("%.3f %.3f %.3f\n",truth.gripper.pos.position.x,truth.gripper.pos.position.y,truth.gripper.pos.position.z);
                    if (ros::service::call("right_arm_trajectory_controller/TrajectoryCancel", req, res))
                    {
                            
                    }
                    close_to_goal = 1;
                    sleep(2);
                }
        }
        else
        {
            ROS_INFO("service call failed");
            sleep(.1);
            if(!ros::service::call("right_arm_trajectory_controller/TrajectoryQuery", req_q, res_q))  
                exit(1);
        }
    }
    printf("done with path 1.\n");

    // at destination 1, close gripper
    sleep(2);
    GripperPos.data = CLOSE;
    node->publish("r_gripper_controller/set_command",GripperPos);
    printf("published gripper close\n");
    sleep(4);

    /*********** Start Path 2 ************/
    num_points = path2.size();

    req.traj.set_points_size(num_points);
    req.requesttiming = 1;
    res_q.done = 0;

    for(int i=0; i<num_points; i++)
        req.traj.points[i].set_positions_size(num_joints);

    for(int i=0; i < num_points; i++)
    {
        req.traj.points[i].positions[0] = path2[i].at(0);
        req.traj.points[i].positions[1] = path2[i].at(1);
        req.traj.points[i].positions[2] = path2[i].at(2);
        req.traj.points[i].positions[3] = path2[i].at(3);
        req.traj.points[i].positions[4] = path2[i].at(4);
        req.traj.points[i].positions[5] = path2[i].at(5);
        req.traj.points[i].positions[6] = path2[i].at(6);
        req.traj.points[i].time = 0.0;
    }

  if (ros::service::call("right_arm_trajectory_controller/TrajectoryStart", req, res))
  {
//     ROS_INFO("Done");
    }
 while(!res_q.done)
    {
        if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery", req_q, res_q))
        {
                sleep(.02);
//                 printf("%.3f %.3f %.3f\n",truth.gripper.pos.position.x,truth.gripper.pos.position.y,truth.gripper.pos.position.z); 
        }
        else
        {
            ROS_INFO("service call failed");
            sleep(.4);
            if(!ros::service::call("right_arm_trajectory_controller/TrajectoryQuery", req_q, res_q))  
                exit(1);
        }
    }

    sleep(3);
    // at destination 1, open gripper
    GripperPos.data = OPEN;
    node->publish("r_gripper_controller/set_command",GripperPos);
    sleep(3);

    ros::fini();
}
