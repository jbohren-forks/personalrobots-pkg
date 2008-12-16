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
#include "time.h"
#include "robot_msgs/MechanismState.h"
#include "pr2_mechanism_controllers/JointPosCmd.h"

#define GRIPPER_ROLL_LEFT_JOINT 4
#define WRIST_FLEX_LEFT_JOINT 5
#define FOREARM_ROLL_LEFT_JOINT 6
#define ELBOW_FLEX_LEFT_JOINT 7
#define UPPERARM_ROLL_LEFT_JOINT 8
#define SHOULDER_PITCH_LEFT_JOINT 9
#define SHOULDER_PAN_LEFT_JOINT 10

#define MOE .05

#define NUMOFLINKS 7
#define PI_CONST 3.141592653

static int done = 0;

void finalize(int donecare)
{
  done = 1;
}

class set_arm_traj
{
  public:

    robot_msgs::MechanismState in;  // Our input msg
    pr2_mechanism_controllers::JointPosCmd LeftArmCmd;  

    set_arm_traj()
    {
        //set number of joints to command            
        LeftArmCmd.set_names_size(7);
        LeftArmCmd.set_positions_size(7);
        LeftArmCmd.set_margins_size(7);
        LeftArmCmd.timeout = 10;

        // names of joints to control
        LeftArmCmd.names[0] = "shoulder_pan_left_joint";
        LeftArmCmd.names[1] = "shoulder_pitch_left_joint";
        LeftArmCmd.names[2] = "upperarm_roll_left_joint";
        LeftArmCmd.names[3] = "elbow_flex_left_joint";
        LeftArmCmd.names[4] = "forearm_roll_left_joint";
        LeftArmCmd.names[5] = "wrist_flex_left_joint";
        LeftArmCmd.names[6] = "gripper_roll_left_joint";

        LeftArmCmd.positions[0] = 0;
        LeftArmCmd.positions[1] = 0;
        LeftArmCmd.positions[2] = 0;
        LeftArmCmd.positions[3] = 0;
        LeftArmCmd.positions[4] = 0;
        LeftArmCmd.positions[5] = 0;
        LeftArmCmd.positions[6] = 0;

        // margin of allowable error
        LeftArmCmd.margins[0] = 0.0;
        LeftArmCmd.margins[1] = 0.0;
        LeftArmCmd.margins[2] = 0.0;
        LeftArmCmd.margins[3] = 0.0;
        LeftArmCmd.margins[4] = 0.0;
        LeftArmCmd.margins[5] = 0.0;
        LeftArmCmd.margins[6] = 0.0;
    }

    ~set_arm_traj() {}

    void doNothing()
    {
    }
};

std::vector<std::vector<float> > ReadTrajectoryFile(FILE* fCfg) // std::vector <std::vector <float> > traj)
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
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                curr_angle[i] = atof(sTemp);

                if(curr_angle[i] > 180.0)
                    curr_angle[i] = -360.0 + curr_angle[i];

                curr_angle[i] = curr_angle[i]*(PI_CONST/180.0);

                std::cout << curr_angle[i] << "  ";
            }
            std::cout << std::endl;

            traj.push_back(curr_angle);

            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);
            fscanf(fCfg, "%s", sTemp);
        }
    }
    return traj;
}


int main( int argc, char** argv )
{

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

    /*********** Initialize ROS  ****************/
    ros::init(argc,argv);
    ros::node *node = new ros::node("test_arm_traj"); 

    set_arm_traj sat;
    node->subscribe("mechanism_state",sat.in, &set_arm_traj::doNothing,10);

    signal(SIGINT,  finalize);
    signal(SIGQUIT, finalize);
    signal(SIGTERM, finalize);

    /*********** Start moving the robot ************/
    double run_time = 25.0;
    ros::Time iteration_time;
    bool run_time_set = true;
    int m,n,g;

    node->advertise<pr2_mechanism_controllers::JointPosCmd>("left_arm_commands",10);
    sleep(1);
    node->publish("left_arm_commands", sat.LeftArmCmd);
    sleep(1);

    ros::Time start_time = ros::Time::now();
    ros::Duration sleep_time(0.01);

    //subtract one for now because the goal is all zeros
    for (m = 0; m < traj.size(); m++)
    {
        iteration_time = ros::Time::now();
        done = 0;
        for(n = 0; n < NUMOFLINKS; n++)
        {
            sat.LeftArmCmd.positions[n] = double(traj[m].at(n));
        }

        // publish output
        node->publish("left_arm_commands", sat.LeftArmCmd);

        while(!done)
        {
//             ros::Duration delta_time = ros::Time::now() - start_time;

            ros::Duration delta_time = ros::Time::now() - iteration_time;

            if(run_time_set && delta_time.toSec() > run_time)
            {
                printf("It's taking too long to get to waypoint #%i...\n\n",m+1);
                g = 6;
                for (int i = GRIPPER_ROLL_LEFT_JOINT; i <= SHOULDER_PAN_LEFT_JOINT; i++)
                {
                    printf("%s: %2.3f (%2.3f)\n", sat.in.joint_states[i].name.c_str(),sat.in.joint_states[i].position, traj[m].at(g));
                    g--;
                }
                printf("\n");
                break;
            }

            if ((sat.in.joint_states[GRIPPER_ROLL_LEFT_JOINT].position + MOE >= traj[m].at(6) &&
                 sat.in.joint_states[GRIPPER_ROLL_LEFT_JOINT].position - MOE <= traj[m].at(6)) &&
                (sat.in.joint_states[WRIST_FLEX_LEFT_JOINT].position + MOE >= traj[m].at(5) &&
                 sat.in.joint_states[WRIST_FLEX_LEFT_JOINT].position - MOE <= traj[m].at(5)) &&
                (sat.in.joint_states[FOREARM_ROLL_LEFT_JOINT].position + MOE >= traj[m].at(4) &&
                 sat.in.joint_states[FOREARM_ROLL_LEFT_JOINT].position - MOE <= traj[m].at(4)) &&
                (sat.in.joint_states[ELBOW_FLEX_LEFT_JOINT].position + MOE >= traj[m].at(3) &&
                 sat.in.joint_states[ELBOW_FLEX_LEFT_JOINT].position - MOE <= traj[m].at(3)) &&
                (sat.in.joint_states[UPPERARM_ROLL_LEFT_JOINT].position + MOE >= traj[m].at(2) &&
                 sat.in.joint_states[UPPERARM_ROLL_LEFT_JOINT].position - MOE <= traj[m].at(2)) &&
                (sat.in.joint_states[SHOULDER_PITCH_LEFT_JOINT].position + MOE >= traj[m].at(1) &&
                 sat.in.joint_states[SHOULDER_PITCH_LEFT_JOINT].position - MOE <= traj[m].at(1)) &&
                (sat.in.joint_states[SHOULDER_PAN_LEFT_JOINT].position + MOE >= traj[m].at(0) &&
                 sat.in.joint_states[SHOULDER_PAN_LEFT_JOINT].position - MOE <= traj[m].at(0)))
            {
                printf("\nArm is at waypoint #%i of %i\n\n", m+1, traj.size());
                g = 6;
                for (int i = GRIPPER_ROLL_LEFT_JOINT; i <= SHOULDER_PAN_LEFT_JOINT; i++)
                {
                    printf("%s: %2.3f (%2.3f)\n", sat.in.joint_states[i].name.c_str(),sat.in.joint_states[i].position, traj[m].at(g));
                    g--;
                }
                printf("\n");
                done = 1;
            }
            sleep_time.sleep();
        }
    }

    node->unsubscribe("mechanism_state");
    node->unadvertise("left_arm_commands");
    ros::fini();
    return 0;
}
