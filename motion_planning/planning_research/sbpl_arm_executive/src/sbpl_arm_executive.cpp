/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *W
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <sbpl_arm_executive/pr2_arm_node.h>

#define JS_GOAL 0

using namespace pr2_arm_node;

class ExecNode : public PR2ArmNode
{
  public:

    int state_;
    enum{INITIALIZED};
    ExecNode(std::string node_name, std::string arm_name, std::string gripper_name):PR2ArmNode(node_name,arm_name,gripper_name),state_(INITIALIZED){};
    robot_msgs::JointTrajPoint current_joint_positions_;
    std::vector<robot_msgs::JointTrajPoint> goal_joint_positions_;
    robot_msgs::JointTraj planned_path_;

    // cartesian goal
    std::vector<robot_msgs::Pose> goal_;


    void spin()
    {
        int num_joints = 7; int y, goal_id = 1; int at_goal = 0;
        double hp[7] = {-1.75, 0, 0, -0.5, 0, 0.3, 0};  //home position
        std::vector<double>home_position(hp,hp + sizeof(hp)/sizeof(*hp));

#if JS_GOAL
        //joint space goals
        double goal1[7] = {0.540 -0.270, 0.250, 0.900, 0.320, 1.740};
        double goal2[7] = {5.725, 0.175, 0.733, 6.178, 0.349, 0.942, 4.084};
        goal_joint_positions_.resize(1);
        goal_joint_positions_[0].set_positions_size(7);

        printf("initial movements complete....are you brave enough to continue?\n");
        scanf("%d",&y);

        while(ok())
        {
            at_goal = 0;

            if(goal_id == 1)
            {
                for(int i=0; i < num_joints; i++)
                    goal_joint_positions_[0].positions[i] = goal1[i];
                goal_id = 2;
            }

            else if(goal_id == 2)
            {
                for(int i=0; i < num_joints; i++)
                    goal_joint_positions_[0].positions[i] = goal2[i];
                goal_id = 1;
            }

            ROS_INFO("Planning");
            ROS_INFO("goal: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                    goal_joint_positions_[0].positions[0],goal_joint_positions_[0].positions[1],goal_joint_positions_[0].positions[2],goal_joint_positions_[0].positions[3],
                    goal_joint_positions_[0].positions[4],goal_joint_positions_[0].positions[5],goal_joint_positions_[0].positions[6]);

            //get current configuration
            getCurrentPosition(current_joint_positions_);

            //plan joint space path - change this so that it can replan if it doesn't find a path
            if(planSBPLPath(current_joint_positions_,goal_joint_positions_,planned_path_))
            {
                ROS_INFO("Planning was a success.\n");
            }
            else
                ROS_INFO("Planning failed: Retry");

#else
        //cartesian goals {x,y,z,r,p,y}
        double goal1[6] = {0.800, -0.440, -0.190, -0.860, -0.130, 0.560};
        double goal2[6] = {0.540, -0.270, 0.250, 0.900, 0.320, 1.740};
        goal_.resize(1);
        goal_[0] = RPYToTransform(goal1[3],goal1[4],goal1[5],goal1[0],goal1[1],goal1[2]);

        printf("initial movements complete....are you brave enough to continue?\n");
        scanf("%d",&y);

        while(ok())
        {
            at_goal = 0;

            if(goal_id == 1)
            {
                goal_[0] = RPYToTransform(goal1[3],goal1[4],goal1[5],goal1[0],goal1[1],goal1[2]);
                goal_id = 2;
            }
            else if(goal_id == 2)
            {
                goal_[0] = RPYToTransform(goal2[3],goal2[4],goal2[5],goal2[0],goal2[1],goal2[2]);
                goal_id = 1;
            }

            ROS_INFO("Planning");
            ROS_INFO("goal:  xyz: %.2f %.2f %.2f  quat: %.2f %.2f %.2f %.2f\n",
                    goal_[0].position.x,goal_[0].position.y,goal_[0].position.z,goal_[0].orientation.x,goal_[0].orientation.y,goal_[0].orientation.z,goal_[0].orientation.w);

            //get current configuration
            getCurrentPosition(current_joint_positions_);

            //plan joint space path - change this so that it can replan if it doesn't find a path
            if(planSBPLPath(current_joint_positions_,goal_,planned_path_))
            {
                ROS_INFO("Planning was a success.\n");
            }
            else
                ROS_INFO("Planning failed: Retry");
#endif

            //execute path
            if(sendTrajectory(arm_name_,planned_path_))
            {
                ROS_INFO("Executed trajectory successfully");
                sleep(25);
            }
            else
                ROS_INFO("Could not execute trajectory.");

            //check for collision
	    //            while(!at_goal)
            //{
            //    for(int i =0; i < planned_path_.size(); i++)
            //    {
            //        for (int j=0; j < NUMOFLINKS; j++)
            //        {
            //            req.joint_positions_
            //        {
            //    at_goal = checkPath(planned_path_,&resp);
            //}

            sleep(60);
        }
    }
};

int main(int argc, char *argv[])
{
  ros::init(argc,argv); 

  ExecNode node("sbpl_executive","right_arm","right_gripper");

  try 
  {
    node.spin();
  }
  catch(char const* e)
  {
    std::cout << e << std::endl;
  }
  return(0);
}
