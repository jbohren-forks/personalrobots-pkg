/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

// Author: Advait Jain

/**

  @mainpage

  @htmlinclude manifest.html

  @b pr2_kinematic_controllers provides functionality for moving the end effector to a desired cartesian pose (with interpolation), moving the end effector along the direction of the gripper (a reaching motion).

  <hr>

  @section usage Usage
  @verbatim
  $ pr2_kinematic_controllers [standard ROS args]
  @endverbatim

  <hr>

  @section topic ROS topics

  Subscribes to (name/type):
  - @b "right_pr2arm_set_end_effector" / @b pr2_msgs::EndEffectorState : Desired pose of the right arm's end effector. (This should probably be a service advertised by pr2_kinematic_controllers).
  - @b "left_pr2arm_set_end_effector" / @b pr2_msgs::EndEffectorState : Desired pose of the left arm. (Should probably be a service)
  - @b "left_pr2arm_pos" / @b std_msgs::PR2Arm : current joint angles of the left arm.
  - @b "right_pr2arm_pos" / @b std_msgs::PR2Arm : current joint angles of the right arm
  - @b "interpolate_step_size" / @b std_msgs::Float64 : step size for the linear interpolation. Should also be a service.
  - @b "interpolate_wait_time" / @b std_msgs::Float64 : wait in wall clock time (and seconds) between successive interpolation commands. Useful for making the trajectory smooth. Should also be a service.

  Publishes to (name / type):
  - @b "cmd_leftarm_cartesian" / @b pr2_msgs::EndEffectorState : desired cartesian pose of the left arm. Legacy from interpolated_kinematic_controllers. Should be converted over to a service like the right arm. Probably doesn't work either.
  - @b "rightarm_tooltip_cartesian" / @b pr2_msgs::EndEffectorState : Current pose of the end effector of the right arm.

  <hr>

  @section parameters ROS parameters

  - None

  <hr>

  @section services ROS services

  Advertises (name/type):
  - @b "move_along_gripper" / @b pr2_kinematic_contollers::Float64Int32 : move end effector in the direction of the gripper through the specidifed distance. (Reaching motion)

  Calls (name/type):
  - @b "reset_IK_guess" / @b rosgazebo::VoidVoid : command to the initialize the joint array for the initial guess for solving the IK with the current joint angles.
  - @b "set_rightarm_cartesian" / @b rosgazebo::MoveCartesian : desired end effector pose for the right arm.

**/



#include <ros/node.h>

#include <std_msgs/PR2Arm.h>
#include <std_msgs/Float64.h>
#include <pr2_msgs/EndEffectorState.h>
#include <pr2_kinematic_controllers/Float64Int32.h>
#include <rosgazebo/VoidVoid.h>
#include <rosgazebo/MoveCartesian.h>

#include <libpr2API/pr2API.h>
//#include <libKDL/kdl_kinematics.h>

#include <robot_kinematics/robot_kinematics.h>
#include <robot_kinematics/serial_chain.h>

#include <kdl/rotational_interpolation_sa.hpp>

#include <unistd.h>

using namespace std_msgs;
using namespace PR2;
using namespace KDL;

using namespace robot_kinematics;


class Pr2KinematicControllers : public ros::node
{
  public:

    Pr2KinematicControllers(void) : ros::node("pr2_kinematic_controller") 
    {
      // matches send.xml
      std::string pr2Content;
      get_param("robotdesc/pr2",pr2Content);
      pr2_kin.loadXMLString(pr2Content.c_str());

      right_arm = pr2_kin.getSerialChain("rightArm");

      step_size = 0.05;
      wait_time = 1;
      advertise<pr2_msgs::EndEffectorState>("cmd_leftarm_cartesian");

      advertise<pr2_msgs::EndEffectorState>("rightarm_tooltip_cartesian"); // position of tip of right arm in cartesian coord.
      advertise_service("move_along_gripper", &Pr2KinematicControllers::moveAlongGripper);

      subscribe("right_pr2arm_set_end_effector", _rightEndEffectorGoal, &Pr2KinematicControllers::setRightEndEffector);  
      subscribe("left_pr2arm_set_end_effector", _leftEndEffectorGoal, &Pr2KinematicControllers::setLeftEndEffector);
      subscribe("left_pr2arm_pos",  leftArmPosMsg,  &Pr2KinematicControllers::currentLeftArmPos);  // configuration of left arm.
      subscribe("right_pr2arm_pos", rightArmPosMsg, &Pr2KinematicControllers::currentRightArmPos); // configuration of right arm.

      subscribe("interpolate_step_size", float64_msg, &Pr2KinematicControllers::setStepSize);
      subscribe("interpolate_wait_time", float64_msg, &Pr2KinematicControllers::setWaitTime);  
    }

    void setWaitTime(void)
    {
      wait_time = float64_msg.data;
    }

    void setStepSize(void)
    {
      step_size = float64_msg.data;
    }

    void setRightEndEffector(void) 
    {
      KDL::Frame f;
      for(int i = 0; i < 9; i++)
        f.M.data[i] = _rightEndEffectorGoal.rot[i];

      for(int i = 0; i < 3; i++)
        f.p.data[i] = _rightEndEffectorGoal.trans[i];

      RunControlLoop(true, f, rightArmPosMsg);
    }

    void setLeftEndEffector(void) 
    {
      KDL::Frame f;
      for(int i = 0; i < 9; i++)
        f.M.data[i] = _leftEndEffectorGoal.rot[i];

      for(int i = 0; i < 3; i++)
        f.p.data[i] = _leftEndEffectorGoal.trans[i];

      RunControlLoop(false, f, leftArmPosMsg);
    }


    void currentLeftArmPos(void)
    {
    }

    void currentRightArmPos(void)
    {
      JntArray q = JntArray(right_arm->num_joints_);
      q(0) = rightArmPosMsg.turretAngle;
      q(1) = rightArmPosMsg.shoulderLiftAngle;
      q(2) = rightArmPosMsg.upperarmRollAngle;
      q(3) = rightArmPosMsg.elbowAngle;
      q(4) = rightArmPosMsg.forearmRollAngle;
      q(5) = rightArmPosMsg.wristPitchAngle;
      q(6) = rightArmPosMsg.wristRollAngle;

      Frame f;
      right_arm->computeFK(q,f);
      pr2_msgs::EndEffectorState efs;
      KDL_to_EndEffectorStateMsg(f, efs);
      publish("rightarm_tooltip_cartesian",efs);
    }

    void KDL_to_EndEffectorStateMsg(const Frame& f, pr2_msgs::EndEffectorState &efs)
    {
      efs.set_rot_size(9);
      efs.set_trans_size(3);
      for(int i = 0; i < 9; i++)
        efs.rot[i] = f.M.data[i];

      for(int i = 0; i < 3; i++)
        efs.trans[i] = f.p.data[i];
    }

    void publishFrame(bool isRightArm, const Frame& f)
    {
      pr2_msgs::EndEffectorState efs;
      KDL_to_EndEffectorStateMsg(f, efs);

      if(isRightArm)
      {
        printf("[pr2_kinematic_controllers/pr2_kinematic_controllers.cc] <publishFrame> For the right arm, SetRightArmCartesian should be used instead of publishFrame\nExiting..\n");
        exit(0);
      }
      else
        publish("cmd_leftarm_cartesian",efs);
    }

    void RunControlLoop(bool isRightArm, const Frame& r, const std_msgs::PR2Arm& arm)
    {
      rosgazebo::VoidVoid::request req;
      rosgazebo::VoidVoid::response res;

      cout<<"RunControlLoop: rotation: "<<r.M<<"\n";

      if (ros::service::call("reset_IK_guess", req, res)==false)
      {
        printf("[pr2_kinematic_controllers] <pr2_kinematic_controllers.cpp> reset_IK_guess service failed.\nExiting..\n");
        exit(0);
      }

      JntArray q = JntArray(right_arm->num_joints_);
      q(0) = arm.turretAngle;
      q(1) = arm.shoulderLiftAngle;
      q(2) = arm.upperarmRollAngle;
      q(3) = arm.elbowAngle;
      q(4) = arm.forearmRollAngle;
      q(5) = arm.wristPitchAngle;
      q(6) = arm.wristRollAngle;

      Frame f;
      right_arm->computeFK(q,f);
      Vector start = f.p;
      Vector move = r.p-start;
      double dist = move.Norm();
      move = move/dist;

      RotationalInterpolation_SingleAxis rotInterpolater;
      rotInterpolater.SetStartEnd(f.M, r.M);
      double total_angle = rotInterpolater.Angle();
      //  printf("Angle: %f\n", rotInterpolater.Angle());

      Vector target;
      int nSteps = (int)(dist/step_size);
      double angle_step = total_angle/nSteps;
      bool reachable = true;
      for(int i=0;i<nSteps && reachable==true;i++)
      {
        printf("[pr2_kinematic_controllers] interpolating...\n");
        f.p = start+(i+1)*move*step_size;
        f.M = rotInterpolater.Pos(angle_step*(i+1));

        if (isRightArm)
          reachable=SetRightArmCartesian(f); // services.
        else
          publishFrame(isRightArm, f); // old way of doing things. from interpolated_kinematic_controller.

        usleep(wait_time*1e6);
      }

      f.p = r.p;
      f.M = r.M;
      if (isRightArm)
        reachable=SetRightArmCartesian(f); // services.
      else
        publishFrame(isRightArm, f); // old way of doing things. from interpolated_kinematic_controller.

      if (reachable==false)
        printf("[pr2_kinematic_controllers] reachable became FALSE.\n");
    }

    bool SetRightArmCartesian(const Frame &f)
    {
      rosgazebo::MoveCartesian::request req;
      rosgazebo::MoveCartesian::response res;
      KDL_to_EndEffectorStateMsg(f, req.e);

      if (ros::service::call("set_rightarm_cartesian", req, res)==false)
      {
        printf("[pr2_kinematic_controllers] <pr2_kinematic_controllers.cpp> set_rightarm_cartesian service failed.\nExiting..\n");
        exit(0);
      }

      return (res.reachable==-1) ? false : true;
    }

    bool moveAlongGripper(pr2_kinematic_controllers::Float64Int32::request &req, pr2_kinematic_controllers::Float64Int32::response &res)
    {
      moveAlongGripper(req.f);
      return true;
    }

    void moveAlongGripper(double dist)
    {
      JntArray q = JntArray(right_arm->num_joints_);

      q(0) = rightArmPosMsg.turretAngle;
      q(1) = rightArmPosMsg.shoulderLiftAngle;
      q(2) = rightArmPosMsg.upperarmRollAngle;
      q(3) = rightArmPosMsg.elbowAngle;
      q(4) = rightArmPosMsg.forearmRollAngle;
      q(5) = rightArmPosMsg.wristPitchAngle;
      q(6) = rightArmPosMsg.wristRollAngle;
      Frame f;
      right_arm->computeFK(q,f);
      cout<<"current end effector position: "<<f.p<<"\n";
      cout<<"current end effector rotation: "<<f.M<<"\n";
      Vector v(0,0,dist);
      v = f*v;
      cout<<"final end effector position: "<<v<<"\n";
      cout<<"final end effector rotation: "<<f.M<<"\n";
      f.p=v;
      RunControlLoop(true, f, rightArmPosMsg);
    }

  private:

    pr2_msgs::EndEffectorState _leftEndEffectorGoal;
    pr2_msgs::EndEffectorState _rightEndEffectorGoal;
    std_msgs::PR2Arm leftArmPosMsg, rightArmPosMsg;
    double step_size;
    double wait_time;
    std_msgs::Float64 float64_msg;
    
    RobotKinematics pr2_kin;
    SerialChain *right_arm;

};

int main(int argc, char **argv)
{  
  ros::init(argc, argv);
  Pr2KinematicControllers easy;
  easy.spin();
  easy.shutdown();
  return 0;    
}

