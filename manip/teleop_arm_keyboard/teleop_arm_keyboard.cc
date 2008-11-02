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

  @b teleop_arm_keyboard teleoperates the arms of the PR2 by mapping
  key presses into joint angle set points.

  <hr>

  @section usage Usage
  @verbatim
  $ teleop_arm_keyboard [standard ROS args]
  @endverbatim

  Key mappings are printed to screen on startup.

  <hr>

  @section topic ROS topics

  Subscribes to (name/type):
  - None

  Publishes to (name / type):
  - @b "lArmCmd"/JointPosCmd : configuration of the left arm (all the joint angles); sent on every keypress.

  <hr>

  @section parameters ROS parameters

  - None

 **/

#include <termios.h>
#include <signal.h>
#include <math.h>

#include <ros/node.h>
#include <pr2_mechanism_controllers/JointPosCmd.h>
#include <std_msgs/Float64.h>

// For transform support
#include <rosTF/rosTF.h>

#define COMMAND_TIMEOUT_SEC 0.2

/// @todo Remove this giant enum, which was stoled from pr2Core/pr2Core.h.
/// It can be replaced by some simpler indexing scheme.
enum PR2_JOINT_ID
{
  CASTER_FL_STEER   , // 0
  CASTER_FL_DRIVE_L , // 1 
  CASTER_FL_DRIVE_R , // 2
  CASTER_FR_STEER   , // 3
  CASTER_FR_DRIVE_L , // 4
  CASTER_FR_DRIVE_R , // 5
  CASTER_RL_STEER   , // 6
  CASTER_RL_DRIVE_L , // 7 
  CASTER_RL_DRIVE_R , // 8
  CASTER_RR_STEER   , // 9 
  CASTER_RR_DRIVE_L , // 10
  CASTER_RR_DRIVE_R , // 11
  SPINE_ELEVATOR    ,
  ARM_L_PAN         , 
  ARM_L_SHOULDER_PITCH, 
  ARM_L_SHOULDER_ROLL,
  ARM_L_ELBOW_PITCH , 
  ARM_L_ELBOW_ROLL  ,
  ARM_L_WRIST_PITCH , 
  ARM_L_WRIST_ROLL  ,
  ARM_L_GRIPPER_GAP ,  // added 20080802 by john
  ARM_R_PAN         , 
  ARM_R_SHOULDER_PITCH, 
  ARM_R_SHOULDER_ROLL,
  ARM_R_ELBOW_PITCH , 
  ARM_R_ELBOW_ROLL  ,
  ARM_R_WRIST_PITCH , 
  ARM_R_WRIST_ROLL  ,
  ARM_R_GRIPPER_GAP ,  // added 20080802 by john
  HEAD_YAW          , 
  HEAD_PITCH        ,
  HEAD_LASER_PITCH  ,
  HEAD_PTZ_L_PAN    , 
  HEAD_PTZ_L_TILT   ,
  HEAD_PTZ_R_PAN    , 
  HEAD_PTZ_R_TILT   ,
  BASE_6DOF,
  PR2_WORLD,
  MAX_JOINTS   
};

class TArmK_Node : public ros::node
{
  private:
    pr2_mechanism_controllers::JointPosCmd lArmCmd;
    pr2_mechanism_controllers::JointPosCmd rArmCmd;
    std_msgs::Float64 lGripperCmd;
    std_msgs::Float64 rGripperCmd;

  public:
    TArmK_Node() : ros::node("tarmk"), tf(*this, true)
  {
    // cmd_armconfig should probably be initialised
    // with the current joint angles of the arm rather
    // than zeros.
    this->lArmCmd.set_names_size(7);
    this->rArmCmd.set_names_size(7);
    this->lArmCmd.set_positions_size(7);
    this->rArmCmd.set_positions_size(7);
    this->lArmCmd.set_margins_size(7);
    this->rArmCmd.set_margins_size(7);

    this->lArmCmd.names[0] = "shoulder_pan_left_joint";
    this->lArmCmd.names[1] = "shoulder_pitch_left_joint";
    this->lArmCmd.names[2] = "upperarm_roll_left_joint";
    this->lArmCmd.names[3] = "elbow_flex_left_joint";
    this->lArmCmd.names[4] = "forearm_roll_left_joint";
    this->lArmCmd.names[5] = "wrist_flex_left_joint";
    this->lArmCmd.names[6] = "gripper_roll_left_joint";

    this->lArmCmd.positions[0] = 0;
    this->lArmCmd.positions[1] = 0;
    this->lArmCmd.positions[2] = 0;
    this->lArmCmd.positions[3] = 0;
    this->lArmCmd.positions[4] = 0;
    this->lArmCmd.positions[5] = 0;
    this->lArmCmd.positions[6] = 0;

    this->lArmCmd.margins[0] = 0;
    this->lArmCmd.margins[1] = 0;
    this->lArmCmd.margins[2] = 0;
    this->lArmCmd.margins[3] = 0;
    this->lArmCmd.margins[4] = 0;
    this->lArmCmd.margins[5] = 0;
    this->lArmCmd.margins[6] = 0;

    this->rArmCmd.names[0] = "shoulder_pan_right_joint";
    this->rArmCmd.names[1] = "shoulder_pitch_right_joint";
    this->rArmCmd.names[2] = "upperarm_roll_right_joint";
    this->rArmCmd.names[3] = "elbow_flex_right_joint";
    this->rArmCmd.names[4] = "forearm_roll_right_joint";
    this->rArmCmd.names[5] = "wrist_flex_right_joint";
    this->rArmCmd.names[6] = "gripper_roll_right_joint";

    this->rArmCmd.positions[0] = 0;
    this->rArmCmd.positions[1] = 0;
    this->rArmCmd.positions[2] = 0;
    this->rArmCmd.positions[3] = 0;
    this->rArmCmd.positions[4] = 0;
    this->rArmCmd.positions[5] = 0;
    this->rArmCmd.positions[6] = 0;

    this->rArmCmd.margins[0] = 0;
    this->rArmCmd.margins[1] = 0;
    this->rArmCmd.margins[2] = 0;
    this->rArmCmd.margins[3] = 0;
    this->rArmCmd.margins[4] = 0;
    this->rArmCmd.margins[5] = 0;
    this->rArmCmd.margins[6] = 0;

    advertise<pr2_mechanism_controllers::JointPosCmd>("left_arm_commands");
    advertise<pr2_mechanism_controllers::JointPosCmd>("right_arm_commands");
    advertise<std_msgs::Float64>("gripper_left_controller/set_command");
    advertise<std_msgs::Float64>("gripper_right_controller/set_command");

    // deal with grippers separately
    this->lGripperCmd.data      = 0;
    this->rGripperCmd.data     = 0;

  }
    ~TArmK_Node() { }

    void printCurrentJointValues() {
      std::cout << "left arm " << std::endl;
      std::cout << " cmds: "
          << " " << this->lArmCmd.positions[0]
          << " " << this->lArmCmd.positions[1]
          << " " << this->lArmCmd.positions[2]
          << " " << this->lArmCmd.positions[3]
          << " " << this->lArmCmd.positions[4]
          << " " << this->lArmCmd.positions[5]
          << " " << this->lArmCmd.positions[6]
          << " " << this->lGripperCmd.data
          << std::endl;
      std::cout << "right arm " << std::endl;
      std::cout << " cmds: "
          << " " << this->rArmCmd.positions[0]
          << " " << this->rArmCmd.positions[1]
          << " " << this->rArmCmd.positions[2]
          << " " << this->rArmCmd.positions[3]
          << " " << this->rArmCmd.positions[4]
          << " " << this->rArmCmd.positions[5]
          << " " << this->rArmCmd.positions[6]
          << " " << this->rGripperCmd.data
          << std::endl;

    }

    void printCurrentEndEffectorWorldCoord() {
      libTF::TFPose aPose;
      aPose.x = 0.0;
      aPose.y = 0.0;
      aPose.z = 0.0;
      aPose.roll = 0;
      aPose.pitch = 0;
      aPose.yaw = 0;
      aPose.time = 0;
      aPose.frame = "gripper_roll_right";

      libTF::TFPose inOdomFrame = tf.transformPose("FRAMEID_ODOM", aPose);

      std::cout << "In odom frame x " << inOdomFrame.x << std::endl;
      std::cout << "In odom frame y " << inOdomFrame.y << std::endl;
      std::cout << "In odom frame z " << inOdomFrame.z << std::endl;
    }

    void printCurrentEndEffectorShoulderCoord() {
      libTF::TFPose aPose;
      aPose.x = .64;
      aPose.y = -.37;
      aPose.z = 1.76;
      aPose.roll = 0;
      aPose.pitch = 0;
      aPose.yaw = 0;
      aPose.time = 0;
      aPose.frame = "FRAMEID_ODOM";

      libTF::TFPose inOdomFrame = tf.transformPose("gripper_roll_right", aPose);

      std::cout << "In shoulder frame x " << inOdomFrame.x << std::endl;
      std::cout << "In shoulder frame y " << inOdomFrame.y << std::endl;
      std::cout << "In shoulder frame z " << inOdomFrame.z << std::endl;
    }

    void keyboardLoop();
    void changeJointAngle(PR2_JOINT_ID jointID, bool increment);
    void openGripper(PR2_JOINT_ID jointID);
    void closeGripper(PR2_JOINT_ID jointID);

    rosTFClient tf;
};

TArmK_Node* tarmk;
int kfd = 0;
struct termios cooked, raw;

  void
quit(int sig)
{
  //  tbk->stopRobot();
  ros::fini();
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

  int
main(int argc, char** argv)
{
  ros::init(argc,argv);

  tarmk = new TArmK_Node();

  signal(SIGINT,quit);

  tarmk->keyboardLoop();

  return(0);
}

void TArmK_Node::openGripper(PR2_JOINT_ID jointID) {
  if(jointID != ARM_R_GRIPPER_GAP && jointID != ARM_L_GRIPPER_GAP) return;
  if(jointID == ARM_R_GRIPPER_GAP) {
    this->rGripperCmd.data = .2;
    printf("Opening right gripper\n");
  } else { 
    this->lGripperCmd.data = .2;
    printf("Opening left gripper\n");
  }
}

void TArmK_Node::closeGripper(PR2_JOINT_ID jointID) {
  if(jointID != ARM_R_GRIPPER_GAP && jointID != ARM_L_GRIPPER_GAP) return;
  if(jointID == ARM_R_GRIPPER_GAP) {
    this->rGripperCmd.data = 0;
  } else { 
    this->lGripperCmd.data = 0;
  }
}


void TArmK_Node::changeJointAngle(PR2_JOINT_ID jointID, bool increment)
{
  float jointCmdStep = 5*M_PI/180;
  float gripperStep = 1*M_PI/180;
  if (increment == false)
  {
    jointCmdStep *= -1;
    gripperStep *= -1;
  }

  //this->lArmCmd.gripperForceCmd = 10; // FIXME: why is this getting reset to 0?
  //this->rArmCmd.gripperForceCmd = 10; // FIXME: why is this getting reset to 0?

  switch(jointID)
  {
    case ARM_L_PAN:
      this->lArmCmd.positions[0] += jointCmdStep;
      std::cout << " lp " << this->lArmCmd.positions[0] << std::endl;
      break;
    case ARM_L_SHOULDER_PITCH:
      this->lArmCmd.positions[1] += jointCmdStep;
      break;
    case ARM_L_SHOULDER_ROLL:
      this->lArmCmd.positions[2] += jointCmdStep;
      break;
    case ARM_L_ELBOW_PITCH:
      this->lArmCmd.positions[3] += jointCmdStep;
      break;
    case ARM_L_ELBOW_ROLL:
      this->lArmCmd.positions[4] += jointCmdStep;
      break;
    case ARM_L_WRIST_PITCH:
      this->lArmCmd.positions[5] += jointCmdStep;
      break;
    case ARM_L_WRIST_ROLL:
      this->lArmCmd.positions[6] += jointCmdStep;
      break;
    case ARM_L_GRIPPER_GAP:
      this->lGripperCmd.data += gripperStep;
      break;
    case ARM_R_PAN:
      this->rArmCmd.positions[0] += jointCmdStep;
      break;
    case ARM_R_SHOULDER_PITCH:
      this->rArmCmd.positions[1] += jointCmdStep;
      break;
    case ARM_R_SHOULDER_ROLL:
      this->rArmCmd.positions[2] += jointCmdStep;
      break;
    case ARM_R_ELBOW_PITCH:
      this->rArmCmd.positions[3] += jointCmdStep;
      break;
    case ARM_R_ELBOW_ROLL:
      this->rArmCmd.positions[4] += jointCmdStep;
      break;
    case ARM_R_WRIST_PITCH:
      this->rArmCmd.positions[5] += jointCmdStep;
      break;
    case ARM_R_WRIST_ROLL:
      this->rArmCmd.positions[6] += jointCmdStep;
      break;
    case ARM_R_GRIPPER_GAP:
      this->rGripperCmd.data += gripperStep;
      break;
    default:
      printf("This joint is not handled.\n");
      break;
  }
}


  void
TArmK_Node::keyboardLoop()
{
  char c;
  bool dirty=false;
  PR2_JOINT_ID curr_jointID = ARM_L_PAN; // joint which will be actuated.
  bool right_arm = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  printf("Press l/r to operate left/right arm.\n");
  printf("Numbers 1 through 8 to select the joint to operate.\n");
  printf("9 initializes teleop_arm_keyboard's robot state with the state of the robot in the simulation.\n");
  printf("+ and - will move the joint in different directions by 5 degrees.\n");
  puts("");
  puts("---------------------------");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
      case 'l':
      case 'L':
        right_arm = false;
        printf("Actuating left arm.\n");
        break;
      case 'r':
      case 'R':
        right_arm = true;
        printf("Actuating right arm.\n");
        break;
      case '+':
      case '=':
        changeJointAngle(curr_jointID, true);
        dirty=true;
        break;
      case '_':
      case '-':
        changeJointAngle(curr_jointID, false);
        dirty=true;
        break;
      case '.':
        openGripper(curr_jointID);
        dirty = true;
        break;
      case '/':
        sleep(1);
        closeGripper(curr_jointID);
        dirty = true;
        break;
      case 'q':
        printCurrentJointValues();
        break;
      case 'k':
        printCurrentEndEffectorWorldCoord();
        break;
      case 'j':
        printCurrentEndEffectorShoulderCoord();
        break;
      default:
        break;
    }

    if (right_arm==false)
    {
      switch(c)
      {
        case '1':
          curr_jointID = ARM_L_PAN;
          printf("left turret\n");
          break;
        case '2':
          curr_jointID = ARM_L_SHOULDER_PITCH;
          printf("left shoulder pitch\n");
          break;
        case '3':
          curr_jointID = ARM_L_SHOULDER_ROLL;
          printf("left shoulder roll\n");
          break;
        case '4':
          curr_jointID = ARM_L_ELBOW_PITCH;
          printf("left elbow pitch\n");
          break;
        case '5':
          curr_jointID = ARM_L_ELBOW_ROLL;
          printf("left elbow roll\n");
          break;
        case '6':
          curr_jointID = ARM_L_WRIST_PITCH;
          printf("left wrist pitch\n");
          break;
        case '7':
          curr_jointID = ARM_L_WRIST_ROLL;
          printf("left wrist roll\n");
          break;
        case '8':
          curr_jointID = ARM_L_GRIPPER_GAP;
          printf("left gripper\n");
          break;
        case '9':
          printf("Resetting left commands to current position.\n");
        default:
          break;
      }
    }
    else
    {
      switch(c)
      {
        case '1':
          curr_jointID = ARM_R_PAN;
          printf("right turret\n");
          break;
        case '2':
          curr_jointID = ARM_R_SHOULDER_PITCH;
          printf("right shoulder pitch\n");
          break;
        case '3':
          curr_jointID = ARM_R_SHOULDER_ROLL;
          printf("right shoulder roll\n");
          break;
        case '4':
          curr_jointID = ARM_R_ELBOW_PITCH;
          printf("right elbow pitch\n");
          break;
        case '5':
          curr_jointID = ARM_R_ELBOW_ROLL;
          printf("right elbow roll\n");
          break;
        case '6':
          curr_jointID = ARM_R_WRIST_PITCH;
          printf("right wrist pitch\n");
          break;
        case '7':
          curr_jointID = ARM_R_WRIST_ROLL;
          printf("right wrist roll\n");
          break;
        case '8':
          curr_jointID = ARM_R_GRIPPER_GAP;
          printf("right gripper\n");
          break;
        case '9':
          printf("Resetting right commands to current position.\n");
        default:
          break;
      }
    }

    if (dirty == true) {
      dirty=false; // Sending the command only once for each key press.
      if(!right_arm) {
        publish("left_arm_commands",lArmCmd);
        publish("gripper_left_controller/set_command",lGripperCmd);
      } else {
        publish("right_arm_commands",rArmCmd);
        publish("gripper_right_controller/set_command",rGripperCmd);
      }
    }
  }
}


