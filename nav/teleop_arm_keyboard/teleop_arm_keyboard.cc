/*
 * teleop_base_keyboard
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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
- @b "cmd_leftarmconfig"/PR2Arm : configuration of the left arm (all the joint angles); sent on every keypress.

<hr>

@section parameters ROS parameters

- None

 **/

#include <termios.h>
#include <signal.h>
#include <math.h>

#include <ros/node.h>
#include <std_msgs/MsgPR2Arm.h>

#include <pr2API.hh>

#define COMMAND_TIMEOUT_SEC 0.2

using namespace PR2;


class TArmK_Node : public ros::node
{
  private:
    MsgPR2Arm cmd_leftarmconfig;

  public:
    TArmK_Node() : ros::node("tarmk")
    {
			// cmd_armconfig should probably be initialised
			// with the current joint angles of the arm rather
			// than zeros.
			this->cmd_leftarmconfig.turretAngle = 0;
			this->cmd_leftarmconfig.shoulderLiftAngle = 0;
			this->cmd_leftarmconfig.upperarmRollAngle = 0;
			this->cmd_leftarmconfig.elbowAngle        = 0;
			this->cmd_leftarmconfig.forearmRollAngle  = 0;
			this->cmd_leftarmconfig.wristPitchAngle   = 0;
			this->cmd_leftarmconfig.wristRollAngle    = 0;
			this->cmd_leftarmconfig.gripperForceCmd   = 0;
			this->cmd_leftarmconfig.gripperGapCmd     = 0;
      advertise<MsgPR2Arm>("cmd_leftarmconfig");
      advertise<MsgPR2Arm>("cmd_armconfig_2");
    }
    ~TArmK_Node() { }
    void keyboardLoop();
		void changeJointAngle(PR2_JOINT_ID jointID, bool increment);
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


void TArmK_Node::changeJointAngle(PR2_JOINT_ID jointID, bool increment)
{
	float jointCmdStep = 5*M_PI/180;
	if (increment == false)
		jointCmdStep *= -1;

	switch(jointID)
	{
		case ARM_L_PAN:
			this->cmd_leftarmconfig.turretAngle += jointCmdStep;
			break;
		case ARM_L_SHOULDER_PITCH:
			this->cmd_leftarmconfig.shoulderLiftAngle += jointCmdStep;
			break;
		case ARM_L_SHOULDER_ROLL:
			this->cmd_leftarmconfig.upperarmRollAngle += jointCmdStep;
			break;
		case ARM_L_ELBOW_PITCH:
			this->cmd_leftarmconfig.elbowAngle += jointCmdStep;
			break;
		case ARM_L_ELBOW_ROLL:
			this->cmd_leftarmconfig.forearmRollAngle += jointCmdStep;
			break;
		case ARM_L_WRIST_PITCH:
			this->cmd_leftarmconfig.wristPitchAngle += jointCmdStep;
			break;
		case ARM_L_WRIST_ROLL:
			this->cmd_leftarmconfig.wristRollAngle += jointCmdStep;
			break;
		case ARM_L_GRIPPER:
			this->cmd_leftarmconfig.gripperGapCmd += jointCmdStep;
			break;
		case ARM_R_PAN:
			break;
		case ARM_R_SHOULDER_PITCH:
			break;
		case ARM_R_SHOULDER_ROLL:
			break;
		case ARM_R_ELBOW_PITCH:
			break;
		case ARM_R_ELBOW_ROLL:
			break;
		case ARM_R_WRIST_PITCH:
			break;
		case ARM_R_WRIST_ROLL:
			break;
		case ARM_R_GRIPPER:
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

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
	printf("Numbers 1 through 8 to select the joint to operate.\n");
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
				curr_jointID = ARM_L_GRIPPER;
				printf("left gripper\n");
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

			default:
				printf("Something else\n");
				break;
		}

    if (dirty == true)
    {
			dirty=false; // Sending the command only once for each key press.
      publish("cmd_leftarmconfig",cmd_leftarmconfig);
      publish("cmd_armconfig_2",cmd_leftarmconfig);
    }


  }
}
