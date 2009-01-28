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

@b teleop_base_keyboard teleoperates a differential-drive robot by mapping
key presses into velocity commands.  Consider it a poor man's joystick.

<hr>

@section usage Usage
@verbatim
$ teleop_base_keyboard [standard ROS args]
@endverbatim

Key mappings are printed to screen on startup.  Press any unmapped key to
stop the robot.

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "cmd_vel"/BaseVel : velocity to the robot; sent on every keypress.

<hr>

@section parameters ROS parameters

- None

 **/

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdlib.h>

#include <ros/node.h>
#include <std_msgs/BaseVel.h>

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7a
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_E 0x65
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6d
#define KEYCODE_R 0x72
#define KEYCODE_V 0x76
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62

#define KEYCODE_COMMA 0x2c
#define KEYCODE_PERIOD 0x2e

#define COMMAND_TIMEOUT_SEC 0.2

// at full joystick depression you'll go this fast
double max_speed = 0.500; // m/second
double max_turn = 60.0*M_PI/180.0; // rad/second
// should we continuously send commands?
bool always_command = false;


class TBK_Node : public ros::Node
{
  private:
    std_msgs::BaseVel cmdvel;

  public:
    TBK_Node() : ros::Node("tbk")
    {
      advertise<std_msgs::BaseVel>("cmd_vel",1);
    }
    ~TBK_Node() { }
    void keyboardLoop();
    void stopRobot()
    {
      cmdvel.vx = cmdvel.vw = 0.0;
      publish("cmd_vel", cmdvel);
    }
};

TBK_Node* tbk;
int kfd = 0;
struct termios cooked, raw;

void
quit(int sig)
{
  tbk->stopRobot();
  ros::fini();
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int
main(int argc, char** argv)
{
  ros::init(argc,argv);

  tbk = new TBK_Node();

  signal(SIGINT,quit);

  tbk->keyboardLoop();

  return(0);
}

void
TBK_Node::keyboardLoop()
{
  char c;
  double max_tv = max_speed;
  double max_rv = max_turn;
  bool dirty=false;

  int speed=0;
  int turn=0;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Moving around:");
  puts("   u    i    o");
  puts("   j    k    l");
  puts("   m    ,    .");
  puts("");
  puts("q/z : increase/decrease max speeds by 10%");
  puts("w/x : increase/decrease only linear speed by 10%");
  puts("e/c : increase/decrease only angular speed by 10%");
  puts("");
  puts("anything else : stop");
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
      case KEYCODE_I:
        speed = 1;
        turn = 0;
        dirty = true;
        break;
      case KEYCODE_K:
        speed = 0;
        turn = 0;
        dirty = true;
        break;
      case KEYCODE_O:
        speed = 1;
        turn = -1;
        dirty = true;
        break;
      case KEYCODE_J:
        speed = 0;
        turn = 1;
        dirty = true;
        break;
      case KEYCODE_L:
        speed = 0;
        turn = -1;
        dirty = true;
        break;
      case KEYCODE_U:
        turn = 1;
        speed = 1;
        dirty = true;
        break;
      case KEYCODE_COMMA:
        turn = 0;
        speed = -1;
        dirty = true;
        break;
      case KEYCODE_PERIOD:
        turn = 1;
        speed = -1;
        dirty = true;
        break;
      case KEYCODE_M:
        turn = -1;
        speed = -1;
        dirty = true;
        break;
      case KEYCODE_Q:
        max_tv += max_tv / 10.0;
        max_rv += max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_Z:
        max_tv -= max_tv / 10.0;
        max_rv -= max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_W:
        max_tv += max_tv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_X:
        max_tv -= max_tv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_E:
        max_rv += max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
    case KEYCODE_C:
        max_rv -= max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
    default:
      speed = 0;
      turn = 0;
      dirty = true;
    }
    if (dirty == true)
    {
      cmdvel.vx = speed * max_tv;
      cmdvel.vw = turn * max_rv;

      publish("cmd_vel",cmdvel);
    }
  }
}
