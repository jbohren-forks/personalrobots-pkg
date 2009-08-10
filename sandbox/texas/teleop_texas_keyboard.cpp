/*
 * teleop_texas
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

// Author: Kevin Watts

/**

@mainpage

@b teleop_texas_keyboard Teleops a DallasBot from a keyboard

<hr>

@section usage Usage
@verbatim
$ teleop_texas_keyboard [standard ROS args]
@endverbatim

Key mappings are printed to screen on startup. 

<hr>

@section topic ROS topics

Subscribes to (name / type):
- None

Publishes to (name / type):
- @b "texas/TexasCmd" : Command

<hr>

@section parameters ROS parameters

- None

 **/

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>

#include <texas/TexasCmd.h>

#define CMD_TOPIC "texas_cmd"

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 

class TeleopTexasKeyboard
{
  private:
  double angle_, velocity_;
  double max_vel_;

  ros::NodeHandle n_;
  ros::Publisher texas_pub_;

  public:
  void init()
  {
    angle_ = 0.0;
    velocity_ = 0.0;
 
    texas_pub_ = n_.advertise<texas::TexasCmd>(CMD_TOPIC, 1);
    
    // Max velocity of caster
    n_.param("~max_vel", max_vel_, 6.0);
    }
  
  ~TeleopTexasKeyboard()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_keyboard");

  TeleopTexasKeyboard thk;
  thk.init();

  signal(SIGINT,quit);

  thk.keyboardLoop();

  return(0);
}

void TeleopTexasKeyboard::keyboardLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to move");

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
    case KEYCODE_W:
      velocity_ += 1.0;
      dirty = true;
      break;
    case KEYCODE_S:
      velocity_ -= 1.0;
      dirty = true;
      break;
    case KEYCODE_A:
      angle_ += 0.5;
      dirty = true;
      break;
    case KEYCODE_D:
      angle_ -= 0.5;
      dirty = true;
      break;
    default:
      velocity_ = 0.0;
      dirty = true;
      break;
    }

    // Bound velocity
    velocity_ = std::max(std::min(velocity_, max_vel_), - max_vel_);

    if (dirty == true)
    {
      texas::TexasCmd cmd;
      cmd.angle = angle_;
      cmd.velocity = velocity_;

      texas_pub_.publish(cmd);
    }


  }
}
