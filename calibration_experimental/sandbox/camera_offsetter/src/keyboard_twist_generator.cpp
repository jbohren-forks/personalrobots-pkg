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

//! \author Vijay Pradeep

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <unistd.h>
#include <termios.h>

int main(int argc, char** argv)
{
  // Setup terminal settings for getchar
  const int fd = fileno(stdin);
  termios prev_flags ;
  tcgetattr(fd, &prev_flags) ;
  termios flags ;
  tcgetattr(fd,&flags);
  flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
  flags.c_cc[VMIN]  = 0;     // i.e. min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;     // block if waiting for char
  tcsetattr(fd,TCSANOW,&flags);

  ros::init(argc, argv, "keyboard_twist_generator");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("keyboard_twist", 10);

  static const double trans = .001;
  static const double rot   = .1 * 3.14 / 180.0;


  enum ShiftMode { ROTATION=0, TRANSLATION=1 };

  ShiftMode cur_shift_mode = TRANSLATION;

  while (nh.ok())
  {
    char c = getchar();

    geometry_msgs::Twist twist;

    geometry_msgs::Vector3* vec = NULL;
    double shift_val;
    switch (cur_shift_mode)
    {
      case ROTATION:
        vec = &twist.angular;
        shift_val = rot;
        break;
      case TRANSLATION:
        vec = &twist.linear;
        shift_val = trans;
        break;
      default:
        shift_val = 0.0;
        ROS_FATAL("unknown shift type [%u]", cur_shift_mode);
        break;
    }

    bool should_publish = false;

    switch (c)
    {
      case 'x': vec->x = -shift_val;  should_publish = true; break;
      case 'X': vec->x =  shift_val;  should_publish = true; break;
      case 'y': vec->y = -shift_val;  should_publish = true; break;
      case 'Y': vec->y =  shift_val;  should_publish = true; break;
      case 'z': vec->z = -shift_val;  should_publish = true; break;
      case 'Z': vec->z =  shift_val;  should_publish = true; break;
      case 't':
      case 'T':
        ROS_INFO("Switching to translation mode");
        cur_shift_mode = TRANSLATION;
        break;
      case 'r':
      case 'R':
        ROS_INFO("Switching to rotation mode");
        cur_shift_mode = ROTATION;
        break;
      default:
        usleep(100);
        break;
    }

    if (should_publish)
    {
      ROS_INFO("Publishing Twist: Translation(%.3f, %.3f, %.3f)    Rotation(%.3f, %.3f, %.3f)",
               twist.linear.x,  twist.linear.y,  twist.linear.z,
               twist.angular.x, twist.angular.y, twist.angular.z);
      pub.publish(twist);
    }
  }

  tcsetattr(fd,TCSANOW, &prev_flags) ;         // Undo any terminal changes that we made

  return 0;
}
