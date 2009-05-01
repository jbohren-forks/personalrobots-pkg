/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <ros/node.h>
#include "ledwiz/LedState.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("set_led");

  ledwiz::LedState outmsg;

  if (argc < 4 || argc > 5)
  {
    fprintf(stderr, "usage: set_led red green blue [pulse_speed]\n");
    fprintf(stderr, "- red, green and blue are from 0 to 48, or\n");
    fprintf(stderr, "- 129 for sawtooth, 130 for blink, 131 for downramp, 132 for upramp\n");
    fprintf(stderr, "  pulse_speed is from 0 to 7\n");
    return 1;
  }

  outmsg.r = atoi(argv[1]);
  outmsg.g = atoi(argv[2]);
  outmsg.b = atoi(argv[3]);
  outmsg.rate = argc > 4 ? atoi(argv[4]) : 0;
  n.advertise<ledwiz::LedState>("ledwiz/ledstate", 100);
  sleep(1);
  n.publish("ledwiz/ledstate", outmsg);
  sleep(1);
  return 0;
}

