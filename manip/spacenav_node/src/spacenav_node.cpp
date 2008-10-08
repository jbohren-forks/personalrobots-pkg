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

/*
 * Author: Stuart Glaser
 */

#include <stdio.h>
#include "ros/node.h"
#include "spnav.h"
#include "std_msgs/Vector3.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  ros::node node("spacenav");
  node.advertise<std_msgs::Vector3>("/spacenav/offset", 16);

  if (spnav_open() == -1)
  {
    fprintf(stderr, "Error: Could not open the space navigator device\n");
    fprintf(stderr, "Did you remember to run spacenavd (as root)?\n");
    ros::fini();
    return 1;
  }

  spnav_event sev;
  int ret;
  int no_motion_count = 0;
  while (true)
  {
    ret = spnav_poll_event(&sev);
    spnav_remove_events(SPNAV_EVENT_MOTION);

    if (ret == 0)
    {
      if (++no_motion_count > 30)
      {
        std_msgs::Vector3 offset_msg;
        offset_msg.x = offset_msg.y = offset_msg.z = 0;
        node.publish("/spacenav/offset", offset_msg);
      }
    }
    if (sev.type == SPNAV_EVENT_MOTION)
    {
      std_msgs::Vector3 offset_msg;
      offset_msg.x = sev.motion.z;
      offset_msg.y = -sev.motion.x;
      offset_msg.z = sev.motion.y;
      node.publish("/spacenav/offset", offset_msg);
      no_motion_count = 0;
    }
    else if (sev.type == SPNAV_EVENT_BUTTON)
    {
    }
    usleep(1000);
  }

  ros::fini();

  return 0;
}
