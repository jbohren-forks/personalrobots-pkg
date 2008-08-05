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
 * The map_server node reads an occupancy grid map from a bitmap image and
 * serves it up via ROS.
 * 
 * Author: Brian Gerkey
 */


#define USAGE "USAGE: map_server <map> <resolution> [<negate>]\n"\
              "         map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]\n"\
              "      negate: if non-zero, black is free, white is occupied"

#include <stdio.h>
#include <stdlib.h>

#include "ros/node.h"
#include "map_server/map_server.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if(argc < 3)
  {
    puts(USAGE);
    exit(-1);
  }
  const char* fname = argv[1];
  double res = atof(argv[2]);
  bool negate = false;
  if(argc > 3)
    negate = atoi(argv[3]) ? true : false;

  MapServer ms;
  try
  {
    ms.loadMapFromFile(fname,res,negate);
    ms.spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  ros::fini();
  return 0;
}

