/*
 * map_generator
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


#include "ros/node.h"
#include "std_srvs/StaticMap.h"

using namespace std;
 
class MapGenerator : public ros::node {
public:
  MapGenerator(string servname);
  ~MapGenerator();
};

MapGenerator::MapGenerator(string servname) : ros::node("map_generator") {
  puts("Requesting the map...");
  std_srvs::StaticMap::request  req;
  std_srvs::StaticMap::response resp;
  while(!ros::service::call("static_map", req, resp))
  {
    printf("request '%s' failed; trying again...", servname.c_str());
    usleep(1000000);
  }
  printf("Received a %d X %d map @ %.3f m/pix\n",
         resp.map.width,
         resp.map.height,
         resp.map.resolution);


  FILE* out = fopen("map.pgm", "w");

  fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
	  resp.map.resolution, resp.map.width, resp.map.height);
  for(unsigned int y = 0; y < resp.map.height; y++) {
    for(unsigned int x = 0; x < resp.map.width; x++) {
      unsigned int i = x + (resp.map.height - y - 1) * resp.map.width;
      if (resp.map.data[i] == 0) { //occ [0,0.1)
	fputc(255, out);
      } else if (resp.map.data[i] == +100) { //occ (0.65,1]
	fputc(000, out);
      } else { //occ [0.1,0.65]
	fputc(206, out);
      }
    }
  }

  fclose(out);

  printf("Done\n");
  
}


MapGenerator::~MapGenerator() {
}

int main(int argc, char** argv) {
  if (argc != 2) {
    printf("Usage: %s <service>\n<service> is the name of the map service: static_map or dynamic_map.\n", argv[0]);
    return -1;
  }
  ros::init(argc, argv);
  MapGenerator mg(argv[1]);
  
  while(mg.ok()) {
    usleep(100000);
  }


  ros::fini();

  return 0;
}


