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
 * The MapServer class reads an occupancy grid map from a bitmap image and
 * serves it up via ROS.
 * 
 * Author: Brian Gerkey
 */

#include <cstring>

#include <stdlib.h>
#include <stdio.h>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

#include "map_server/map_server.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

// Callback invoked when someone requests our service
bool MapServer::mapCallback(std_srvs::StaticMap::request  &req,
                            std_srvs::StaticMap::response &res )
{
  // request is empty; we ignore it

  // = operator is overloaded to make deep copy (tricky!)
  res = map_resp_;
  puts("sending map");
  return true;
}

// Read the image from file and fill out the map_resp_ object, for later use
// when our services are requested.
bool
MapServer::loadMapFromFile(const char* fname, double res, bool negate)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  int rowstride, n_channels;
  unsigned int i,j;
  int k;
  double occ;
  int color_sum;
  double color_avg;

  printf("loading image file: %s...", fname);
  fflush(stdout);

  if(!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") + 
            std::string(fname) + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  map_resp_.map.width = img->w;
  map_resp_.map.height = img->h;
  map_resp_.map.resolution = res;
  // TODO: make origin configurable
  map_resp_.map.origin.x = 0.0;
  map_resp_.map.origin.y = 0.0;
  map_resp_.map.origin.th = 0.0;

  map_resp_.map.set_data_size(map_resp_.map.width * map_resp_.map.height);

  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // Read data
  pixels = (unsigned char*)(img->pixels);
  for(j = 0; j < map_resp_.map.height; j++)
  {
    for (i = 0; i < map_resp_.map.width; i++)
    {
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)n_channels;

      if(negate)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      if(occ > 0.5)
        map_resp_.map.data[MAP_IDX(map_resp_.map.width,i,map_resp_.map.height - j - 1)] = +100;
      else if(occ < 0.1)
        map_resp_.map.data[MAP_IDX(map_resp_.map.width,i,map_resp_.map.height - j - 1)] = 0;
      else
        map_resp_.map.data[MAP_IDX(map_resp_.map.width,i,map_resp_.map.height - j - 1)] = -1;
    }
  }

  SDL_FreeSurface(img);

  puts("Done.");
  printf("read a %d X %d map\n", map_resp_.map.width, map_resp_.map.height);

  advertise_service("static_map", &MapServer::mapCallback);

  return(true);
}
