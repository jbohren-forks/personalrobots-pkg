///////////////////////////////////////////////////////////////////////////////
// The map_server package reads an occ grid map from a bitmap file and 
// serves it up
//
// Copyright (C) 2008, Willow Garage
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#define USAGE "USAGE: map_server <map> <resolution> [<negate>]\n"\
              "         map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]\n"\
              "      negate: if non-zero, black is free, white is occupied"

#include <stdlib.h>
#include <stdio.h>

// We use gdk-pixbuf to load the image from disk
#include <gdk-pixbuf/gdk-pixbuf.h>

#include "ros/node.h"
#include "std_srvs/StaticMap.h"

class MapServer : public ros::node
{
  private:
    std_srvs::StaticMap::response map_resp;

  public:
    MapServer(const char* fname, double res, bool negate) : 
            ros::node("map_server")
    {
      if(!loadMapFromFile(fname,res,negate))
      {
        printf("failed to load map \"%s\"\n", fname);
        ros::fini();
        exit(-1);
      }
      advertise_service("static_map", &MapServer::map_cb);
    }

    // Callback invoked when someone requests our service
    bool map_cb(std_srvs::StaticMap::request  &req,
                std_srvs::StaticMap::response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp;
      puts("sending map");
      return true;
    }
    // Read the image from file and fill out the map_resp object, 
    // for later use when our services are requested.
    bool loadMapFromFile(const char* fname, double res, bool negate);
};

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
  MapServer ms(fname,res,negate);
  ms.spin();
  ros::fini();
  return 0;
}


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

// Read the image from file and fill out the map_resp object, for later use
// when our services are requested.
bool
MapServer::loadMapFromFile(const char* fname, double res, bool negate)
{
  GdkPixbuf* pixbuf;
  guchar* pixels;
  guchar* p;
  int rowstride, n_channels, bps;
  GError* error = NULL;
  unsigned int i,j;
  int k;
  double occ;
  int color_sum;
  double color_avg;

  // Initialize glib
  g_type_init();

  printf("loading image file: %s...", fname);
  fflush(stdout);

  // Read the image
  if(!(pixbuf = gdk_pixbuf_new_from_file(fname, &error)))
  {
    printf("failed to open image file %s", fname);
    return(false);
  }

  map_resp.map.width = gdk_pixbuf_get_width(pixbuf);
  map_resp.map.height = gdk_pixbuf_get_height(pixbuf);
  map_resp.map.resolution = res;
  // TODO: make origin configurable
  map_resp.map.origin.x = 0.0;
  map_resp.map.origin.y = 0.0;
  map_resp.map.origin.th = 0.0;

  map_resp.map.set_data_size(map_resp.map.width * map_resp.map.height);

  rowstride = gdk_pixbuf_get_rowstride(pixbuf);
  bps = gdk_pixbuf_get_bits_per_sample(pixbuf)/8;
  n_channels = gdk_pixbuf_get_n_channels(pixbuf);
  // This seems to be broken somehow
  //if(gdk_pixbuf_get_has_alpha(pixbuf))
    //n_channels++;

  // Read data
  pixels = gdk_pixbuf_get_pixels(pixbuf);
  for(j = 0; j < map_resp.map.height; j++)
  {
    for (i = 0; i < map_resp.map.width; i++)
    {
      p = pixels + j*rowstride + i*n_channels*bps;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k * bps));
      color_avg = color_sum / (double)n_channels;

      if(negate)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      if(occ > 0.5)
        map_resp.map.data[MAP_IDX(map_resp.map.width,i,map_resp.map.height - j - 1)] = +100;
      else if(occ < 0.1)
        map_resp.map.data[MAP_IDX(map_resp.map.width,i,map_resp.map.height - j - 1)] = 0;
      else
        map_resp.map.data[MAP_IDX(map_resp.map.width,i,map_resp.map.height - j - 1)] = -1;
    }
  }

  gdk_pixbuf_unref(pixbuf);

  puts("Done.");
  printf("read a %d X %d map\n", map_resp.map.width, map_resp.map.height);
  return(true);
}
