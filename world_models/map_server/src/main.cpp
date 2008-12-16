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

/* Author: Brian Gerkey */

/**

@mainpage

@htmlinclude manifest.html

@b map_server is a ROS node that reads an occupancy grid map from an image
file and offers up the map via a ROS service.

When parsing the image, a pixel is considered to be free (occupancy
value 0) if the mean of its RGB values is greater than 0.9*255 (i.e.,
very white).  The pixel is occupied (occupancy value 100) if the mean
is less than 0.5*255 (i.e., sort of black).  Anything in between is
considered unknown (occupancy value -1).

@todo Establish a standard for storing maps, with metadata (origin,
resolution, color thresholds, etc.) in the same file.  Perhaps we can use
PNGs with comments for this purpose. Then rewrite this node to use said
standard.

<hr>

@section usage Usage
@verbatim
map_server <map> <resolution> [<negate>]
           map: image file to load
           resolution: map resolution [meters/pixel]
           negate: if non-zero, black is free, white is occupied
@endverbatim

@par Example

@verbatim
map_server mymap.png 0.1
@endverbatim

<hr>

@section topic ROS topics

- None

@section services ROS services

Offers (name/type):
- @b "static_map"/std_srvs::StaticMap : Retrieve the map via this service

@section parameters ROS parameters

- None
*/



#define USAGE "USAGE: map_server <map> <resolution> [<negate>]\n"\
              "         map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]\n"\
              "      negate: if non-zero, black is free, white is occupied"

#include <stdio.h>
#include <stdlib.h>

#include "ros/node.h"
#include "ros/publisher.h"
#include "map_server/image_loader.h"
#include "std_msgs/MapMetaData.h"

class MapServer : public ros::node
{
  public:
    /** Trivial constructor */
    MapServer() : ros::node("map_server") {}

    /** Callback invoked when someone requests our service */
    bool mapCallback(std_srvs::StaticMap::request  &req,
                     std_srvs::StaticMap::response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      puts("sending map");
      return true;
    }

    /** The map response is cached here, to be sent out to service callers
     */
    std_srvs::StaticMap::response map_resp_;

    void metadataSubscriptionCallback(const ros::PublisherPtr& pub)
    {
      publish( "map_metadata", meta_data_message_ );
    }

    std_msgs::MapMetaData meta_data_message_;
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

  MapServer ms;
  try
  {
    printf("[map_server] loading map from image \"%s\"\n", fname);
    map_server::loadMapFromFile(&ms.map_resp_,fname,res,negate);
    printf("[map_server] read a %d X %d map @ %.3lf m/cell\n",
           ms.map_resp_.map.width,
           ms.map_resp_.map.height,
           ms.map_resp_.map.resolution);

    ms.meta_data_message_.map_load_time = ros::Time::now();
    ms.meta_data_message_.resolution = ms.map_resp_.map.resolution;
    ms.meta_data_message_.width = ms.map_resp_.map.width;
    ms.meta_data_message_.height = ms.map_resp_.map.height;
    ms.meta_data_message_.origin = ms.map_resp_.map.origin;
    ms.advertise_service("static_map", &MapServer::mapCallback);
    ms.advertise("map_metadata", ms.meta_data_message_, &MapServer::metadataSubscriptionCallback, 1);

    ms.spin();
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  ros::fini();
  return 0;
}

