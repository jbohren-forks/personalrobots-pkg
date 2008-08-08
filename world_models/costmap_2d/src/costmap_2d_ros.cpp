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
 *
 * Author: E. Gil Jones
 */

// roscpp
#include <ros/node.h>

// our ros messages
#include <std_msgs/LaserScan.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_srvs/StaticMap.h>

//Laser projection
#include "laser_scan_utils/laser_scan.h"

// For time support
#include <ros/time.h>

// For transform support
#include <rosTF/rosTF.h>

//for map drawing
#include <gdk-pixbuf/gdk-pixbuf.h>

//costmap_2d
#include "costmap_2d/costmap_2d.h" 

// For GUI debug
#include <std_msgs/Polyline2D.h>

//window length for remembering laser data (seconds)
static const double WINDOW_LENGTH = 1.0;

//laser range value - hits beyond this get discard
static const double LASER_MAX_RANGE = 10.0;

class CostMap2DRos: public ros::node
{

public:

  CostMap2DRos();

  ~CostMap2DRos();

  rosTFClient tf_;

private:
  
  void drawCostMap(); 

  void publishMapData();

  //callback for laser data
  void laserReceived();

  //costmap
  CostMap2D costmap_;

  //laser scan message
  std_msgs::LaserScan laser_msg_;
  std_msgs::Polyline2D pointcloud_msg_;

  //projector for the laser
  laser_scan::LaserProjection projector_;

};


CostMap2DRos::CostMap2DRos() :
  ros::node("costmap2d_ros"),
  tf_(*this, true),
  costmap_(WINDOW_LENGTH)
{
  std_srvs::StaticMap::request  req;
  std_srvs::StaticMap::response resp;
  puts("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    puts("request failed; trying again...");
    usleep(1000000);
  }
  printf("Received a %d X %d map @ %.3f m/pix\n",
         resp.map.width,
         resp.map.height,
         resp.map.resolution);
  int sx, sy;
  sx = resp.map.width;
  sy = resp.map.height;
  // Convert to player format
  unsigned char* mapdata = new unsigned char[sx*sy];
  for(int i=0;i<sx*sy;i++)
  {
    if(resp.map.data[i] == 0)
      mapdata[i] = -1;
    else if(resp.map.data[i] == 100)
      mapdata[i] = 100;
    else
      mapdata[i] = 0;
  }
  costmap_.setStaticMap(sx, sy, resp.map.resolution, mapdata);
  delete[] mapdata;
  
  tf_.setWithEulers(tf_.lookup("FRAMEID_LASER"),
                    tf_.lookup("FRAMEID_ROBOT"),
                    0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
  
  advertise<std_msgs::Polyline2D>("gui_laser");
  subscribe("scan", laser_msg_, &CostMap2DRos::laserReceived);
  
}
  
CostMap2DRos::~CostMap2DRos() {
}

void CostMap2DRos::laserReceived() {
  
  std_msgs::PointCloudFloat32 local_cloud;
  projector_.projectLaser(laser_msg_, local_cloud, LASER_MAX_RANGE);
  
  // Convert to a point cloud in the map frame
  std_msgs::PointCloudFloat32 global_cloud;
  
  try
  {
    global_cloud = tf_.transformPointCloud("FRAMEID_MAP", local_cloud);
  }
  catch(libTF::TransformReference::LookupException& ex)
  {
    puts("no global->local Tx yet");
    return;
  }
  catch(libTF::TransformReference::ExtrapolateException& ex)
  {
    puts("extrapolation required");
    return;
  }

  //  libTF::TFPose aPose;
  //   aPose.x = 0.0;
  //   aPose.y = 0.0;
  //   aPose.z = 0.0;
  //   aPose.roll = 0;
  //   aPose.pitch = 0;
  //   aPose.yaw = 0;
  //   aPose.time = 0;
  //   aPose.frame = tf.lookup("FRAMEID_ODOM");
  
  //   libTF::TFPose inMapFrame = tf.transformPose("FRAMEID_MAP", aPose);
  
  //std::cout << "Map frame x " << inMapFrame.x << std::endl;
  //std::cout << "Map frame y " << inMapFrame.y << std::endl;

  costmap_.addObstacles(&global_cloud);

  publishMapData();
 
  //drawCostMap();
}

void CostMap2DRos::publishMapData()
{
  size_t height = costmap_.getHeight();
  size_t width = costmap_.getWidth();
  const unsigned char* mapdata = costmap_.getMap();
  unsigned int tot_points = costmap_.getTotalObsPoints();

  pointcloud_msg_.set_points_size(tot_points);
  pointcloud_msg_.color.a = 0.0;
  pointcloud_msg_.color.r = 0.0;
  pointcloud_msg_.color.b = 0.0;
  pointcloud_msg_.color.g = 1.0;

  unsigned int pointcloud_ind = 0;
  for(size_t i = 0; i < width; i++)
  {
    for(size_t j = 0; j < height; j++) 
    {
      size_t ind = costmap_.getMapIndex(i,j);
      if(mapdata[ind] == 75) {
        double wx, wy;
        costmap_.convertFromIndexesToWorldCoord(i,j,wx,wy);
        if(pointcloud_ind > tot_points) {
          std::cerr << "CostMap2DRos::publishMapData - too many obstacle points.\n";
        } else {
          pointcloud_msg_.points[pointcloud_ind].x = wx;
          pointcloud_msg_.points[pointcloud_ind].y = wy;
          pointcloud_ind++;
        }
      }
    }
  }
  publish("gui_laser",pointcloud_msg_);
}

void CostMap2DRos::drawCostMap() 
{
  GdkPixbuf* pixbuf;
  GError* error = NULL;
  guchar* pixels;
  int p;
  int paddr;

  // Initialize glib
  g_type_init();

  size_t height = costmap_.getHeight();
  size_t width = costmap_.getWidth();

  pixels = (guchar*)malloc(sizeof(guchar)*width*height*3);

  const unsigned char* mapdata = costmap_.getMap();

  //this draws the obstacles
  p=0;
  for(int j=(int)(height-1);j>=0;j--) 
  {
    for(int i=0; i< (int)width; i++,p++) 
    {
      paddr = p * 3;
      //std::cout << "Map index for " << i << " " << j << " is " << costmap_.getMapIndex(i,j) << std::endl;
      if(mapdata[costmap_.getMapIndex(i,j)] == 100) {
        pixels[paddr] = 0;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 0;
      } else if(mapdata[costmap_.getMapIndex(i,j)] == 75) {
        pixels[paddr] = 0;
        pixels[paddr+1] = 255;
        pixels[paddr+2] = 0;
      } else {
        pixels[paddr] = 255;
        pixels[paddr+1] = 255;
        pixels[paddr+2] = 255;
      }
    }
  }

  pixbuf = gdk_pixbuf_new_from_data(pixels, 
                                    GDK_COLORSPACE_RGB,
                                    0,8,
                                    width,
                                    height,
                                    width * 3,
                                    NULL, NULL);
    
  gdk_pixbuf_save(pixbuf,"costmap.png","png",&error,NULL);
  gdk_pixbuf_unref(pixbuf);
  free(pixels);
}

int main(int argc, char **argv) {

  ros::init(argc, argv);
  
  CostMap2DRos cmr;

  cmr.spin();
  
  cmr.shutdown();
}
