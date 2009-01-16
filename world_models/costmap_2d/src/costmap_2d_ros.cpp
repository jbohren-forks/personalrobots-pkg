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
#include <std_msgs/PointCloud.h>
#include <pr2_msgs/OccDiff.h>
#include <std_srvs/StaticMap.h>
#include <pr2_srvs/TransientObstacles.h>

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
static const double LASER_MAX_RANGE = 4.0;

class CostMap2DRos: public ros::Node
{

public:

  CostMap2DRos();

  ~CostMap2DRos();

  rosTFClient tf_;

  /** Callback invoked for getting the full list of obstacle
      points plus a time stamp value
   */
  bool fullTransientObstacleCallback(pr2_srvs::TransientObstacles::request  &req,
                                     pr2_srvs::TransientObstacles::response &res);

  
private:
  
  void drawCostMap(); 

  void publishMapData();

  //callback for laser data
  void laserReceived();
  
  void publishDiffData(const ros::Time ts,
                       const std::list<unsigned int>& occ_ids,
                       const std::list<unsigned int>& unocc_ids);

  //costmap
  CostMap2D costmap_;

  //laser scan message
  std_msgs::LaserScan laser_msg_;
  std_msgs::Polyline2D pointcloud_msg_;

  //projector for the laser
  laser_scan::LaserProjection projector_;

  std::list<std_msgs::LaserScan> buffered_laser_scans;

};


CostMap2DRos::CostMap2DRos() :
  ros::Node("costmap2d_ros"),
  tf_(*this, true, 1 * 1000000000ULL, 0ULL),
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
  
  tf_.setWithEulers("FRAMEID_LASER",
                    "FRAMEID_ROBOT",
                    0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
  
  advertise<pr2_msgs::OccDiff>("transient_obstacles_diff");
  advertise_service("transient_obstacles_full", &CostMap2DRos::fullTransientObstacleCallback);
  subscribe("scan", laser_msg_, &CostMap2DRos::laserReceived);
  
}
  
CostMap2DRos::~CostMap2DRos() {
}

bool CostMap2DRos::fullTransientObstacleCallback(pr2_srvs::TransientObstacles::request  &req,
                                                 pr2_srvs::TransientObstacles::response &res)
{
  //request is empty, so we ignore it
  
  std::list<unsigned int> obstacle_indexes = costmap_.getOccupiedCellDataIndexList();
  
  res.obs.set_points_size(obstacle_indexes.size());
  res.obs.color.a = 0.0;
  res.obs.color.r = 0.0;
  res.obs.color.b = 0.0;
  res.obs.color.g = 1.0;

  unsigned int pointcloud_ind = 0;

  for(std::list<unsigned int>::iterator i = obstacle_indexes.begin();
      i != obstacle_indexes.end();
      i++) {
    unsigned int mx, my;
    double wx, wy;
    costmap_.convertFromMapIndexToXY((*i), mx, my);
    costmap_.convertFromIndexesToWorldCoord(mx, my, wx, wy);
    res.obs.points[pointcloud_ind].x = wx;
    res.obs.points[pointcloud_ind].y = wy;
    pointcloud_ind++;
  }

  return true;
}

void CostMap2DRos::laserReceived() {
  
  std_msgs::LaserScan newscan(laser_msg_);
  this->buffered_laser_scans.push_back(newscan);

  std::list<unsigned int> new_occ_ids;
  std::list<unsigned int> new_unocc_ids;

  for(std::list<std_msgs::LaserScan>::iterator it = this->buffered_laser_scans.begin();
      it != this->buffered_laser_scans.end();
      it++)
  {

    std_msgs::PointCloud local_cloud;
    projector_.projectLaser((*it), local_cloud, LASER_MAX_RANGE);
    
    // Convert to a point cloud in the map frame
    std_msgs::PointCloud global_cloud;
    
    try
    {
      global_cloud = tf_.transformPointCloud("FRAMEID_MAP", local_cloud);
    }
    catch(libTF::TransformReference::LookupException& ex)
    {
      puts("no global->local Tx yet");
      continue;
    }
    catch(libTF::TransformReference::ExtrapolateException& ex)
    {
      //puts("extrapolation required");
      continue;
    }  catch(libTF::TransformReference::ConnectivityException& ce) {
      puts("no transform parent for frameid_map");
      continue;
    }

    costmap_.addObstacles(&global_cloud, new_occ_ids, new_unocc_ids);
    it = this->buffered_laser_scans.erase(it);
    it--;
  }

  publishDiffData(laser_msg_.header.stamp, new_occ_ids, new_unocc_ids);
 
  //drawCostMap();
}

void CostMap2DRos::publishDiffData(const ros::Time ts,
                                   const std::list<unsigned int>& occ_ids,
                                   const std::list<unsigned int>& unocc_ids)
{
  pr2_msgs::OccDiff occ_diff;
  occ_diff.header.stamp = ts;
  occ_diff.set_occ_points_size(occ_ids.size());
  occ_diff.set_unocc_points_size(unocc_ids.size());

  size_t i = 0;
  for(std::list<unsigned int>::const_iterator it = occ_ids.begin();
      it != occ_ids.end();
      it++, i++) {
    unsigned int mx, my;
    double wx, wy;
    costmap_.convertFromMapIndexToXY((*it), mx, my);
    costmap_.convertFromIndexesToWorldCoord(mx, my, wx, wy);
    occ_diff.occ_points[i].x = wx;
    occ_diff.occ_points[i].y = wy;
  }

  i = 0;
  for(std::list<unsigned int>::const_iterator it = unocc_ids.begin();
      it != unocc_ids.end();
      it++, i++) {
    unsigned int mx, my;
    double wx, wy;
    costmap_.convertFromMapIndexToXY((*it), mx, my);
    costmap_.convertFromIndexesToWorldCoord(mx, my, wx, wy);
    occ_diff.unocc_points[i].x = wx;
    occ_diff.unocc_points[i].y = wy;
  }
  //std::cout << "New occ " << occ_ids.size() << " unocc " << unocc_ids.size() << std::endl;

  publish("transient_obstacles_diff", occ_diff);
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
