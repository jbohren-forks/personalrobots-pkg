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

#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "rosconsole/rosconsole.h"

#include "sensor/sensor_range/rangesensor.h"
#include "sensor/sensor_odometry/odometrysensor.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping() 
{
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  ROS_ASSERT(gsp_);

  node_ = new ros::node("gmapping");
  ROS_ASSERT(gsp_);

  /// @todo Disable extrapolation, and implement scan-buffering.  This is
  /// not urgent for a robot like the PR2, where odometry is being
  /// published several times faster than laser scans.
  tf_ = new tf::TransformListener(*node_, true, 10000000000ULL);
  tf_->setExtrapolationLimit((int64_t) 200000000ULL );
  ROS_ASSERT(tf_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;

  node_->subscribe("base_scan", scan_, &SlamGMapping::laser_cb, this, -1);
  node_->advertise_service("dynamic_map", &SlamGMapping::map_cb, this);
  
  // Parameters used by our GMapping wrapper
  double tmp;
  node_->param("~/map_udpate_interval", tmp, 5.0);
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  node_->param("~/maxUrange", maxUrange_, 80.0);
  node_->param("~/sigma", sigma_, 0.05);
  node_->param("~/kernelSize", kernelSize_, 1);
  node_->param("~/lstep", lstep_, 0.05);
  node_->param("~/astep", astep_, 0.05);
  node_->param("~/iterations", iterations_, 5);
  node_->param("~/lsigma", lsigma_, 0.075);
  node_->param("~/ogain", ogain_, 3.0);
  node_->param("~/lskip", lskip_, 0);
  node_->param("~/srr", srr_, 0.1);
  node_->param("~/srt", srt_, 0.2);
  node_->param("~/str", str_, 0.1);
  node_->param("~/stt", stt_, 0.2);
  node_->param("~/linearUpdate", linearUpdate_, 1.0);
  node_->param("~/angularUpdate", angularUpdate_, 0.5);
  node_->param("~/resampleThreshold", resampleThreshold_, 0.5);
  node_->param("~/particles", particles_, 30);
  node_->param("~/xmin", xmin_, -100.0);
  node_->param("~/ymin", ymin_, -100.0);
  node_->param("~/xmax", xmax_, 100.0);
  node_->param("~/ymax", ymax_, 100.0);
  node_->param("~/delta", delta_, 0.05);
  node_->param("~/llsamplerange", llsamplerange_, 0.01);
  node_->param("~/llsamplestep", llsamplestep_, 0.01);
  node_->param("~/lasamplerange", lasamplerange_, 0.005);
  node_->param("~/lasamplestep", lasamplestep_, 0.005);
}

SlamGMapping::~SlamGMapping()
{
  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if(tf_)
    delete tf_;
  delete node_;
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the robot's pose 
  tf::Stamped<tf::Pose> ident (btTransform(btQuaternion(0,0,0), 
                                           btVector3(0,0,0)), t, "base");
  tf::Stamped<btTransform> odom_pose;
  try
  {
    this->tf_->transformPose("odom", ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw,pitch,roll;
  odom_pose.getBasis().getEulerZYX(yaw, pitch, roll);

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const std_msgs::LaserScan& scan)
{
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<btTransform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = "base_laser";
  ident.stamp_ = scan.header.stamp;
  try
  {
    this->tf_->transformPose("base", ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", 
             e.what());
    return false;
  }
  double yaw,pitch,roll;
  btMatrix3x3 mat =  laser_pose.getBasis();
  mat.getEulerZYX(yaw, pitch, roll);

  GMapping::OrientedPoint gmap_pose(laser_pose.getOrigin().x(),
                                    laser_pose.getOrigin().y(),
                                    yaw);
  ROS_DEBUG("laser's pose wrt base: %.3f %.3f %.3f",
            laser_pose.getOrigin().x(),
            laser_pose.getOrigin().y(),
            yaw);

  // The laser must be called "FLASER"
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         scan.ranges.size(),
                                         scan.angle_increment,
                                         gmap_pose,
                                         0.0,
                                         scan.range_max);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor("odom");
  ROS_ASSERT(gsp_odom_);

  double maxrange = scan.range_max;

  /// @todo Expose setting an intial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);

  gsp_->setMatchingParameters(maxUrange_, maxrange, sigma_, 
                              kernelSize_, lstep_, astep_, iterations_, 
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, 
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,time(NULL));

  ROS_INFO("Initialization complete");

  return true;
}

bool
SlamGMapping::addScan(const std_msgs::LaserScan& scan)
{
  GMapping::OrientedPoint gmap_pose;
  if(!getOdomPose(gmap_pose, scan.header.stamp))
     return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  for(unsigned int i=0; i < scan.ranges.size(); i++)
  {
    // Must filter out short readings, because the mapper won't
    if(scan.ranges[i] < scan.range_min)
      ranges_double[i] = (double)scan.range_max;
    else
      ranges_double[i] = (double)scan.ranges[i];
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_, 
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't 
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */

  return gsp_->processScan(reading);
}

void
SlamGMapping::laser_cb()
{
  static ros::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(scan_))
      return;
    got_first_scan_ = true;
  }

  if(addScan(scan_))
  {
    ROS_DEBUG("scan processed");
    
    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", 
              mpose.x, mpose.y, mpose.theta);

    ros::Time curr = ros::Time::now();
    if((curr - last_map_update) > map_update_interval_)
    {
      updateMap();
      last_map_update = curr;
      ROS_DEBUG("Updated the map");
    }
  }
}

void
SlamGMapping::updateMap()
{
  GMapping::ScanMatcher matcher;
  double* laser_angles = new double[scan_.ranges.size()];
  double theta = scan_.angle_min;
  for(unsigned int i=0; i<scan_.ranges.size(); i++)
  {
    laser_angles[i]=theta;
    theta += scan_.angle_increment;
  }
  /// @todo Check the pose that's being passed here
  matcher.setLaserParameters(scan_.ranges.size(), laser_angles, 
                             GMapping::OrientedPoint(0,0,0));
  delete[] laser_angles;
  matcher.setlaserMaxRange(scan_.range_max);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best = 
          gsp_->getParticles()[gsp_->getBestParticleIndex()];

  /// @todo Dynamically determine bounding box for map
  GMapping::Point wmin(-100.0, -100.0);
  GMapping::Point wmax(100.0, 100.0);
  map_.map.resolution = delta_;
  map_.map.origin.x = wmin.x;
  map_.map.origin.y = wmin.y;
  map_.map.origin.th = 0.0;

  GMapping::Point center;
  center.x=(wmin.x + wmax.x) / 2.0;
  center.y=(wmin.y + wmax.y) / 2.0;

  GMapping::ScanMatcherMap smap(center, wmin.x, wmin.y, wmax.x, wmax.y, 
                                map_.map.resolution);

  GMapping::IntPoint imin = smap.world2map(wmin);
  GMapping::IntPoint imax = smap.world2map(wmax);
  map_.map.width = imax.x - imin.x;
  map_.map.height = imax.y - imin.y;
  map_.map.data.resize(map_.map.width * map_.map.height);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f", 
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(imin.x + x, imin.y + y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.width, x, y)] = -1;
      else if(occ > 0.1)
        map_.map.data[MAP_IDX(map_.map.width, x, y)] = (int)round(occ*100.0);
      else
        map_.map.data[MAP_IDX(map_.map.width, x, y)] = 0;
    }
  }
  got_map_ = true;
}

bool 
SlamGMapping::map_cb(std_srvs::StaticMap::request  &req,
                     std_srvs::StaticMap::response &res)
{
  if(got_map_)
  {
    res = map_;
    return true;
  }
  else
    return false;
}
