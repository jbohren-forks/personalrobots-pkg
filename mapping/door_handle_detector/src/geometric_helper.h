/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
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
 * $Id$
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _DOOR_HANDLE_GEOMETRIC_HELPER_H_
#define _DOOR_HANDLE_GEOMETRIC_HELPER_H_

#include <math.h>
#include <vector>

// ROS includes
#include <std_msgs/PointCloud.h>
#include <std_msgs/Polygon3D.h>
#include <std_msgs/Point32.h>
#include <std_msgs/PointStamped.h>

#include <tf/transform_listener.h>

// Point Cloud Mapping includes
#include <cloud_geometry/angles.h>
#include <cloud_geometry/areas.h>
#include <cloud_geometry/distances.h>
#include <cloud_geometry/intersections.h>
#include <cloud_geometry/nearest.h>
#include <cloud_geometry/transforms.h>
#include <cloud_geometry/point.h>
#include <cloud_geometry/projections.h>
#include <cloud_geometry/statistics.h>
#include <cloud_geometry/transforms.h>
#include <cloud_kdtree/kdtree.h>
// Sample Consensus
#include <sample_consensus/sac.h>
#include <sample_consensus/msac.h>
#include <sample_consensus/ransac.h>
#include <sample_consensus/lmeds.h>
#include <sample_consensus/sac_model_plane.h>
#include <sample_consensus/sac_model_oriented_line.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Comparison operator for a vector of vectors
inline bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Obtain a 24-bit RGB coded value from 3 independent <r, g, b> channel values
  * \param r the red channel value
  * \param g the green channel value
  * \param b the blue channel value
  */
inline double
  getRGB (float r, float g, float b)
{
  int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
  double rgb = *(float*)(&res);
  return (rgb);
}

void get3DBounds (std_msgs::Point32 *p1, std_msgs::Point32 *p2, std_msgs::Point32 &min_b, std_msgs::Point32 &max_b,
                  double min_z_bounds, double max_z_bounds, int multiplier);

void getCloudViewPoint (std::string cloud_frame, std_msgs::PointStamped &viewpoint_cloud, tf::TransformListener *tf);

bool checkDoorEdges (std_msgs::Polygon3D *poly, std_msgs::Point32 *z_axis, double min_height, double eps_angle,
                     double &door_frame1, double &door_frame2);

void selectBestDistributionStatistics (std_msgs::PointCloud *points, std::vector<int> *indices, int d_idx, std::vector<int> &inliers);

bool checkIfClusterPerpendicular (std_msgs::PointCloud *points, std::vector<int> *indices, std_msgs::PointStamped *viewpoint,
                                  std::vector<double> *coeff, double eps_angle);
void findClusters (std_msgs::PointCloud *points, std::vector<int> *indices, double tolerance, std::vector<std::vector<int> > &clusters,
                   int nx_idx, int ny_idx, int nz_idx, double eps_angle, unsigned int min_pts_per_cluster = 1);

int fitSACPlane (std_msgs::PointCloud &points, std::vector<int> indices, std::vector<int> &inliers, std::vector<double> &coeff,
                 std_msgs::PointStamped *viewpoint_cloud, double dist_thresh, int min_pts);

void estimatePointNormals (std_msgs::PointCloud *points, std::vector<int> *point_indices, std_msgs::PointCloud *points_down, int k, std_msgs::PointStamped *viewpoint_cloud);

#endif
