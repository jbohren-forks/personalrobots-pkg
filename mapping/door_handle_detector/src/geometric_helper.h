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
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/Point32.h>
#include <robot_msgs/PointStamped.h>

#include <tf/transform_listener.h>

// Include the service call type
#include "door_handle_detector/DoorDetector.h"

// Point Cloud Mapping includes
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/intersections.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/projections.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/cloud_kdtree.h>
// Sample Consensus
#include <sample_consensus/sac.h>
#include <sample_consensus/msac.h>
#include <sample_consensus/ransac.h>
#include <sample_consensus/lmeds.h>
#include <sample_consensus/sac_model_plane.h>
#include <sample_consensus/sac_model_oriented_line.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Transform a given point from its current frame to a given target frame
  * \param tf a pointer to a TransformListener object
  * \param target_frame the target frame to transform the point into
  * \param stamped_in the input point
  * \param stamped_out the output point
  */
inline void
  transformPoint (tf::TransformListener *tf, const std::string &target_frame,
                  const tf::Stamped< robot_msgs::Point32 > &stamped_in, tf::Stamped< robot_msgs::Point32 > &stamped_out)
{
  tf::Stamped<tf::Point> tmp;
  tmp.stamp_ = stamped_in.stamp_;
  tmp.frame_id_ = stamped_in.frame_id_;
  tmp[0] = stamped_in.x;
  tmp[1] = stamped_in.y;
  tmp[2] = stamped_in.z;

  tf->transformPoint (target_frame, tmp, tmp);

  stamped_out.stamp_ = tmp.stamp_;
  stamped_out.frame_id_ = tmp.frame_id_;
  stamped_out.x = tmp[0];
  stamped_out.y = tmp[1];
  stamped_out.z = tmp[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Transform a value from a source frame to a target frame at a certain moment in time with TF
  * \param val the value to transform
  * \param src_frame the source frame to transform the value from
  * \param tgt_frame the target frame to transform the value into
  * \param stamp a given time stamp
  * \param tf a pointer to a TransformListener object
  */
inline double
  transformDoubleValueTF (double val, std::string src_frame, std::string tgt_frame, ros::Time stamp, tf::TransformListener *tf)
{
  robot_msgs::Point32 temp;
  temp.x = temp.y = 0;
  temp.z = val;
  tf::Stamped<robot_msgs::Point32> temp_stamped (temp, stamp, src_frame);
  transformPoint (tf, tgt_frame, temp_stamped, temp_stamped);
  return (temp_stamped.z);
}


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

void obtainCloudIndicesSet (robot_msgs::PointCloud *points, std::vector<int> &indices, door_handle_detector::DoorDetector::Request door_req,
                            tf::TransformListener *tf, std::string fixed_param_frame, double min_z_bounds, double max_z_bounds, double frame_multiplier);


int fitSACOrientedLine (robot_msgs::PointCloud *points, std::vector<int> indices, double dist_thresh,
                        robot_msgs::Point32 *axis, double eps_angle, std::vector<int> &line_inliers);

void get3DBounds (robot_msgs::Point32 *p1, robot_msgs::Point32 *p2, robot_msgs::Point32 &min_b, robot_msgs::Point32 &max_b,
                  double min_z_bounds, double max_z_bounds, int multiplier);

void getCloudViewPoint (std::string cloud_frame, robot_msgs::PointStamped &viewpoint_cloud, tf::TransformListener *tf);

bool checkDoorEdges (robot_msgs::Polygon3D *poly, robot_msgs::Point32 *z_axis, double min_height, double eps_angle,
                     double &door_frame1, double &door_frame2);

void selectBestDistributionStatistics (robot_msgs::PointCloud *points, std::vector<int> *indices, int d_idx, std::vector<int> &inliers);

bool checkIfClusterPerpendicular (robot_msgs::PointCloud *points, std::vector<int> *indices, robot_msgs::PointStamped *viewpoint,
                                  std::vector<double> *coeff, double eps_angle);
void findClusters (robot_msgs::PointCloud *points, std::vector<int> *indices, double tolerance, std::vector<std::vector<int> > &clusters,
                   int nx_idx, int ny_idx, int nz_idx, double eps_angle, unsigned int min_pts_per_cluster = 1);

int fitSACPlane (robot_msgs::PointCloud &points, std::vector<int> indices, std::vector<int> &inliers, std::vector<double> &coeff,
                 robot_msgs::PointStamped *viewpoint_cloud, double dist_thresh, int min_pts);

void estimatePointNormals (robot_msgs::PointCloud *points, std::vector<int> *point_indices, robot_msgs::PointCloud *points_down, int k, robot_msgs::PointStamped *viewpoint_cloud);

#endif
