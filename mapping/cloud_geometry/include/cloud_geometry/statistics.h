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

#ifndef _CLOUD_GEOMETRY_STATISTICS_H_
#define _CLOUD_GEOMETRY_STATISTICS_H_

// ROS includes
#include <std_msgs/PointCloud.h>
#include <std_msgs/Point32.h>
#include <std_msgs/Polygon3D.h>

namespace cloud_geometry
{

  namespace statistics
  {
    std_msgs::Point32 computeMedian (std_msgs::PointCloud points);
    std_msgs::Point32 computeMedian (std_msgs::PointCloud points, std::vector<int> indices);

    double computeMedianAbsoluteDeviation (std_msgs::PointCloud points, double sigma);
    double computeMedianAbsoluteDeviation (std_msgs::PointCloud points, std::vector<int> indices, double sigma);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centralized moment at a 3D points patch
      * \param points the point cloud data message
      * \param p the p-dimension
      * \param q the q-dimension
      * \param r the r-dimension
      */
    inline double
      computeCentralizedMoment (std_msgs::PointCloud *points, double p, double q, double r)
    {
      double result = 0.0;
      for (unsigned int cp = 0; cp < points->pts.size (); cp++)
        result += pow (points->pts[cp].x, p) * pow (points->pts[cp].y, q) * pow (points->pts[cp].z, r);

      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centralized moment at a 3D points patch, using their indices.
      * \param points the point cloud data message
      * \param indices the point cloud indices that need to be used
      * \param p the p-dimension
      * \param q the q-dimension
      * \param r the r-dimension
      */
    inline double
      computeCentralizedMoment (std_msgs::PointCloud *points, std::vector<int> *indices, double p, double q, double r)
    {
      double result = 0.0;
      for (unsigned int cp = 0; cp < indices->size (); cp++)
        result += pow (points->pts.at (indices->at (cp)).x, p) * pow (points->pts.at (indices->at (cp)).y, q) * pow (points->pts.at (indices->at (cp)).z, r);

      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the minimum and maximum 3D bounding box coordinates for a given point cloud
      * \param points the point cloud message
      * \param minP the resultant minimum bounding box coordinates
      * \param maxP the resultant maximum bounding box coordinates
      */
    inline void
      getMinMax (std_msgs::PointCloud *points, std_msgs::Point32 &minP, std_msgs::Point32 &maxP)
    {
      minP.x = minP.y = minP.z = FLT_MAX;
      maxP.x = maxP.y = maxP.z = FLT_MIN;

      for (unsigned int i = 0; i < points->pts.size (); i++)
      {
        minP.x = (points->pts[i].x < minP.x) ? points->pts[i].x : minP.x;
        minP.y = (points->pts[i].y < minP.y) ? points->pts[i].y : minP.y;
        minP.z = (points->pts[i].z < minP.z) ? points->pts[i].z : minP.z;

        maxP.x = (points->pts[i].x > maxP.x) ? points->pts[i].x : maxP.x;
        maxP.y = (points->pts[i].y > maxP.y) ? points->pts[i].y : maxP.y;
        maxP.z = (points->pts[i].z > maxP.z) ? points->pts[i].z : maxP.z;
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the minimum and maximum 3D bounding box coordinates for a given point cloud
      * \param points the point cloud message
      * \param minP the resultant minimum bounding box coordinates
      * \param maxP the resultant maximum bounding box coordinates
      */
    inline void
      getMinMax (std_msgs::Polygon3D *poly, std_msgs::Point32 &minP, std_msgs::Point32 &maxP)
    {
      minP.x = minP.y = minP.z = FLT_MAX;
      maxP.x = maxP.y = maxP.z = FLT_MIN;

      for (unsigned int i = 0; i < poly->points.size (); i++)
      {
        minP.x = (poly->points[i].x < minP.x) ? poly->points[i].x : minP.x;
        minP.y = (poly->points[i].y < minP.y) ? poly->points[i].y : minP.y;
        minP.z = (poly->points[i].z < minP.z) ? poly->points[i].z : minP.z;

        maxP.x = (poly->points[i].x > maxP.x) ? poly->points[i].x : maxP.x;
        maxP.y = (poly->points[i].y > maxP.y) ? poly->points[i].y : maxP.y;
        maxP.z = (poly->points[i].z > maxP.z) ? poly->points[i].z : maxP.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the minimum and maximum 3D bounding box coordinates for a given point cloud, using point indices
      * \param points the point cloud message
      * \param indices the point cloud indices to use
      * \param minP the resultant minimum bounding box coordinates
      * \param maxP the resultant maximum bounding box coordinates
      */
    inline void
      getMinMax (std_msgs::PointCloud *points, std::vector<int> *indices, std_msgs::Point32 &minP, std_msgs::Point32 &maxP)
    {
      minP.x = minP.y = minP.z = FLT_MAX;
      maxP.x = maxP.y = maxP.z = FLT_MIN;

      for (unsigned int i = 0; i < indices->size (); i++)
      {
        minP.x = (points->pts.at (indices->at (i)).x < minP.x) ? points->pts.at (indices->at (i)).x : minP.x;
        minP.y = (points->pts.at (indices->at (i)).y < minP.y) ? points->pts.at (indices->at (i)).y : minP.y;
        minP.z = (points->pts.at (indices->at (i)).z < minP.z) ? points->pts.at (indices->at (i)).z : minP.z;

        maxP.x = (points->pts.at (indices->at (i)).x > maxP.x) ? points->pts.at (indices->at (i)).x : maxP.x;
        maxP.y = (points->pts.at (indices->at (i)).y > maxP.y) ? points->pts.at (indices->at (i)).y : maxP.y;
        maxP.z = (points->pts.at (indices->at (i)).z > maxP.z) ? points->pts.at (indices->at (i)).z : maxP.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
      * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
      * \param points the point cloud data message
      * \param min_pt the resultant minimum bounds
      * \param max_pt the resultant maximum bounds
      * \param c_idx the index of the channel holding distance information
      * \param cut_distance a maximum admissible distance threshold for points from the laser origin
      */
    inline void
      getMinMax (std_msgs::PointCloud *points, std_msgs::Point32 &min_pt, std_msgs::Point32 &max_pt,
                 int c_idx, double cut_distance)
    {
      min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
      max_pt.x = max_pt.y = max_pt.z = FLT_MIN;

      for (unsigned int i = 0; i < points->pts.size (); i++)
      {
        if (c_idx != -1 && points->chan[c_idx].vals[i] > cut_distance)
          continue;
        min_pt.x = (points->pts[i].x < min_pt.x) ? points->pts[i].x : min_pt.x;
        min_pt.y = (points->pts[i].y < min_pt.y) ? points->pts[i].y : min_pt.y;
        min_pt.z = (points->pts[i].z < min_pt.z) ? points->pts[i].z : min_pt.z;

        max_pt.x = (points->pts[i].x > max_pt.x) ? points->pts[i].x : max_pt.x;
        max_pt.y = (points->pts[i].y > max_pt.y) ? points->pts[i].y : max_pt.y;
        max_pt.z = (points->pts[i].z > max_pt.z) ? points->pts[i].z : max_pt.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
      * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
      * \param points the point cloud data message
      * \param indices the point cloud indices to use
      * \param min_pt the resultant minimum bounds
      * \param max_pt the resultant maximum bounds
      * \param c_idx the index of the channel holding distance information
      * \param cut_distance a maximum admissible distance threshold for points from the laser origin
      */
    inline void
      getMinMax (std_msgs::PointCloud *points, std::vector<int> *indices, std_msgs::Point32 &min_pt, std_msgs::Point32 &max_pt,
                 int c_idx, double cut_distance)
    {
      min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
      max_pt.x = max_pt.y = max_pt.z = FLT_MIN;

      for (unsigned int i = 0; i < indices->size (); i++)
      {
        if (c_idx != -1 && points->chan[c_idx].vals[indices->at (i)] > cut_distance)
          continue;
        min_pt.x = (points->pts[indices->at (i)].x < min_pt.x) ? points->pts[indices->at (i)].x : min_pt.x;
        min_pt.y = (points->pts[indices->at (i)].y < min_pt.y) ? points->pts[indices->at (i)].y : min_pt.y;
        min_pt.z = (points->pts[indices->at (i)].z < min_pt.z) ? points->pts[indices->at (i)].z : min_pt.z;

        max_pt.x = (points->pts[indices->at (i)].x > max_pt.x) ? points->pts[indices->at (i)].x : max_pt.x;
        max_pt.y = (points->pts[indices->at (i)].y > max_pt.y) ? points->pts[indices->at (i)].y : max_pt.y;
        max_pt.z = (points->pts[indices->at (i)].z > max_pt.z) ? points->pts[indices->at (i)].z : max_pt.z;
      }
    }

    void getChannelMeanStd (std_msgs::PointCloud *points, int d_idx, double &mean, double &stddev);
    void getChannelMeanStd (std_msgs::PointCloud *points, std::vector<int> *indices, int d_idx, double &mean, double &stddev);

    void selectPointsOutsideDistribution (std_msgs::PointCloud *points, std::vector<int> *indices, int d_idx,
                                          double mean, double stddev, double alpha, std::vector<int> &inliers);
    void selectPointsInsideDistribution (std_msgs::PointCloud *points, std::vector<int> *indices, int d_idx,
                                         double mean, double stddev, double alpha, std::vector<int> &inliers);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute both the mean and the standard deviation of an array of values
      * \param values the array of values
      * \param mean the resultant mean of the distribution
      * \param stddev the resultant standard deviation of the distribution
      */
    inline void
      getMeanStd (std::vector<int> *values, double &mean, double &stddev)
    {
      double sum = 0, sq_sum = 0;

      for (unsigned int i = 0; i < values->size (); i++)
      {
        sum += values->at (i);
        sq_sum += values->at (i) * values->at (i);
      }
      mean = sum / values->size ();
      double variance = (double)(sq_sum - sum * sum / values->size ()) / (values->size () - 1);
      stddev = sqrt (variance);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute both the trimean (a statistical resistent L-Estimate) of an array of values
      * \note The trimean is computed by adding the 25th percentile plus twice the 50th percentile plus the 75th
      * percentile and dividing by four.
      * \param values the array of values
      * \param trimean the resultant trimean of the distribution
      */
    inline void
      getTrimean (std::vector<int> *values, double &trimean)
    {
      double sum = 0;

      for (unsigned int i = 0; i < values->size (); i++)
      {
        sum += values->at (i);
      }
      trimean = sum / values->size ();
    }

  }
}

#endif
