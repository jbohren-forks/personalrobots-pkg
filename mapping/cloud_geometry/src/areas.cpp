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

#include <cloud_geometry/point.h>
#include <cloud_geometry/areas.h>
#include <cloud_geometry/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cfloat>
#include <algorithm>

namespace cloud_geometry
{

  namespace areas
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
      * \param points the point cloud (planar)
      * \param normal the plane normal
      */
    double
      compute2DPolygonalArea (std_msgs::PointCloud points, std::vector<double> normal)
    {
      int k0, k1, k2;

      // Find axis with largest normal component and project onto perpendicular plane
      k0 = (fabs (normal.at (0) ) > fabs (normal.at (1))) ? 0  : 1;
      k0 = (fabs (normal.at (k0)) > fabs (normal.at (2))) ? k0 : 2;
      k1 = (k0 + 1) % 3;
      k2 = (k0 + 2) % 3;

      // cos(theta), where theta is the angle between the polygon and the projected plane
      double ct = fabs ( normal.at (k0) );

      double area = 0;
      float p_i[3], p_j[3];

      for (unsigned int i = 0; i < points.pts.size (); i++)
      {
        p_i[0] = points.pts[i].x; p_i[1] = points.pts[i].y; p_i[2] = points.pts[i].z;
        int j = (i + 1) % points.pts.size ();
        p_j[0] = points.pts[j].x; p_j[1] = points.pts[j].y; p_j[2] = points.pts[j].z;

        area += p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];
      }
      area = fabs (area) / (2 * ct);

      return (area);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
      * \param polygon the planar polygon
      * \param normal the plane normal
      */
    double
      compute2DPolygonalArea (std_msgs::Polygon3D polygon, std::vector<double> normal)
    {
      int k0, k1, k2;

      // Find axis with largest normal component and project onto perpendicular plane
      k0 = (fabs (normal.at (0) ) > fabs (normal.at (1))) ? 0  : 1;
      k0 = (fabs (normal.at (k0)) > fabs (normal.at (2))) ? k0 : 2;
      k1 = (k0 + 1) % 3;
      k2 = (k0 + 2) % 3;

      // cos(theta), where theta is the angle between the polygon and the projected plane
      double ct = fabs ( normal.at (k0) );

      double area = 0;
      float p_i[3], p_j[3];

      for (unsigned int i = 0; i < polygon.points.size (); i++)
      {
        p_i[0] = polygon.points[i].x; p_i[1] = polygon.points[i].y; p_i[2] = polygon.points[i].z;
        int j = (i + 1) % polygon.points.size ();
        p_j[0] = polygon.points[j].x; p_j[1] = polygon.points[j].y; p_j[2] = polygon.points[j].z;

        area += p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];
      }
      area = fabs (area) / (2 * ct);

      return (area);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute a 2D convex hull in 3D space using Andrew's monotone chain algorithm
      * \param points the point cloud
      * \param indices the point indices to use from the cloud (they must form a planar model)
      * \param coeff the *normalized* planar model coefficients
      * \param hull the resultant convex hull model as a \a Polygon3D
      */
    void
      convexHull2D (std_msgs::PointCloud *points, std::vector<int> *indices, std::vector<double> *coeff, std_msgs::Polygon3D &hull)
    {
      // Copy the point data to a local Eigen::Matrix. This is slow and should be replaced by extending std_msgs::Point32
      // to allow []/() accessors.
      std::vector<Eigen::Vector3f> epoints (indices->size ());
      for (unsigned int cp = 0; cp < indices->size (); cp++)
      {
        epoints[cp](0) = points->pts[indices->at (cp)].x;
        epoints[cp](1) = points->pts[indices->at (cp)].y;
        epoints[cp](2) = points->pts[indices->at (cp)].z;
      }

      // Determine the best plane to project points onto
      int k0, k1, k2;
      k0 = (fabs (coeff->at (0) ) > fabs (coeff->at (1))) ? 0  : 1;
      k0 = (fabs (coeff->at (k0)) > fabs (coeff->at (2))) ? k0 : 2;
      k1 = (k0 + 1) % 3;
      k2 = (k0 + 2) % 3;

      // Compute a 2D centroid for two dimensions
      Eigen::Vector2d centroid (0, 0);
      for (unsigned int cp = 0; cp < epoints.size (); cp++)
      {
        centroid (0) += epoints[cp](k1);
        centroid (1) += epoints[cp](k2);
      }
      centroid (0) /= epoints.size ();
      centroid (1) /= epoints.size ();

      // Push projected centered 2d points
      std::vector<std_msgs::Point2DFloat32> epoints_demean (epoints.size ());
      for (unsigned int cp = 0; cp < indices->size (); cp++)
      {
        epoints_demean[cp].x = epoints[cp](k1) - centroid (0);
        epoints_demean[cp].y = epoints[cp](k2) - centroid (1);
      }
      
      std::sort (epoints_demean.begin (), epoints_demean.end (), comparePoint2DFloat32);

      std_msgs::Polyline2D hull_2d;
      convexHull2D (epoints_demean, hull_2d);

      int nr_points_hull = hull_2d.points.size ();
      if (nr_points_hull >= 3)
      {
        // Determine the convex hull direction
        Eigen::Vector3d p1, p2, p3;

        p1 (k0) = 0;
        p1 (k1) = -hull_2d.points[0].x + hull_2d.points[1].x;
        p1 (k2) = -hull_2d.points[0].y + hull_2d.points[1].y;

        p2 (k0) = 0;
        p2 (k1) = -hull_2d.points[0].x + hull_2d.points[2].x;
        p2 (k2) = -hull_2d.points[0].y + hull_2d.points[2].y;

        p3 = p1.cross (p2);

        bool direction = (p3 (k0) * coeff->at (k0) > 0);

        // Create the Polygon3D object
        hull.points.resize (nr_points_hull);

        // Copy hull points in clockwise or anti-clockwise format
        for (int cp = 0; cp < nr_points_hull; cp++)
        {
          int d = direction ? cp : (nr_points_hull - cp - 1);
          Eigen::Vector3f pt;
          pt (k1) = hull_2d.points[cp].x + centroid (0);
          pt (k2) = hull_2d.points[cp].y + centroid (1);
          pt (k0) = -(coeff->at (3) + pt (k1) * coeff->at (k1) + pt (k2) * coeff->at (k2)) / coeff->at (k0);

          // Copy the point data to Polygon3D format
          hull.points[d].x = pt (0);
          hull.points[d].y = pt (1);
          hull.points[d].z = pt (2);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute a 2D convex hull using Andrew's monotone chain algorithm
      * \note (code snippet inspired from http://www.softsurfer.com/Archive/algorithm_0109/algorithm_0109.htm)
      *        Copyright 2001, softSurfer (www.softsurfer.com)
      * \param points the 2D projected point cloud representing a planar model
      * \param hull the resultant 2D convex hull model as a \a Polyline2D
      */
    void
      convexHull2D (std::vector<std_msgs::Point2DFloat32> points, std_msgs::Polyline2D &hull)
    {
      int nr_points = points.size ();
      hull.points.resize (nr_points + 1);

      // Indices for bottom and top of the stack
      int bot = 0, top = -1;
      int i;

      for (i = 1; i < nr_points; i++)
        // points[0].x represents the smallest X coordinate
        if (points[i].x != points[0].x)
          break;

      // Get the indices of points with min|max y-coord
      int minmax = i - 1;

      // Degenerate case: all x-coords == xmin
      if ( minmax == (nr_points - 1) )
      {
        ++top;
        hull.points[top].x = points[0].x;
        hull.points[top].y = points[0].y;
        // A nontrivial segment
        if (points[minmax].y != points[0].y)
        {
          ++top;
          hull.points[top].x = points[minmax].x;
          hull.points[top].y = points[minmax].y;
        }
        ++top;
        // Add the polygon's endpoint
        hull.points[top].x = points[0].x;
        hull.points[top].y = points[0].y;
        hull.points.resize (top + 1);
        return;
      }

      int maxmin;
      for (i = nr_points - 2; i >= 0; i--)
        if (points[i].x != points[nr_points - 1].x)
          break;
      maxmin = i + 1;

      // Compute the lower hull
      ++top;
      // Add the polygon's endpoint
      hull.points[top].x = points[0].x;
      hull.points[top].y = points[0].y;

      i = minmax;
      while (++i <= maxmin)
      {
        // The lower line joins P[minmin] with P[maxmin]
        if ((i < maxmin) && (
            (points[maxmin].x - points[0].x) * (points[i].y      - points[0].y) -
            (points[i].x      - points[0].x) * (points[maxmin].y - points[0].y) >= 0))
          continue;          // ignore P[i] above or on the lower line

        // If there are at least 2 points on the stack
        while (top > 0)
        {
          // Test if P[i] is left of the line at the stack top
          if ((hull.points[top].x - hull.points[top-1].x) * (points[i].y        - hull.points[top-1].y) -
              (points[i].x        - hull.points[top-1].x) * (hull.points[top].y - hull.points[top-1].y) > 0)
            break;         // P[i] is a new hull vertex
          else
            top--;         // pop top point off stack
        }
        ++top;
        hull.points[top].x = points[i].x;
        hull.points[top].y = points[i].y;
      }

      // Next, compute the upper hull above the bottom hull
      if ((nr_points - 1) != maxmin)      // if distinct xmax points
      {
        ++top;
        // Add the point with max X and max Y coordinates to the hull
        hull.points[top].x = points[nr_points - 1].x;
        hull.points[top].y = points[nr_points - 1].y;
      }
      // The bottom point of the upper hull stack
      bot = top;

      i = maxmin;
      while (--i >= minmax)
      {
        // The upper line joins P[nr_points - 1] with P[minmax]
        if ((i > minmax) && (
            (points[minmax].x - points[nr_points - 1].x) * (points[i].y      - points[nr_points - 1].y) -
            (points[i].x      - points[nr_points - 1].x) * (points[minmax].y - points[nr_points - 1].y) >= 0))
          continue;          // ignore P[i] below or on the upper line

        // If there are at least 2 points on the stack
        while (top > bot)
        {
          // Test if P[i] is left of the line at the stack top
          if ((hull.points[top].x - hull.points[top-1].x) * (points[i].y        - hull.points[top-1].y) -
              (points[i].x        - hull.points[top-1].x) * (hull.points[top].y - hull.points[top-1].y) > 0)
            break;         // P[i] is a new hull vertex
          else
            top--;         // pop top point off stack
        }
        ++top;

        hull.points[top].x = points[i].x;
        hull.points[top].y = points[i].y;
      }

      if (minmax != 0)
      {
        ++top;
        // Add the polygon's endpoint
        hull.points[top].x = points[0].x;
        hull.points[top].y = points[0].y;
      }
      hull.points.resize (top + 1);
      return;
    }

  }
}
