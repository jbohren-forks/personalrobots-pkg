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

#include "cloud_geometry/point.h"
#include "cloud_geometry/areas.h"

#include <cfloat>

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
      k0 = (fabs ( normal.at (0)  ) > fabs ( normal.at (1) )) ? 0  : 1;
      k0 = (fabs ( normal.at (k0) ) > fabs ( normal.at (2) )) ? k0 : 2;
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
  }
}
