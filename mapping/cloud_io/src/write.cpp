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

#include "cloud_io/cloud_io.h"

namespace cloud_io
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Save point cloud data to a PCD file containing n-D points
    * \param file_name the output file name
    * \param points the point cloud data message
    * \param precision the specified output numeric stream precision
    */
  int
    savePCDFile (const char* file_name, std_msgs::PointCloud points, int precision)
  {
    std::ofstream fs;
    fs.precision (precision);
    fs.open (file_name);      // Open file

    int nr_pts = points.get_pts_size ();
    int dim    = points.get_chan_size ();

    fs << "COLUMNS x y z";
    for (int d = 0; d < dim; d++)
      fs << " " << points.chan[d].name;
    fs << std::endl;
    fs << "POINTS " << nr_pts << std::endl;
    fs << "DATA ascii" << std::endl;

    // Iterate through the points
    for (int i = 0; i < nr_pts; i++)
    {
      fs << points.pts[i].x << " " << points.pts[i].y << " " << points.pts[i].z;
      for (int d = 0; d < dim; d++)
        fs << " " << points.chan[d].vals[i];
      fs << std::endl;
    }
    fs.close ();              // Close file
    return (0);
  }
}
