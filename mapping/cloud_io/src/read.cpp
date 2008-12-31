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

#include <stdlib.h>
#include "cloud_io/cloud_io.h"
#include "string_utils/string_utils.h"

namespace cloud_io
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Load point cloud data from a PCD file containing n-D points.
  * Returns -1 on error, 0 on success.
  * \note Only ASCII files are currently supported
  * \note All lines besides:
  * - the ones beginning with # (treated as comments)
  * - COLUMNS ...
  * - POINTS ...
  * - DATA ...
  * ...are intepreted as data points.
  * \param file_name the name of the file to load
  * \param points the resulting point array
  */
  int
    loadPCDFile (const char* file_name, std_msgs::PointCloud &points)
  {
    int nr_points = 0;
    std::ifstream fs;
    std::string line;

    int idx = 0;
    // Open file
    fs.open (file_name);
    if (!fs.is_open () || fs.fail ())
      return (-1);

    // Read the header and fill it in with wonderful values
    while (!fs.eof ())
    {
      getline (fs, line);
      if (line == "")
        continue;

      std::vector<std::string> st;
      string_utils::split (line, st, " ");

      std::string line_type = st.at (0);

      // ---[ Perform checks to see what does this line represents
      if (line_type.substr (0, 1) == "#")
        continue;
      // Get the column indices
      if (line_type.substr (0, 7) == "COLUMNS")
      {
        int remaining_tokens = st.size () - (1 + 3);
        points.set_chan_size (remaining_tokens);
        for (int i = 0; i < remaining_tokens; i++)
        {
          std::string col_type = st.at (i + 4);
          points.chan[i].name = col_type;
        }
        
        continue;
      }
      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        nr_points = atoi (st.at (1).c_str ());
        points.set_pts_size (nr_points);

        for (unsigned int d = 0; d < points.get_chan_size (); d++)
          points.chan[d].set_vals_size (nr_points);

        continue;
      }

      // Check DATA type
      if (line_type.substr (0, 4) == "DATA")
        continue;

      // Nothing of the above? We must have points then
      // Convert the first token to float and use it as the first point coordinate
      if (idx >= nr_points)
      {
        //fprintf (stderr, "Error: input file %s has more points than advertised (%d)!\n", file_name, nr_points);
        break;
      }

      // Assume x-y-z to be the first dimensions in the file
      points.pts[idx].x = atof (st.at (0).c_str ());
      points.pts[idx].y = atof (st.at (1).c_str ());
      points.pts[idx].z = atof (st.at (2).c_str ());
      for (unsigned int i = 0; i < points.get_chan_size (); i++)
        points.chan[i].vals[idx] = atof (st.at (i+3).c_str ());

      idx++;
    }
    // Close file
    fs.close ();

    if (idx != nr_points)
    {
      fprintf (stderr, "Warning! Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
      return (-1);
    }

    return (0);
  }
}
