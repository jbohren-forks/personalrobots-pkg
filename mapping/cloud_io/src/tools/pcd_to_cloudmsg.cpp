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
 * $Id: pcd_to_cloudmsg.cpp 8080 2008-12-14 23:40:14Z veedee $
 *
 */

/** \author Radu Bogdan Rusu
  *
  * Extremely silly PCD to C++ ROS std_msgs PointCloud converter.
  * Useful for tools which don't want to link against \a cloud_io (I guess).
  *
  */
#include "cloud_io/cloud_io.h"

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    fprintf (stderr, "Error. No command line argument(s) specified. Syntax is: %s <input.pcd>\n", argv[0]);
    return (-1);
  }
  
  std_msgs::PointCloud points;
  int res = cloud_io::loadPCDFile (argv[1], points);
  
  if (res != 0)
  {
    fprintf (stderr, "Error loading %s.\n", argv[1]);
    return (-1);
  }
  
  fprintf (stdout, "std_msgs::PointCloud points;\n");
  fprintf (stdout, "points.pts.resize (%i);\n", points.pts.size ());

  if (points.chan.size () > 0)
  {
    fprintf (stdout, "points.chan.resize (%i);\n", points.chan.size ());
    for (unsigned int d = 0; d < points.chan.size (); d++)
    {
      fprintf (stdout, "points.chan[%i].name = %s;\n", d, points.chan[d].name.c_str ());
      fprintf (stdout, "points.chan[%i].vals.resize (%i);", d, points.chan.size ());
    }
  }

  fprintf (stdout, "\n");
  for (unsigned int i = 0; i < points.pts.size (); i++)
    fprintf (stdout, "points.pts[%5i].x = %f; points.pts[%5i].y = %f; points.pts[%5i].z = %f;\n", i, points.pts[i].x, i, points.pts[i].y, i, points.pts[i].z);

  fprintf (stdout, "\n");
  for (unsigned int d = 0; d < points.chan.size (); d++)
    for (unsigned int i = 0; i < points.pts.size (); i++)
      fprintf (stdout, "points.chan[%i].vals[%5i] = %f;\n", d, i, points.chan[d].vals[i]);

  return (0);
}
/* ]--- */
