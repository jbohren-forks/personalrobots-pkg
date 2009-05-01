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

#include <topological_map/region.h>

using std::ostream;
using std::vector;

namespace topological_map
{

ostream& operator<< (ostream& str, const Cell2D& c) 
{
  str << "(" << c.r << ", " << c.c << ")";
  return str;
}

vector<Cell2D> cellNeighbors (const Cell2D& p)
{
  int r=p.r;
  int c=p.c;
  vector<Cell2D> neighbors(4);
  neighbors[0]=Cell2D(r-1,c);
  neighbors[1]=Cell2D(r+1,c);
  neighbors[2]=Cell2D(r,c-1);
  neighbors[3]=Cell2D(r,c+1);
  return neighbors;
}



Point2D cellCorner (const Cell2D& cell, const double resolution)
{
  return Point2D(cell.c*resolution,cell.r*resolution);
}

Point2D cellCenter (const Cell2D& cell, const double resolution)
{
  return Point2D((.5+cell.c)*resolution, (.5+cell.r)*resolution);
}

Cell2D pointToCell (const Point2D& p, const double resolution)
{
  return Cell2D(p.y/resolution, p.x/resolution);
}

Point2D cellToPoint (const Cell2D& c, const double resolution)
{
  return Point2D(c.c*resolution, c.r*resolution);
}

} // namespace topological_map
