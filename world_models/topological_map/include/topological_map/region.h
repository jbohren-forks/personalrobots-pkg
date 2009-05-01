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


#ifndef TOPOLOGICAL_MAP_REGION_H
#define TOPOLOGICAL_MAP_REGION_H

#include <iostream>
#include <set>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace topological_map
{

using std::ostream;

struct Cell2D
{
  int r, c;
  Cell2D() {}
  Cell2D(const int row, const int column) : r(row), c(column) {}
};

ostream& operator<< (ostream& str, const Cell2D& c);

inline 
int operator< (const Cell2D& c, const Cell2D& c2)
{
  return (c.r<c2.r) || ((c.r==c2.r) && (c.c<c2.c));
}


inline
bool operator== (const Cell2D& c, const Cell2D& c2)
{
  return (c.r==c2.r) && (c.c==c2.c);
}



std::vector<Cell2D> cellNeighbors (const Cell2D& c);

/// Represent a region as a set of Cell2D.
typedef std::set<Cell2D> Region;
typedef boost::shared_ptr<const Region> RegionPtr;

/// Use when creating regions
typedef boost::shared_ptr<Region> MutableRegionPtr;

/// Convenience
typedef unsigned int uint;




struct Point2D 
{
  Point2D(double x=0.0, double y=0.0) : x(x), y(y) {}
  double x,y;
};

bool operator== (const Point2D& p1, const Point2D& p2);
ostream& operator<< (ostream& str, const Point2D& p);

Point2D cellCorner (const Cell2D& cell, double resolution);
Point2D cellCenter (const Cell2D& cell, double resolution);

Cell2D pointToCell (const Point2D& p, double resolution);
Point2D cellToPoint (const Cell2D& c, double resolution);







} // namespace topological_map

#endif
