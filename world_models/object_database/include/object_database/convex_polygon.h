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

#ifndef OBJECT_DATABASE_POLYGON
#define OBJECT_DATABASE_POLYGON

#include <vector>
#include "half_plane.h"

using std::vector;

namespace object_database
{

/************************************************************
 * Class declaration
 ************************************************************/
class ConvexPolygon
{
public:
  /// Create polygon with given vertices (which must be in radial order)
  ConvexPolygon(const vector<Point2D>& vertices);

  /// Named constructor that takes in array (x1, ..., x_2n) and returns an
  /// n-sized polygon with vertices (x1,x2), (x3,x4), etc
  static ConvexPolygon polygonFromArray(const double* vertices, int num_vertices);

  /// Create empty polygon - don't call any ops on this
  ConvexPolygon() {}
  
  // The default copy and assignment are fine
  
  /// Return a (const) reference to the ith vertex
  const Point2D& getVertex(const int i) const { return vertices_[i]; }

  /// Return number of vertices
  int numVertices() const;

  /// Does polygon contain this point?
  bool contains (const Point2D& p) const;
  
  friend std::ostream& operator<< (std::ostream& stream, const ConvexPolygon& poly);

private:
  vector<Point2D> vertices_;
  vector<HalfPlane> half_planes_;
};


/************************************************************
 * API nonmember functions
 ************************************************************/

/// Do the two polygons intersect?
bool intersects (const ConvexPolygon& p1, const ConvexPolygon& p2);


/************************************************************
 * Inline member functions
 ************************************************************/

inline int ConvexPolygon::numVertices () const
{
  return vertices_.size();
}

} // namespace object_database

#endif
