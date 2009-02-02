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

#include "object_database/convex_polygon.h"
#include <iostream>
#include <algorithm>

using namespace std;

namespace object_database
{

// Internal
class DoesNotContain {
public:
  DoesNotContain (const Point2D& p) : p_(p) { }
  bool operator() (const HalfPlane& h) { return !h.contains(p_); }
private:
  const Point2D& p_;
};

// Interface

ConvexPolygon::ConvexPolygon (const vector<Point2D>& vertices)
{
  // Copy over argument
  // Should ideally do a check here that vertices are in radial order
  vertices_=vertices;
  int num_vertices=vertices_.size();
  double multiplier=0.0; // This will be set in first iteration of loop

  // Precompute the halfplanes
  half_planes_.reserve(num_vertices);
  for (int i=0; i<num_vertices; i++) {

    // make i and j be two consecutive vertices
    int j = i ? i-1 : num_vertices-1;
    
    double x1=vertices_[i].x;
    double x2=vertices_[j].x;
    double y1=vertices_[i].y;
    double y2=vertices_[j].y;

    double a=y2-y1;
    double b=x1-x2;
    double c=x1*(y1-y2) + y1*(x2-x1);

    // The first time round, need to compute a multiplier based on whether
    // points are in clockwise or counterclockwise order
    if (i==0) {
      // We'll use the multiplier (1 or -1) that ensures that vertex 1 is in the polygon
      if (a*vertices_[1].x + b*vertices_[1].y + c >= 0) {
        multiplier=1.0;
      }
      else {
        multiplier=-1.0;
      }
    }

    // create the half plane
    half_planes_.push_back(HalfPlane(multiplier*a,multiplier*b,multiplier*c));
  }
}


bool ConvexPolygon::contains (const Point2D& p) const
{
  return std::find_if (half_planes_.begin(), half_planes_.end(), DoesNotContain (p)) == half_planes_.end();
}

bool intersects (ConvexPolygon& p1, ConvexPolygon& p2)
{
  int n1=p1.numVertices();
  int n2=p2.numVertices();
  for (int i=0; i<n1; i++) {
    if (p2.contains(p1.getVertex(i))) {
      return true;
    }
  }
  for (int j=0; j<n2; j++) {
    if (p1.contains(p2.getVertex(j))) {
      return true;
    }
  }
  return false;
}


} // namespace object_database

using namespace object_database;

int main (int, char**)
{
  vector<Point2D> p1(3);
  vector<Point2D> p2(4);
  vector<Point2D> p3(3);

  p1[0] = Point2D(0,0);
  p1[1] = Point2D(3,0);
  p1[2] = Point2D(0,4);

  p2[0] = Point2D(2,-1);
  p2[1] = Point2D(2,1);
  p2[2] = Point2D(4,1);
  p2[3] = Point2D(4,-1);

  p3[0] = Point2D(4,1);
  p3[1] = Point2D(4,-1);
  p3[2] = Point2D(5,0);

  ConvexPolygon poly1(p1);
  ConvexPolygon poly2(p2);
  ConvexPolygon poly3(p3);

  if (object_database::intersects (poly1, poly2)) {
    cout << "P1 intersects P2" << endl;
  }
  else {
    cout << "P1 does not intersect P2" << endl;
  }

  if (object_database::intersects (poly2, poly1)) {
    cout << "P2 intersects P1" << endl;
  }
  else {
    cout << "P2 does not intersect P1" << endl;
  }

  if (object_database::intersects (poly1, poly3)) {
    cout << "P1 intersects P3" << endl;
  }
  else {
    cout << "P1 does not intersect P3" << endl;
  }

  if (object_database::intersects (poly3, poly1)) {
    cout << "P3 intersects P1" << endl;
  }
  else {
    cout << "P3 does not intersect P1" << endl;
  }

  if (object_database::intersects (poly2, poly3)) {
    cout << "P2 intersects P3" << endl;
  }
  else {
    cout << "P2 does not intersect P3" << endl;
  }

  if (object_database::intersects (poly3, poly2)) {
    cout << "P3 intersects P2" << endl;
  }
  else {
    cout << "P3 does not intersect P2" << endl;
  }

  

  

  return 0;
}
