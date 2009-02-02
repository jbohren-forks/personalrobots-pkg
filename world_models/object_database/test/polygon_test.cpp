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
#include <vector>
#include <gtest/gtest.h>

using namespace object_database;

TEST(Polygon, intersection)
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

  EXPECT_TRUE(intersects (poly1, poly2));
  EXPECT_TRUE(intersects (poly2, poly1));
  EXPECT_FALSE(intersects (poly1, poly3));
  EXPECT_FALSE(intersects (poly3, poly1));
  EXPECT_TRUE(intersects (poly2, poly3));
  EXPECT_TRUE(intersects (poly3, poly2));
}


int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
