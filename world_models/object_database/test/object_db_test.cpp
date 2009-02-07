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

#include <vector>
#include <string>
#include <algorithm>
#include <gtest/gtest.h>
#include "object_database/object_database.h"

using namespace object_database;
using namespace std;

// Helpers

struct NotMember
{
  NotMember(ObjectsPtr o) : objects(o) {}
  bool operator() (int i) { return find(objects->begin(), objects->end(), i) == objects->end(); }
  ObjectsPtr objects;
};

bool isRearrangement (ObjectsPtr objects, int* indices, int length)
{
  return (int)objects->size() == length && find_if(indices, indices+length, NotMember(objects))==indices+length;
}


// Tests


TEST(Polygon, intersection)
{
  vector<Point2D> p1(3);
  vector<Point2D> p2(4);

  p1[0] = Point2D(0,0);
  p1[1] = Point2D(3,0);
  p1[2] = Point2D(0,4);

  p2[0] = Point2D(2,-1);
  p2[1] = Point2D(2,1);
  p2[2] = Point2D(4,1);
  p2[3] = Point2D(4,-1);

  double p3[6] = {4,1,4,-1,5,0};

  ConvexPolygon poly1(p1);
  ConvexPolygon poly2(p2);
  ConvexPolygon poly3 = ConvexPolygon::polygonFromArray(p3, 3);

  EXPECT_TRUE(intersects (poly1, poly2));
  EXPECT_TRUE(intersects (poly2, poly1));
  EXPECT_FALSE(intersects (poly1, poly3));
  EXPECT_FALSE(intersects (poly3, poly1));
  EXPECT_TRUE(intersects (poly2, poly3));
  EXPECT_TRUE(intersects (poly3, poly2));
}

TEST(ObjectDB, all)
{
  // Set up
  vector<string> door_keys, person_keys;
  door_keys.push_back ("locked");
  door_keys.push_back ("angle");
  person_keys.push_back ("gender");
  person_keys.push_back ("speed");
  vector<ObjectTypeDescription> types;
  types.push_back(ObjectTypeDescription("door", door_keys));
  types.push_back(ObjectTypeDescription("person", person_keys));
  ObjectDatabase db(types);
  double p1[6] = {0,0, 1.5,2, 3,0};
  double p3[8] = {4,6, 3,5, 4,4, 5,5};
  double d1[8] = {6,1.5, 1.5,1.5, 1.5,.7, 6,.7};
  double d2[8] = {4,0, 5,0, 5,2, 4,2};
  double p4[8] = {0,4, 3,4, 3,5, 0,5};
  double large_box[8] = {-100,-100, -100,100, 100,100, 100,-100};
  double small_box[8] = {0,0, 0,4, 3,4, 3,0};
  ConvexPolygon b1 = ConvexPolygon::polygonFromArray(large_box, 4);
  ConvexPolygon b2 = ConvexPolygon::polygonFromArray(small_box, 4);



  // Add a few doors and people
  db.addObject ("door");
  db.addObject ("person");
  db.setKeyValue ("door", 1, "locked", 0);
  db.setKeyValue ("door", 1, "angle", 1.5);
  

  db.addObject ("door");
  db.addObject ("person", 3);
  db.setGeometry ("door", 1, ConvexPolygon::polygonFromArray(d1,4));
  db.setGeometry ("door", 2, ConvexPolygon::polygonFromArray(d2, 4));
  db.setGeometry ("person", 1, ConvexPolygon::polygonFromArray(p1, 3));
  db.setGeometry ("person", 3, ConvexPolygon::polygonFromArray(p3, 4));


  // Test key values
  EXPECT_EQ (db.getKeyValue("door", 1, "locked"), 0);
  EXPECT_EQ (db.getKeyValue("door", 2, "locked"), 0);
  EXPECT_EQ (db.getKeyValue("door", 1, "angle"), 1.5);
  EXPECT_EQ (db.getKeyValue("door", 2, "locked"), 0);

  // Test spatial queries
  int q1[2] = {1, 2};
  EXPECT_TRUE (isRearrangement(db.getObjects ("door", b1), q1, 2));
  int q2[1] = {1};
  EXPECT_TRUE (isRearrangement(db.getObjects ("person", b2), q2, 1));
  int q3[2] = {1, 3};
  EXPECT_TRUE (isRearrangement(db.getObjects ("person", b1), q3, 2));

  // Remove a person and retest spatial queries
  db.removeObject ("person", 1);
  EXPECT_TRUE (isRearrangement(db.getObjects ("door", b1), q1, 2));
  int q4[0] = {};
  EXPECT_TRUE (isRearrangement(db.getObjects ("person", b2), q4, 0));
  int q5[1] = {3};
  EXPECT_TRUE (isRearrangement(db.getObjects ("person", b1), q5, 1));

  // Add a person and retest
  db.addObject ("person");
  db.setGeometry ("person", 4, ConvexPolygon::polygonFromArray(p4, 4));
  EXPECT_TRUE (isRearrangement(db.getObjects ("door", b1), q1, 2));
  int q6[1] = {4};
  EXPECT_TRUE (isRearrangement(db.getObjects ("person", b2), q6, 1));
  int q7[2] = {3, 4};
  EXPECT_TRUE (isRearrangement(db.getObjects ("person", b1), q7, 2));

}

  



int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
