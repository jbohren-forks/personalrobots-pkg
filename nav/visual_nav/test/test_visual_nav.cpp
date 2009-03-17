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

#include <visual_nav/visual_nav.h>
#include <visual_nav/exceptions.h>
#include <visual_nav/transform.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace visual_nav;
using namespace std;
using boost::shared_ptr;


TEST(VisualNav, Transforms)
{
  Pose p1(1,5,0);
  Pose p2(8,3,.1);
  Pose p3(7,4,-.3);

  double pi=3.14159265;
  
  Transform2D t1(3,6,pi/2);

  EXPECT_TRUE(!(p1==p2));
  EXPECT_EQ(transform(t1, p1), Pose(-2,7,pi/2));
  Transform2D t2 = inverse(t1);
  EXPECT_EQ(transform(t2, transform(t1, p1)), p1);
  Transform2D t3 = getTransformBetween (p1, p2);
  EXPECT_EQ(transform(t3,p1), p2);
}


bool scanContains (const PointSet& scan, const Point2D& p)
{
  cout << "scanContains result is " << (scan.find(p)!=scan.end()) << endl;
  return scan.find(p)!=scan.end();
}

TEST(VisualNav, BasicAPI)
{
  VisualNavRoadmap r;
  typedef vector<int> Path;

  NodeId first = r.addNode(2,1.2);
  NodeId second = r.addNode(2,6);
  r.addNode(4,3);
  r.addNode(3,-1);

  PointSet points = r.overlayScans();
  EXPECT_EQ(0u, points.size());
  

  r.addEdge(2,1);
  r.addEdge(0,2);
  r.addNode(1,2.5);
  r.addEdge(4,2);
  r.addEdge(1,4);
  EXPECT_EQ(r.addNode(4,7), 5);
  

  r.addEdge(1,5);
  PathPtr path1 = r.pathToGoal(0,5);
  int expected_path[4] = {0, 2, 1, 5};
  EXPECT_TRUE(*path1==Path(expected_path, expected_path+4));
  EXPECT_EQ(r.pathExitPoint(path1, .1), Pose(4,3,0));
  EXPECT_EQ(r.pathExitPoint(path1, 2.5), Pose(4,3,0));
  EXPECT_EQ(r.pathExitPoint(path1, 3.0), Pose(2,6,0));
  EXPECT_EQ(r.pathExitPoint(path1, 4.7), Pose(2,6,0));

  r.addEdge(0,4);
  PathPtr path2 = r.pathToGoal(0,5);
  int expected_path2[4] = {0, 4, 1, 5};
  EXPECT_TRUE(*path2==Path(expected_path2, expected_path2+4));
  EXPECT_EQ(r.pathExitPoint(path2, 1.0), Pose(1,2.5));
  EXPECT_EQ(r.pathExitPoint(path2, 2.0), Pose(2,6));

  PointSet scan1, scan2;
  scan1.insert(Point2D(1,2));
  scan1.insert(Point2D(3,4));
  scan2.insert(Point2D(5,6));
  r.attachScan(first, scan1, Pose(1,2.2));
  r.attachScan(second, scan2, Pose(0,0));

  PointSet scan = r.overlayScans();

  EXPECT_EQ(3u, scan.size());
  EXPECT_EQ(Point2D(2.0,1.0), *(scan.begin()));
  PointSet::iterator iter=scan.begin();
  EXPECT_EQ(*(iter++), Point2D(2.0,1.0));
  EXPECT_EQ(*(iter++), Point2D(4,3));
  EXPECT_EQ(*(iter++), Point2D(7,12));
}



// Just repeat the above tests but read in the file
TEST(VisualNav, ReadFromFile)
{
  RoadmapPtr r = readRoadmapFromFile("test/example_roadmap.dat");
  typedef vector<int> Path;

  PathPtr path2 = r->pathToGoal(0,5);
  int expected_path2[4] = {0, 4, 1, 5};
  EXPECT_TRUE(*path2==Path(expected_path2, expected_path2+4));
  EXPECT_EQ(r->pathExitPoint(path2, 1.0), Pose(1,2.5));
  EXPECT_EQ(r->pathExitPoint(path2, 2.0), Pose(2,6));
}



int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
