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
 * $Id: test_kdtree.cpp 8160 2008-12-16 07:41:55Z veedee $
 *
 */

/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include "std_msgs/PointCloud.h"

#include "cloud_kdtree/kdtree.h"

#include "bunny_model.h"      // Import the Stanford bunny model

using namespace cloud_kdtree;

TEST (CloudKdTree, CreateDestroy)
{
  std_msgs::PointCloud points;

  // Get a point cloud dataset
  cloud_kdtree_tests::getBunnyModel (points);

  // Create a KdTree object
  KdTree* tree = new KdTree (points);
  EXPECT_TRUE (tree != NULL);

  // Destroy the tree
  delete tree;
}

TEST (CloudKdTree, Search)
{
  bool state;
  std_msgs::PointCloud points;
  std::vector<int> indices;
  std::vector<double> distances;

  // Get a point cloud dataset
  cloud_kdtree_tests::getBunnyModel (points);

  // Create a KdTree object
  KdTree* tree = new KdTree (points);

  state = tree->nearestKSearch (points.pts[0], 10);
  EXPECT_EQ (state, true);

  tree->getNeighborsIndices (indices);
  tree->getNeighborsDistances (distances);
  EXPECT_EQ (indices[0], 0);
  EXPECT_EQ (indices[1], 12);
  EXPECT_EQ (indices[2], 198);
  EXPECT_EQ (indices[3], 1);
  EXPECT_EQ (indices[4], 127);
  EXPECT_EQ (indices[5], 18);
  EXPECT_EQ (indices[6], 132);
  EXPECT_EQ (indices[7], 10);
  EXPECT_EQ (indices[8], 11);
  EXPECT_EQ (indices[9], 197);
  EXPECT_NEAR (distances[0], 0, 1e-7);
  EXPECT_NEAR (distances[1], 3.75822e-05, 1e-7);
  EXPECT_NEAR (distances[2], 4.04651e-05, 1e-7);
  EXPECT_NEAR (distances[3], 5.2208e-05, 1e-7);
  EXPECT_NEAR (distances[4], 6.26006e-05, 1e-7);
  EXPECT_NEAR (distances[5], 9.67441e-05, 1e-7);
  EXPECT_NEAR (distances[6], 0.000103859, 1e-7);
  EXPECT_NEAR (distances[7], 0.000188363, 1e-7);
  EXPECT_NEAR (distances[8], 0.000198955, 1e-7);
  EXPECT_NEAR (distances[9], 0.000214294, 1e-7);

  state = tree->nearestKSearch (points, 0, 10);
  EXPECT_EQ (state, true);

  tree->getNeighborsIndices (indices);
  tree->getNeighborsDistances (distances);
  EXPECT_EQ (indices[0], 0);
  EXPECT_EQ (indices[1], 12);
  EXPECT_EQ (indices[2], 198);
  EXPECT_EQ (indices[3], 1);
  EXPECT_EQ (indices[4], 127);
  EXPECT_EQ (indices[5], 18);
  EXPECT_EQ (indices[6], 132);
  EXPECT_EQ (indices[7], 10);
  EXPECT_EQ (indices[8], 11);
  EXPECT_EQ (indices[9], 197);
  EXPECT_NEAR (distances[0], 0, 1e-7);
  EXPECT_NEAR (distances[1], 3.75822e-05, 1e-7);
  EXPECT_NEAR (distances[2], 4.04651e-05, 1e-7);
  EXPECT_NEAR (distances[3], 5.2208e-05, 1e-7);
  EXPECT_NEAR (distances[4], 6.26006e-05, 1e-7);
  EXPECT_NEAR (distances[5], 9.67441e-05, 1e-7);
  EXPECT_NEAR (distances[6], 0.000103859, 1e-7);
  EXPECT_NEAR (distances[7], 0.000188363, 1e-7);
  EXPECT_NEAR (distances[8], 0.000198955, 1e-7);
  EXPECT_NEAR (distances[9], 0.000214294, 1e-7);

  state = tree->radiusSearch (points.pts[0], 0.01);
  EXPECT_EQ (state, true);

  tree->getNeighborsIndices (indices);
  tree->getNeighborsDistances (distances);
  EXPECT_EQ (indices[0], 0);
  EXPECT_EQ (indices[1], 12);
  EXPECT_EQ (indices[2], 198);
  EXPECT_EQ (indices[3], 1);
  EXPECT_EQ (indices[4], 127);
  EXPECT_EQ (indices[5], 18);
  EXPECT_NEAR (distances[0], 0, 1e-7);
  EXPECT_NEAR (distances[1], 3.75822e-05, 1e-7);
  EXPECT_NEAR (distances[2], 4.04651e-05, 1e-7);
  EXPECT_NEAR (distances[3], 5.2208e-05, 1e-7);
  EXPECT_NEAR (distances[4], 6.26006e-05, 1e-7);
  EXPECT_NEAR (distances[5], 9.67441e-05, 1e-7);

  state = tree->radiusSearch (points, 0, 0.01);
  EXPECT_EQ (state, true);

  tree->getNeighborsIndices (indices);
  tree->getNeighborsDistances (distances);
  EXPECT_EQ (indices[0], 0);
  EXPECT_EQ (indices[1], 12);
  EXPECT_EQ (indices[2], 198);
  EXPECT_EQ (indices[3], 1);
  EXPECT_EQ (indices[4], 127);
  EXPECT_EQ (indices[5], 18);
  EXPECT_NEAR (distances[0], 0, 1e-7);
  EXPECT_NEAR (distances[1], 3.75822e-05, 1e-7);
  EXPECT_NEAR (distances[2], 4.04651e-05, 1e-7);
  EXPECT_NEAR (distances[3], 5.2208e-05, 1e-7);
  EXPECT_NEAR (distances[4], 6.26006e-05, 1e-7);
  EXPECT_NEAR (distances[5], 9.67441e-05, 1e-7);

  // Destroy the tree
  delete tree;
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
