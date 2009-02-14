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

#include "topological_map/topological_map.h"
#include "topological_map/exception.h"
#include <iostream>
#include <gtest/gtest.h>

using namespace topological_map;
using namespace std;
using boost::shared_ptr;


// Helpers

bool isRearrangement (RegionIdVector v, RegionId* a, unsigned int k)
{
  if (v.size()!=k) {
    return false;
  }

  for (unsigned int i=0; i<k; i++) {
    if (find(v.begin(), v.end(), a[i])==v.end()) {
      return false;
    }
  }
  return true;
}



// Tests


TEST(TopologicalMap, BasicAPI)
{
  TopologicalMap m;
  unsigned int s=m.allRegions().size();
  int r, c;

  EXPECT_EQ (0u, s);
  
  MutableRegionPtr r1(new Region);
  for (r=2; r<4; r++) {
    for (c=-1; c<3; c++) {
      r1->insert(Cell2D(r,c));
    }
  }
  unsigned int id=m.addRegion(r1, 3);
  EXPECT_EQ(0u, id);
  EXPECT_EQ(1u, m.allRegions().size());

  MutableRegionPtr r2(new Region);
  
  for (r=4; r<8; r++) {
    for (c=-1; c<2; c++) {
      r2->insert(Cell2D(r,c));
    }
  }
  
  EXPECT_EQ(1u, m.addRegion(r2, 4));
  EXPECT_EQ(2u, m.allRegions().size());

  RegionIdVector n1=m.neighbors(0);
  RegionIdVector n2=m.neighbors(1);
  RegionId en1[1] = {1};
  RegionId en2[1] = {0};
  EXPECT_TRUE(isRearrangement(n1, en1, 1u));
  EXPECT_TRUE(isRearrangement(n2, en2, 1u));

  MutableRegionPtr r4(new Region);
  for (r=-10; r<10; r++) {
    for (c=-10; c<10; c++) {
      r4->insert(Cell2D(r,c));
    }
  }

  try {
    m.addRegion(r4, 2);
    ADD_FAILURE() << "Expected exception didn't happen";
  }
  catch (topological_map::OverlappingRegionException& e) {}

  MutableRegionPtr r3(new Region);
  r3->insert(Cell2D(3,3));
  r3->insert(Cell2D(4,3));
  EXPECT_EQ(2u, m.addRegion(r3, 3));
  EXPECT_EQ(3u, m.allRegions().size());

  n1=m.neighbors(0);
  n2=m.neighbors(1);
  RegionIdVector n3=m.neighbors(2);
  RegionId en11[2]={1,2};
  RegionId en3[1]={0};
  EXPECT_TRUE(isRearrangement(n1, en11, 2u));
  EXPECT_TRUE(isRearrangement(n2, en2, 1u));
  EXPECT_TRUE(isRearrangement(n3, en3, 1u));
  

  EXPECT_EQ(0u, m.containingRegion(Cell2D(3,0)));
  EXPECT_EQ(2u, m.containingRegion(Cell2D(4,3)));
  EXPECT_EQ(1u, m.containingRegion(Cell2D(4,-1)));

  m.removeRegion(1);
  try {
    m.removeRegion(1);
    ADD_FAILURE() << "Expected UnknownRegionException didn't happen";
  }
  catch (topological_map::UnknownRegionException& e) {}

  EXPECT_EQ(2u, m.allRegions().size());
  n1=m.neighbors(0);
  n3=m.neighbors(2);

  RegionId en12[2]={2};
  EXPECT_TRUE(isRearrangement(n1, en12, 1));
  EXPECT_TRUE(isRearrangement(n3, en3, 1));

  MutableRegionPtr r5(new Region);
  for (c=-10; c<10; c++) {
    r5->insert(Cell2D(5,c));
  }
  EXPECT_EQ(3u, m.addRegion(r5,1));
  n1=m.neighbors(0);
  n3=m.neighbors(2);
  RegionIdVector n5=m.neighbors(3);
  RegionId en32[2]={0,3};
  RegionId en5[1]={2};
  EXPECT_TRUE(isRearrangement(n1,en12,1));
  EXPECT_TRUE(isRearrangement(n3,en32,2));
  EXPECT_TRUE(isRearrangement(n5,en5,1));
  EXPECT_EQ(3u, m.allRegions().size());
}

int main (int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
